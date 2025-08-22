// uart1.c
#include "uart1.h"
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_heap_caps.h"   // PSRAM 할당용

#define TAG "UART1"


// UART task 쪽에서 최신 데이터 전달할 때 호출 (mutex로 보호)
extern void wifi_post_tpm_info(const _TPM_Info *p);
extern void wifi_post_power_info(const _TPM_Power_Info *p);
extern void wifi_post_adc_sensing(const _TPM_RAW *p);

// ===================== 핀/속도 =====================
#define UART_PORT          UART_NUM_1
#define UART_TXD_PIN       (GPIO_NUM_17)
#define UART_RXD_PIN       (GPIO_NUM_18)
#define UART_BAUD          (460800)

// ===================== 드라이버/큐 용량 =====================
// RX 링버퍼는 DRAM에 유지(너무 크게 잡지 않음)
#define UART_RX_BUF_SIZE   (8192)     // 8KB 권장
#define UART_TX_BUF_SIZE   (4096)
#define UART_EVT_QUEUE_LEN (30)

// 대형 payload는 PSRAM에 동적할당 (heap_caps_malloc)
#define UART_MAX_PAYLOAD_HINT (25600) // 25KB 정도를 상한 가이드로(실제 길이는 프레임 len로 결정)

// ===================== 프로토콜 상수 =====================
#define STX                0x02
#define ETX                0x03
#define DEV_TYPE_TX        0xA5   // ESP32->STM32
#define DEV_TYPE_RX        0xA7   // STM32->ESP32
//#define LORA_MODULE_CODE   0xA6
//#define TPM_CODE		   0xA7
#define CMD1_REQ           'S'    // 요청은 'S'
#define CMD1_RSP           's'    // 응답은 's'

// ===================== 전역(수신 결과) =====================
volatile _TPM_Info       g_tpm_info;
volatile _TPM_Power_Info g_tpm_power_info;
//volatile _ADC_Sensing    g_volt_curr_adc;
volatile _TPM_RAW        g_volt_curr_adc;
volatile _IOR_Info       g_ior_info;
volatile _IOR_State      g_ior_state;
volatile _Vo_Info        g_vo_info;
volatile _Vo_State       g_vo_state;


// ===================== 내부 리소스 =====================
static QueueHandle_t    s_uart_evtq   = NULL;  // UART 드라이버 이벤트 큐
static SemaphoreHandle_t s_tx_mutex   = NULL;  // 송신 보호

typedef struct {
    uint8_t  device_type;
    uint8_t  cmd1;
    uint8_t  cmd2;
    uint32_t target_id;
    uint16_t payload_len;
    uint8_t *payload;        // PSRAM에 할당된 버퍼 포인터(소유권 이동)
} rx_frame_t;

static QueueHandle_t s_rx_frame_q = NULL;      // 파싱 완료 프레임 큐

// ===================== 유틸 =====================
static inline uint8_t addsum_bytes(const uint8_t *d, size_t n)
{
    uint8_t s = 0;
    for (size_t i = 0; i < n; ++i) s += d[i];
    return s;
}

// ===================== 송신 =====================
bool uart1_send_frame(uint32_t target_id, uint8_t cmd1, uint8_t cmd2,
                      const void *payload, uint16_t payload_len)
{
    // 프레임 총 길이: 1(STX)+1(dev)+4(tid)+1(cmd1)+1(cmd2)+2(len)+N(payload)+1(cksum)+1(ETX)
    size_t frame_len = (size_t)12 + payload_len;
    uint8_t *buf = (uint8_t *)malloc(frame_len);
    if (!buf) {
        ESP_LOGE(TAG, "tx malloc fail (%u bytes)", (unsigned)frame_len);
        return false;
    }

    size_t i = 0;
    buf[i++] = STX;                       // [0]
    buf[i++] = DEV_TYPE_TX;               // [1]
    buf[i++] = (target_id >> 24) & 0xFF;  // [2]
    buf[i++] = (target_id >> 16) & 0xFF;  // [3]
    buf[i++] = (target_id >>  8) & 0xFF;  // [4]
    buf[i++] = (target_id >>  0) & 0xFF;  // [5]
    buf[i++] = cmd1;                      // [6]
    buf[i++] = cmd2;                      // [7]
    buf[i++] = (payload_len >> 8) & 0xFF; // [8]
    buf[i++] = (payload_len >> 0) & 0xFF; // [9]

    if (payload_len && payload) {
        memcpy(&buf[i], payload, payload_len);
    }
    i += payload_len;

    // 체크섬: STM32 코드 기준 STX 포함하여 [0..(9+payload_len)] 합
    uint8_t chksum = addsum_bytes(buf, 10 + payload_len);
    buf[i++] = chksum;                    // [10+payload]
    buf[i++] = ETX;                       // [11+payload]

    // 송신
    if (s_tx_mutex) xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    int w = uart_write_bytes(UART_PORT, (const char *)buf, i);
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(100));
    if (s_tx_mutex) xSemaphoreGive(s_tx_mutex);

    free(buf);
    bool ok = (w == (int)i);
    if (!ok) ESP_LOGE(TAG, "tx write mismatch w=%d need=%d", w, (int)i);
    return ok;
}

// ===================== 수신 파서(FSM) =====================
typedef enum {
    RX_ST_WAIT_STX = 0,
    RX_ST_FIXED,      // device(1)+tid4+cmd1+cmd2+len2 => 총 9바이트
    RX_ST_PAYLOAD,
    RX_ST_CHKSUM,
    RX_ST_WAIT_ETX
} rx_state_t;

// FSM 내부 상태
static rx_state_t st = RX_ST_WAIT_STX;

// 고정 필드
static uint8_t  dev = 0, cmd1 = 0, cmd2 = 0;
static uint8_t  tid_b[4];
static uint16_t pay_len = 0;

// 페이로드 버퍼(PSRAM)
static uint8_t *s_paybuf = NULL;
static uint16_t s_paypos = 0;

// 체크섬 계산용 "가산기"(STX 포함 누적)
static uint8_t cksum_acc = 0;

// 고정 필드 읽힌 개수(1..9)
static uint8_t fixed_cnt = 0;

static void fsm_reset(void)
{
    st = RX_ST_WAIT_STX;
    dev = cmd1 = cmd2 = 0;
    tid_b[0] = tid_b[1] = tid_b[2] = tid_b[3] = 0;
    pay_len = 0;
    if (s_paybuf) { free(s_paybuf); s_paybuf = NULL; }
    s_paypos = 0;
    cksum_acc = 0;
    fixed_cnt = 0;
}

static void uart1_rx_fsm_run(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        uint8_t b = data[i];

        switch (st) {
        case RX_ST_WAIT_STX:
            if (b == STX) {
                fsm_reset();              // 내부 변수 초기화
                st = RX_ST_FIXED;
                cksum_acc = b;            // STX 포함 누적 시작
                fixed_cnt = 0;
            }
            break;

        case RX_ST_FIXED:
            // device, tid[4], cmd1, cmd2, len_hi, len_lo => 총 9바이트 수집
            fixed_cnt++;
            cksum_acc += b;

            if (fixed_cnt == 1)       dev = b;
            else if (fixed_cnt >= 2 && fixed_cnt <= 5) tid_b[fixed_cnt - 2] = b;
            else if (fixed_cnt == 6)  cmd1 = b;
            else if (fixed_cnt == 7)  cmd2 = b;
            else if (fixed_cnt == 8)  pay_len = ((uint16_t)b) << 8; // len_hi
            else if (fixed_cnt == 9) {
                pay_len |= b;                                 // len_lo
                if (pay_len > 0) {
                    // PSRAM에 payload 크기만큼 할당
                    s_paybuf = (uint8_t *)heap_caps_malloc(pay_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                    if (!s_paybuf) {
                        ESP_LOGE(TAG, "PSRAM alloc fail len=%u", pay_len);
                        fsm_reset();
                        break;
                    }
                    s_paypos = 0;
                    st = RX_ST_PAYLOAD;
                } else {
                    st = RX_ST_CHKSUM;
                }
            }
            break;

        case RX_ST_PAYLOAD:
            if (s_paypos < pay_len) {
                s_paybuf[s_paypos++] = b;
                cksum_acc += b;
                if (s_paypos >= pay_len) {
                    st = RX_ST_CHKSUM;
                }
            } else {
                // 방어적 처리
                ESP_LOGW(TAG, "payload overflow");
            }
            break;

        case RX_ST_CHKSUM:
            // 누적합과 비교
            if (cksum_acc != b) {
                ESP_LOGW(TAG, "checksum mismatch calc=0x%02X rx=0x%02X (len=%u)", cksum_acc, b, pay_len);
                fsm_reset();
                break;
            }
            st = RX_ST_WAIT_ETX;
            break;

        case RX_ST_WAIT_ETX:
            if (b == ETX) {
                rx_frame_t frm = {0};
                frm.device_type = dev;
                frm.cmd1 = cmd1;
                frm.cmd2 = cmd2;
                frm.target_id = ((uint32_t)tid_b[0] << 24) | ((uint32_t)tid_b[1] << 16) |
                                ((uint32_t)tid_b[2] << 8)  | ((uint32_t)tid_b[3]);
                frm.payload_len = pay_len;
                frm.payload = s_paybuf;   // 소유권 이전
                s_paybuf = NULL;          // 이 FSM에서는 더 이상 해제하지 않음

                if (xQueueSend(s_rx_frame_q, &frm, 0) != pdTRUE) {
                    // 큐가 가득이면 메모리 누수 방지
                    if (frm.payload) free(frm.payload);
                    ESP_LOGW(TAG, "rx_frame_q full, drop frame");
                }
            }
            // 프레임 종료 후 리셋
            fsm_reset();
            break;
        }
    }
}

// ===================== UART 이벤트 루프 =====================
static void uart1_event_loop(void)
{
    uart_event_t evt;
    uint8_t *rb = (uint8_t *) malloc(UART_RX_BUF_SIZE);
    if (!rb) {
        ESP_LOGE(TAG, "rx temp buf malloc fail");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        if (xQueueReceive(s_uart_evtq, &evt, portMAX_DELAY)) {
            switch (evt.type) {
            case UART_DATA: {
                int r = uart_read_bytes(UART_PORT, rb, evt.size, pdMS_TO_TICKS(20));
                if (r > 0) {
                    uart1_rx_fsm_run(rb, (size_t)r);
                }
                break;
            }
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "RX overflow (%d), flush", evt.type);
                uart_flush_input(UART_PORT);
                xQueueReset(s_uart_evtq);
                fsm_reset();
                break;
            case UART_BREAK:
            case UART_PARITY_ERR:
            case UART_FRAME_ERR:
            default:
                // 필요 시 추가 처리
                break;
            }
        }
    }
}

// FreeRTOS 태스크 엔트리
static void uart1_evt_task(void *arg)
{
    (void)arg;
    uart1_event_loop();   // 블로킹
    vTaskDelete(NULL);
}

// ===================== 공개 API =====================
void uart1_init(void)
{
    // 드라이버 설치
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE,
                                        UART_EVT_QUEUE_LEN, &s_uart_evtq, 0));

    uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,   // RTS/CTS 배선 가능하면 ENABLE 권장
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TXD_PIN, UART_RXD_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // (옵션) RX 타임아웃 튜닝
    // uart_set_rx_timeout(UART_PORT, 2);

    s_tx_mutex   = xSemaphoreCreateMutex();
    s_rx_frame_q = xQueueCreate(4, sizeof(rx_frame_t));

    // 이벤트 루프 태스크 시작
    xTaskCreatePinnedToCore(uart1_evt_task, "uart1_evt", 4096, NULL, 11, NULL, tskNO_AFFINITY);
}

// 요청/응답 (blocking)
// cmd2 = 0x00 -> TPM_Info, 0x01 -> TPM_Power_Info
bool uart1_request_info(uint32_t target_id, uint8_t cmd2, uint32_t timeout_ms)
{
    // 요청 송신: cmd1='S', payload 없음
    if (!uart1_send_frame(target_id, CMD1_REQ, cmd2, NULL, 0)) {
        ESP_LOGE(TAG, "send failed");
        return false;
    }

    rx_frame_t frm;
    TickType_t t_end = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (xTaskGetTickCount() < t_end) {
        if (xQueueReceive(s_rx_frame_q, &frm, pdMS_TO_TICKS(50)) == pdTRUE) {
            bool consumed = false;

            if (frm.device_type == DEV_TYPE_RX && frm.cmd1 == CMD1_RSP) {
                if (cmd2 == 0x00) {
                    if (frm.payload && frm.payload_len >= sizeof(_TPM_Info)) {
                        memcpy((void *)&g_tpm_info, frm.payload, sizeof(_TPM_Info));
                        consumed = true;
                    }
                } else if (cmd2 == 0x01) {
                    if (frm.payload && frm.payload_len >= sizeof(_TPM_Power_Info)) {
                        memcpy((void *)&g_tpm_power_info, frm.payload, sizeof(_TPM_Power_Info));
                        consumed = true;
                    }
                } else if (cmd2 == 0x02) {
                    if (frm.payload && frm.payload_len >= sizeof(_TPM_RAW)) {
                        memcpy((void *)&g_volt_curr_adc, frm.payload, sizeof(_TPM_RAW));
                        consumed = true;
                    } //volatile _TPM_RAW        g_volt_curr_adc;
                }
            }

            if (frm.payload) free(frm.payload);
            if (consumed) return true;
        }
    }

    ESP_LOGE(TAG, "response timeout (cmd2=0x%02X)", cmd2);
    return false;
}


// 데모 태스크: 0x00/0x01 요청하고 로그 출력
void uart1_task(void *arg)
{
    uint16_t ok_count=0;
    uint16_t fail_count=0;
    const uint32_t TARGET_ID = 0x11223344; // 예시
    vTaskDelay(pdMS_TO_TICKS(500)); // 초기 안정화 대기



    for(;;){
        // 1) TPM_Info 요청
        if (uart1_request_info(TARGET_ID, 0x00, 3000)) {
            const _TPM_Info *p = (const _TPM_Info *)&g_tpm_info;
            ESP_LOGI(TAG, "[TPM_Info] sn=%.8s hw=%d sw=%d ct_ratio=%d",
                    p->sn, p->hw_ver, p->sw_ver, p->ct_ratio);
            wifi_post_tpm_info((const _TPM_Info *)&g_tpm_info);
            ok_count++;
        } else {
            ESP_LOGE(TAG, "TPM_Info request failed");
            fail_count++;
        }
        // TPM_Info 수신 후
        vTaskDelay(pdMS_TO_TICKS(100));

        // 2) TPM_Power_Info 요청
        if (uart1_request_info(TARGET_ID, 0x01, 5000)) {
            const _TPM_Power_Info *q = (const _TPM_Power_Info *)&g_tpm_power_info;
            ESP_LOGI(TAG, "[TPM_Power] state=%d run_min=%d bat=%d L1Vrms=%.4f L1Arms=%.4f PF1=%.4f",
                    q->state, q->run_time, q->bat_volt, q->L1V_rms, q->L1A_rms, q->L1_PowerFactor);
            // 필요 시 더 많은 필드 로그 추가
            wifi_post_power_info((const _TPM_Power_Info *)&g_tpm_power_info);        
            ok_count++;
        } else {
            ESP_LOGE(TAG, "TPM_Power_Info request failed");
            fail_count++;
        }
        // TPM_Power_Info 수신 후
        vTaskDelay(pdMS_TO_TICKS(100));

        // 3) adc raw data 요청  _TPM_RAW
        if (uart1_request_info(TARGET_ID, 0x02, 3000)) {
            const _TPM_RAW *q = (const _TPM_RAW*)&g_volt_curr_adc;
            ESP_LOGI(TAG, "[TPM_ADC] adc-VL1=%d", q->L1V[0] );
            wifi_post_adc_sensing((const _TPM_RAW *)&g_volt_curr_adc);        
            // 필요하면 나머지도 추가 출력
            ok_count++;
        } else {
            ESP_LOGE(TAG, "ADC_Sensing request failed");
            fail_count++;
        }
        // ADC Sensing 수신 후 (24KB)
        ESP_LOGW(TAG, "[OK=%d, FAIL=%d]", ok_count, fail_count);
        vTaskDelay(pdMS_TO_TICKS(1000));        

    }

    vTaskDelete(NULL);
}
