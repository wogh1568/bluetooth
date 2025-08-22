#include "wifi.h"
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"

#include "esp_heap_caps.h"   // PSRAM 할당

#define TAG "WIFI_TASK"

// ===== 내부 보관 버퍼(뮤텍스 보호) =====
static _TPM_Info       s_w_tpm_info;
static _TPM_Power_Info s_w_tpm_power_info;
static _TPM_RAW         s_w_adc;

// 갱신 플래그
static bool s_flag_tpm_info  = false;
static bool s_flag_power     = false;
static bool s_flag_adc       = false;

// 보호용 mutex
static SemaphoreHandle_t s_mutex = NULL;

// ===== Wi-Fi 이벤트 콜백 =====
static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data)
{
    switch (id) {
    case WIFI_EVENT_STA_START:
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi start → connect");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGW(TAG, "WiFi disconnected → reconnect");
        esp_wifi_connect();
        break;
    case IP_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "Got IP");
        break;
    default:
        break;
    }
}

// ===== 초기화 =====
void wifi_init_start(void)
{
    // (보통 app_main에서 nvs_flash_init() 수행) 안전하게 한번 더
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t sta_cfg = { 0 };
    strncpy((char*)sta_cfg.sta.ssid, WIFI_SSID, sizeof(sta_cfg.sta.ssid));
    strncpy((char*)sta_cfg.sta.password, WIFI_PASSWORD, sizeof(sta_cfg.sta.password));
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;  // 필요시 WPA3 설정 가능

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 리소스
    if (!s_mutex) s_mutex = xSemaphoreCreateMutex();
}

// ===== UART task가 호출하는 데이터 업데이트 함수들 =====
void wifi_post_tpm_info(const _TPM_Info *p)
{
    if (!p) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_w_tpm_info = *p;         // 작은 구조체 복사
    s_flag_tpm_info = true;
    xSemaphoreGive(s_mutex);
}

void wifi_post_power_info(const _TPM_Power_Info *p)
{
    if (!p) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_w_tpm_power_info = *p;   // 수백 바이트 → 문제 없음
    s_flag_power = true;
    xSemaphoreGive(s_mutex);
}

void wifi_post_adc_sensing(const _TPM_RAW *p)
{
    if (!p) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    // 24KB 구조체 복사 (memcpy)
    memcpy(&s_w_adc, p, sizeof(_TPM_RAW));
    s_flag_adc = true;
    xSemaphoreGive(s_mutex);
}

// ===== 프로토콜 유틸 =====
static inline uint8_t addsum_bytes(const uint8_t *d, size_t n)
{
    uint8_t s = 0;
    for (size_t i = 0; i < n; ++i) s += d[i];
    return s;
}

// Wi-Fi/TCP 송신용 프레임 작성 + 송신 (대용량 payload는 PSRAM 버퍼 사용)
static bool wifi_send_frame_tcp(int sock, uint32_t target_id, uint8_t cmd1, uint8_t cmd2,
                                const void *payload, uint16_t payload_len)
{
    size_t frame_len = (size_t)12 + payload_len; // STX..ETX 포함
    uint8_t *buf = (uint8_t*)heap_caps_malloc(frame_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        ESP_LOGE(TAG, "alloc fail %u bytes", (unsigned)frame_len);
        return false;
    }

    size_t i = 0;
    buf[i++] = STX;
    buf[i++] = DEV_TYPE_TX;                   // ESP32 → Server(CM4)
    buf[i++] = (target_id >> 24) & 0xFF;
    buf[i++] = (target_id >> 16) & 0xFF;
    buf[i++] = (target_id >>  8) & 0xFF;
    buf[i++] = (target_id >>  0) & 0xFF;
    buf[i++] = cmd1;                          // 'S'
    buf[i++] = cmd2;                          // 0x00/0x01/0x02
    buf[i++] = (payload_len >> 8) & 0xFF;
    buf[i++] = (payload_len >> 0) & 0xFF;

    if (payload_len && payload) {
        memcpy(&buf[i], payload, payload_len);
    }
    i += payload_len;

    uint8_t sum = addsum_bytes(buf, 10 + payload_len);
    buf[i++] = sum;
    buf[i++] = ETX;

    // 송신 (부분 전송 대비해서 루프)
    size_t sent = 0;
    while (sent < i) {
        int w = send(sock, buf + sent, i - sent, 0);
        if (w <= 0) { free(buf); return false; }
        sent += (size_t)w;
    }

    free(buf);
    return true;
}

// ===== 서버 수신(선택) — 프로토콜 검사 후 필요시 버퍼 보관 (여기선 드롭) =====
static void tcp_receive_drain(int sock)
{
    uint8_t tmp[1024];
    int r = recv(sock, tmp, sizeof(tmp), MSG_DONTWAIT);
    (void)r; // 필요하다면 추후 FSM 추가
}

// ===== TCP 연결 헬퍼 =====
static int tcp_connect_blocking(void)
{
    int sock = -1;
    struct sockaddr_in dest = {0};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(WIFI_SERVER_PORT);
    dest.sin_addr.s_addr = inet_addr(WIFI_SERVER_IP);
    if (dest.sin_addr.s_addr == INADDR_NONE) {
        ESP_LOGE(TAG, "invalid SERVER_IP");
        return -1;
    }

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket err");
        return -1;
    }

    // 5초 타임아웃
    struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (connect(sock, (struct sockaddr*)&dest, sizeof(dest)) < 0) {
        ESP_LOGW(TAG, "connect fail");
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "TCP connected to %s:%d", WIFI_SERVER_IP, WIFI_SERVER_PORT);
    return sock;
}

// ===== 메인 태스크 =====
void wifi_task(void *arg)
{
    // Wi-Fi 초기화(이미 호출했다면 중복 호출해도 안전)
    wifi_init_start();

    int sock = -1;
    TickType_t last_retry = 0;

    for (;;) {
        // Wi-Fi 연결 상태에서만 TCP 연결 시도
        wifi_ap_record_t ap;
        bool wifi_up = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK);

        if (!wifi_up) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (sock < 0) {
            TickType_t now = xTaskGetTickCount();
            if (now - last_retry > pdMS_TO_TICKS(1000)) {
                sock = tcp_connect_blocking();
                last_retry = now;
            } else {
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
        }

        if (sock >= 0) {
            // 갱신 플래그 확인 → 순차 전송: INFO → POWER → ADC
            bool sent_any = false;

            // [수정 포인트 시작]
            _TPM_Info       *info_cp  = NULL;
            _TPM_Power_Info *power_cp = NULL;
            _TPM_RAW        *adc_cp   = NULL;

            xSemaphoreTake(s_mutex, portMAX_DELAY);
            bool do_info  = s_flag_tpm_info;
            bool do_power = s_flag_power;
            bool do_adc   = s_flag_adc;

            if (do_info) {
                info_cp = (_TPM_Info*)malloc(sizeof(_TPM_Info));
                if (info_cp) { *info_cp = s_w_tpm_info; s_flag_tpm_info = false; }
            }
            if (do_power) {
                power_cp = (_TPM_Power_Info*)malloc(sizeof(_TPM_Power_Info));
                if (power_cp) { *power_cp = s_w_tpm_power_info; s_flag_power = false; }
            }
            if (do_adc) {
                adc_cp = (_TPM_RAW*)heap_caps_malloc(sizeof(_TPM_RAW),
                                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                if (adc_cp) {
                    memcpy(adc_cp, &s_w_adc, sizeof(_TPM_RAW));
                    s_flag_adc = false;
                }
            }
            xSemaphoreGive(s_mutex);

            // 포인터 할당 실패 시 전송 스킵
            do_info  = do_info  && (info_cp  != NULL);
            do_power = do_power && (power_cp != NULL);
            do_adc   = do_adc   && (adc_cp   != NULL);

            // 1) TPM_Info
            if (do_info) {
                if (!wifi_send_frame_tcp(sock, WIFI_TARGET_ID, CMD1_REQ, CMD2_TPM_INFO,
                                        info_cp, sizeof(*info_cp))) {
                    ESP_LOGW(TAG, "send TPM_Info fail → reconnect");
                    if (info_cp)  free(info_cp);
                    if (power_cp) free(power_cp);
                    if (adc_cp)   free(adc_cp);
                    close(sock); sock = -1; continue;
                }
                sent_any = true;
            }

            // 2) TPM_Power_Info
            if (do_power) {
                if (!wifi_send_frame_tcp(sock, WIFI_TARGET_ID, CMD1_REQ, CMD2_TPM_POWER,
                                        power_cp, sizeof(*power_cp))) {
                    ESP_LOGW(TAG, "send TPM_Power fail → reconnect");
                    if (info_cp)  free(info_cp);
                    if (power_cp) free(power_cp);
                    if (adc_cp)   free(adc_cp);
                    close(sock); sock = -1; continue;
                }
                sent_any = true;
            }

            // 3) ADC Sensing (≈24KB, PSRAM)
            if (do_adc) {
                if (!wifi_send_frame_tcp(sock, WIFI_TARGET_ID, CMD1_REQ, CMD2_ADC_SENSING,
                                        adc_cp, sizeof(*adc_cp))) {
                    ESP_LOGW(TAG, "send ADC fail → reconnect");
                    if (info_cp)  free(info_cp);
                    if (power_cp) free(power_cp);
                    if (adc_cp)   free(adc_cp);
                    close(sock); sock = -1; continue;
                }
                sent_any = true;
            }

            // 임시 버퍼 해제 (성공 시)
            if (info_cp)  free(info_cp);
            if (power_cp) free(power_cp);
            if (adc_cp)   free(adc_cp);

            if (!sent_any) {
                tcp_receive_drain(sock);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            // [수정 포인트 끝]

        } else {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}
