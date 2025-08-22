#include "adc.h"
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
//#include "driver/adc_types.h"
#include "driver/adc.h"
#include "esp_check.h"

// ===== 로깅 =====
#define TAG "ADC_TASK"

// ===== 채널 매핑 (ESP32-S3 가정: ADC1_CH0=GPIO1, CH1=GPIO2, CH2=GPIO3) =====
#define ADC_UNIT            ADC_UNIT_1
#define ADC_CH_EXT24V       ADC_CHANNEL_0   // GPIO1
#define ADC_CH_REF1225      ADC_CHANNEL_1   // GPIO2
#define ADC_CH_BAT          ADC_CHANNEL_2   // GPIO3
// 모든 채널 동일 atten으로 운용 (24V 분압 Vadc 최대≈3.13V → 11dB 권장)
#define ADC_ATTEN           ADC_ATTEN_DB_11 // ~3.55V full-scale
#define ADC_WIDTH           ADC_WIDTH_BIT_12

// ===== 내부 상태 =====
static SemaphoreHandle_t s_mutex = NULL;

static float    s_volt_per_count = 0.0f;  // 초기화 때 1회만 계산/고정
static float    s_ext24v_volt     = 0.0f; // 복원된 입력 전압[V]
static float    s_bat_volt        = 0.0f; // 복원된 입력 전압[V]
static uint32_t s_ref_raw_avg_1s  = 0;    // 직전 1초 동안의 평균 Raw(표시용)

static uint32_t s_err_count       = 0;

// ===== 유틸: 보호 잠금 =====
static inline void lock(void)   { if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY); }
static inline void unlock(void) { if (s_mutex) xSemaphoreGive(s_mutex); }

// ===== 원샷 읽기 (재시도 포함) =====
// static esp_err_t adc_read_raw_retry(adc_unit_t unit, adc_channel_t ch, int *out_raw)
// {
//     esp_err_t ret = ESP_OK;
//     for (int i = 0; i < 3; ++i) {
//         ret = adc1_get_raw((adc1_channel_t)ch);
//         if (ret >= 0) { // adc1_get_raw returns int; <0 on error
//             *out_raw = ret;
//             return ESP_OK;
//         }
//         vTaskDelay(pdMS_TO_TICKS(1));
//     }
//     return ESP_FAIL;
// }

// replace this function in adc.c
static bool adc_read_raw_retry(adc_unit_t unit, adc_channel_t ch, int *out_raw)
{
    (void)unit; // unused with ADC1 legacy API
    for (int i = 0; i < 3; ++i) {
        int v = adc1_get_raw((adc1_channel_t)ch);
        if (v >= 0) { *out_raw = v; return true; }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

// ===== 초기 캘리브레이션: ADC2(1.225V) 40회 평균으로 volt_per_count 산출 =====
static bool adc_calibrate_from_ref(void)
{
    uint64_t acc = 0;
    int raw = 0;
    int valid = 0;

    for (int i = 0; i < REF_CAL_SAMPLES; ++i) {
        if (adc_read_raw_retry(ADC_UNIT, ADC_CH_REF1225, &raw) == ESP_OK) {
            if (raw > 0) { acc += (uint32_t)raw; valid++; }
        } else {
            s_err_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(2)); // 약간 텀
    }

    if (valid == 0) {
        ESP_LOGE(TAG, "ADC ref calibration failed (no valid samples)");
        return false;
    }

    float avg_raw = (float)(acc / valid);
    // 비트당 전압[V/count]
    s_volt_per_count = EXT_REF_VOLT / avg_raw;

    ESP_LOGI(TAG, "ADC calib: ref_avg_raw=%.1f → volt_per_count=%.6f V/LSB",
             avg_raw, s_volt_per_count);
    return true;
}

// ===== 공통 전압 환산: Raw → Vadc → Vin(분압 보정) =====
static inline float raw_to_vadc(int raw)
{
    return (float)raw * s_volt_per_count; // V/LSB * count
}

static inline float vadc_to_ext24v(float vadc) { return vadc * EXT24V_GAIN; }
static inline float vadc_to_bat   (float vadc) { return vadc * BAT_GAIN; }

// ===== 초기화 =====
void adc_init(void)
{
    if (!s_mutex) s_mutex = xSemaphoreCreateMutex();

    // ADC1 구성
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten((adc1_channel_t)ADC_CH_EXT24V, ADC_ATTEN);
    adc1_config_channel_atten((adc1_channel_t)ADC_CH_REF1225, ADC_ATTEN);
    adc1_config_channel_atten((adc1_channel_t)ADC_CH_BAT,    ADC_ATTEN);

    // 초기 캘리브레이션 (1회)
    if (!adc_calibrate_from_ref()) {
        // 안전한 기본값 (3.3V / 4095) — 실제와 달라질 수 있음
        s_volt_per_count = 3.3f / 4095.0f;
        ESP_LOGW(TAG, "Fallback volt_per_count=%.6f", s_volt_per_count);
    }
}

// ===== 태스크 =====
void adc_task(void *arg)
{
    (void)arg;
    const TickType_t sample_period = pdMS_TO_TICKS(1000 / ADC_SAMPLING_HZ); // 50ms
    const TickType_t print_period  = pdMS_TO_TICKS(ADC_PRINT_INTERVAL_MS);

    TickType_t t0 = xTaskGetTickCount();
    uint64_t ref_acc = 0;
    uint32_t ref_cnt = 0;

    for (;;) {
        // 1) 샘플링
        int raw_ext = 0, raw_ref = 0, raw_bat = 0;
        if (adc_read_raw_retry(ADC_UNIT, ADC_CH_EXT24V, &raw_ext) != ESP_OK) { s_err_count++; }
        if (adc_read_raw_retry(ADC_UNIT, ADC_CH_REF1225, &raw_ref) != ESP_OK) { s_err_count++; }
        if (adc_read_raw_retry(ADC_UNIT, ADC_CH_BAT,    &raw_bat) != ESP_OK) { s_err_count++; }

        // 2) 전압 복원
        float v_ext = vadc_to_ext24v(raw_to_vadc(raw_ext));
        float v_bat = vadc_to_bat   (raw_to_vadc(raw_bat));

        // 3) 공유값 갱신 (마지막 샘플 값 보관)
        lock();
        s_ext24v_volt = v_ext;
        s_bat_volt    = v_bat;
        unlock();

        // 4) 1초 평균용 REF 누적(raw)
        if (raw_ref > 0) { ref_acc += (uint32_t)raw_ref; ref_cnt++; }

        // 5) 1초마다 출력
        if ((xTaskGetTickCount() - t0) >= print_period) {
            uint32_t ref_avg = (ref_cnt > 0) ? (uint32_t)(ref_acc / ref_cnt) : 0;
            float    ref_v   = raw_to_vadc((int)ref_avg);

            lock();
            s_ref_raw_avg_1s = ref_avg;
            float ext_v = s_ext24v_volt;
            float bat_v = s_bat_volt;
            unlock();

            ESP_LOGI(TAG, "EXT24V=%.3f V | BAT=%.3f V | REF(avg1s) raw=%" PRIu32 " (%.4f V)",
                     ext_v, bat_v, ref_avg, ref_v);

            // 다음 1초 준비
            t0 = xTaskGetTickCount();
            ref_acc = 0; ref_cnt = 0;
        }

        vTaskDelay(sample_period);
    }
}

// ===== Getters =====
bool adc_get_ext24v(float *out_volts)
{
    if (!out_volts) return false;
    lock();
    *out_volts = s_ext24v_volt;
    unlock();
    return true;
}
bool adc_get_bat_volt(float *out_volts)
{
    if (!out_volts) return false;
    lock();
    *out_volts = s_bat_volt;
    unlock();
    return true;
}
bool adc_get_ref_avg_raw(uint32_t *out_raw_avg)
{
    if (!out_raw_avg) return false;
    lock();
    *out_raw_avg = s_ref_raw_avg_1s;
    unlock();
    return true;
}
bool adc_get_volt_per_count(float *out_vpc)
{
    if (!out_vpc) return false;
    lock();
    *out_vpc = s_volt_per_count;
    unlock();
    return true;
}
uint32_t adc_get_error_count(void)
{
    return s_err_count;
}
