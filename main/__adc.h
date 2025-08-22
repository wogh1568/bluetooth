#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 샘플링/출력 파라미터
#define ADC_SAMPLING_HZ       20      // 20 samples/sec
#define ADC_PRINT_INTERVAL_MS 1000    // 1 sec

// 외부 저항비 (입력전압 복원에 사용)
#define EXT24V_RTOP_KOHM      10.0f
#define EXT24V_RBOT_KOHM      1.5f
#define EXT24V_GAIN           ((EXT24V_RTOP_KOHM + EXT24V_RBOT_KOHM) / EXT24V_RBOT_KOHM)  // 11.5/1.5=7.6667

#define BAT_RTOP_KOHM         100.0f
#define BAT_RBOT_KOHM         200.0f
#define BAT_GAIN              ((BAT_RTOP_KOHM + BAT_RBOT_KOHM) / BAT_RBOT_KOHM)           // (100+200)/200=1.5

// 외부 기준전압 (ADC2에 인가된 기준)
#define EXT_REF_VOLT          1.225f   // Volts
#define REF_CAL_SAMPLES       40

// 태스크 시작
void adc_init(void);
void adc_task(void *arg);

// Thread-safe getters (true=유효값)
bool adc_get_ext24v(float *out_volts);
bool adc_get_bat_volt(float *out_volts);
bool adc_get_ref_avg_raw(uint32_t *out_raw_avg);
bool adc_get_volt_per_count(float *out_vpc);

// 오류 카운터 조회
uint32_t adc_get_error_count(void);

#ifdef __cplusplus
}
#endif
