#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include "driver/gpio.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// ===== 핀/포트 설정 =====
#define I2C_PORT            I2C_NUM_0
#define I2C_SDA_GPIO        (GPIO_NUM_9)
#define I2C_SCL_GPIO        (GPIO_NUM_14)
#define I2C_FREQ_HZ         (400000)     // 100k로 낮추려면 바꾸세요.

// ===== 디바이스 주소 =====
#define SHT20_ADDR          (0x40)       // 7-bit
#define PCF85063_ADDR       (0x51)       // 7-bit

// ===== 공개 데이터 타입 =====
typedef struct {
    float temperature_c;   // 섭씨
    float humidity_rh;     // %RH
    bool  valid;           // 최신값 유효 여부
    uint32_t err_count;    // 누적 오류 횟수(읽기 실패)
} sht20_data_t;

// 최신 시간의 유효성도 bool로 반환할 수 있으나, 여기서는 함수 성공/실패로 전달

// ===== 초기화/태스크 =====
void i2c_chipset_init(void);     // I2C + 장치 준비 (버스/뮤텍스/최초샘플)
void i2c_chipset_task(void *arg); // 1초 주기 측정/로그 태스크

// ===== 스레드-세이프 API =====
// 최신 온습도 읽기 (복사 반환). 성공 시 true
bool sht20_get_latest(sht20_data_t *out);

// 최신 시간 읽기 (RTC→tm). 성공 시 true
bool rtc_get_time(struct tm *out_tm);

// RTC 시간 설정 (tm→RTC). 성공 시 true
bool rtc_set_time(const struct tm *in_tm);

// (옵션) 오류 카운터 조회
uint32_t i2c_get_bus_reset_count(void);

#ifdef __cplusplus
}
#endif
