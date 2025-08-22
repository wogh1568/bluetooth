#include "i2c_chipset.h"
#include <string.h>
#include <inttypes.h>   // ← printf용 PRIu32 등

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_check.h"

// ================= 공통 설정/리트라이 =================
#define TAG "I2C_CHIPSET"

#define I2C_OP_TIMEOUT_MS   (120)     // 각 트랜잭션 타임아웃
#define I2C_RETRY_MAX       (3)       // 장치 단위 동작 재시도 횟수
#define I2C_ERR_RESET_THRES (5)       // 연속 오류 누적 시 버스 재초기화

// ================= 내부 상태 =================
static SemaphoreHandle_t s_i2c_mutex  = NULL;  // I2C 버스 보호
static SemaphoreHandle_t s_data_mutex = NULL;  // 공유 데이터 보호

static sht20_data_t s_sht20_latest = {0};
static struct tm    s_rtc_latest   = {0};

static uint32_t s_consecutive_i2c_errors = 0;  // 연속 실패 카운트
static uint32_t s_bus_reset_count        = 0;  // 버스 재초기화 누적

// ================= 유틸: I2C 버스 (재)설치 =================
static esp_err_t i2c_bus_install(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num    = I2C_SDA_GPIO,
        .scl_io_num    = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,  // 외부 풀업 권장, 내부는 보조
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
        .clk_flags = 0,
    };
    ESP_RETURN_ON_ERROR(i2c_param_config(I2C_PORT, &conf), TAG, "i2c_param_config");
    ESP_RETURN_ON_ERROR(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0), TAG, "i2c_driver_install");
    return ESP_OK;
}

static void i2c_bus_uninstall(void)
{
    i2c_driver_delete(I2C_PORT);
}

static void i2c_bus_reset(void)
{
    i2c_bus_uninstall();
    // 짧게 쉼 후 재설치
    vTaskDelay(pdMS_TO_TICKS(10));
    if (i2c_bus_install() == ESP_OK) {
        s_consecutive_i2c_errors = 0;
        s_bus_reset_count++;
        //ESP_LOGW(TAG, "I2C bus re-initialized (count=%u)", s_bus_reset_count);
        ESP_LOGW(TAG, "I2C bus re-initialized (count=%" PRIu32 ")", s_bus_reset_count);
    } else {
        ESP_LOGE(TAG, "I2C bus re-init failed");
    }
}

// 공통 I2C 전송/수신(리트라이 + 연속오류 관리)
static esp_err_t i2c_txrx(uint8_t addr7,
                          const uint8_t *w, size_t wlen,
                          uint8_t *r, size_t rlen,
                          uint32_t timeout_ms)
{
    esp_err_t ret = ESP_FAIL;
    for (int attempt = 1; attempt <= I2C_RETRY_MAX; ++attempt) {
        // write-only / read-only / write-then-read 로 처리
        if (w && wlen && r && rlen) {
            ret = i2c_master_write_read_device(I2C_PORT, addr7, w, wlen, r, rlen, pdMS_TO_TICKS(timeout_ms));
        } else if (w && wlen) {
            ret = i2c_master_write_to_device(I2C_PORT, addr7, w, wlen, pdMS_TO_TICKS(timeout_ms));
        } else if (r && rlen) {
            ret = i2c_master_read_from_device(I2C_PORT, addr7, r, rlen, pdMS_TO_TICKS(timeout_ms));
        } else {
            return ESP_ERR_INVALID_ARG;
        }

        if (ret == ESP_OK) {
            s_consecutive_i2c_errors = 0;
            return ESP_OK;
        }

        ESP_LOGW(TAG, "I2C op fail (addr=0x%02X, try=%d/%d): %s",
                 addr7, attempt, I2C_RETRY_MAX, esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // 연속 실패 증가 및 필요 시 버스 리셋
    s_consecutive_i2c_errors++;
    if (s_consecutive_i2c_errors >= I2C_ERR_RESET_THRES) {
        //ESP_LOGW(TAG, "I2C consecutive errors=%u >= %u → bus reset",
        //         s_consecutive_i2c_errors, I2C_ERR_RESET_THRES);
        // after (둘 다 32비트로 맞춰 출력)
        ESP_LOGW(TAG, "I2C consecutive errors=%" PRIu32 " >= %" PRIu32 " → bus reset",
                s_consecutive_i2c_errors, (uint32_t)I2C_ERR_RESET_THRES);                 
        i2c_bus_reset();
    }
    return ret;
}

// ================= SHT20 드라이버 (예외+CRC) =================
#define SHT20_CMD_TRIG_TEMP_NOHOLD  0xF3
#define SHT20_CMD_TRIG_RH_NOHOLD    0xF5
#define SHT20_CMD_SOFT_RESET        0xFE

// CRC-8 다항식 0x31, init 0x00 (Sensirion 표준)
static uint8_t sht_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x31;
            else            crc <<= 1;
        }
    }
    return crc;
}

static float sht20_raw_to_temp(uint16_t raw)
{
    return -46.85f + (175.72f * (float)raw) / 65536.0f;
}
static float sht20_raw_to_rh(uint16_t raw)
{
    float rh = -6.0f + (125.0f * (float)raw) / 65536.0f;
    if (rh < 0) rh = 0;
    if (rh > 100) rh = 100;
    return rh;
}

static bool sht20_soft_reset_locked(void)
{
    uint8_t cmd = SHT20_CMD_SOFT_RESET;
    return (i2c_txrx(SHT20_ADDR, &cmd, 1, NULL, 0, I2C_OP_TIMEOUT_MS) == ESP_OK);
}

// no-hold: 명령 전송 → 변환시간 대기 → 3바이트 읽기 + CRC 검사
static esp_err_t sht20_measure_nohold_locked(uint8_t cmd, uint16_t *out_raw)
{
    if (!out_raw) return ESP_ERR_INVALID_ARG;

    // 1) command write
    esp_err_t ret = i2c_txrx(SHT20_ADDR, &cmd, 1, NULL, 0, I2C_OP_TIMEOUT_MS);
    if (ret != ESP_OK) return ret;

    // 2) conversion wait (최대치 여유)
    //   (데이터시트: typ 85ms@RH, 85ms@T — 노이즈 환경 고려해 100ms)
    vTaskDelay(pdMS_TO_TICKS(100));

    // 3) read 3 bytes (MSB, LSB, CRC)
    uint8_t buf[3] = {0};
    ret = i2c_txrx(SHT20_ADDR, NULL, 0, buf, 3, I2C_OP_TIMEOUT_MS);
    if (ret != ESP_OK) return ret;

    // 4) CRC check
    uint8_t crc = sht_crc8(buf, 2);
    if (crc != buf[2]) {
        ESP_LOGW(TAG, "SHT20 CRC mismatch: calc=0x%02X rx=0x%02X", crc, buf[2]);
        return ESP_FAIL;
    }

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    raw &= ~0x0003; // 상태비트 제거
    *out_raw = raw;
    return ESP_OK;
}

static bool sht20_read_locked(float *out_t, float *out_rh)
{
    // I2C_MUTEX를 이미 잡고 호출
    uint16_t rt = 0, rrh = 0;
    if (sht20_measure_nohold_locked(SHT20_CMD_TRIG_TEMP_NOHOLD, &rt) != ESP_OK) return false;
    if (sht20_measure_nohold_locked(SHT20_CMD_TRIG_RH_NOHOLD,   &rrh) != ESP_OK) return false;

    if (out_t)  *out_t  = sht20_raw_to_temp(rt);
    if (out_rh) *out_rh = sht20_raw_to_rh(rrh);
    return true;
}

// ================= PCF85063AT 드라이버 (예외) =================
#define PCF_REG_CTRL1      0x00
#define PCF_REG_CTRL2      0x01
#define PCF_REG_SEC        0x04
#define PCF_REG_MIN        0x05
#define PCF_REG_HOUR       0x06
#define PCF_REG_DAY        0x07
#define PCF_REG_WEEKDAY    0x08
#define PCF_REG_MONTH      0x09
#define PCF_REG_YEAR       0x0A

static inline uint8_t to_bcd(uint8_t v)   { return (uint8_t)(((v/10)<<4) | (v%10)); }
static inline uint8_t from_bcd(uint8_t b) { return (uint8_t)((((b)>>4)&0x0F) *10 + ((b)&0x0F)); }

static bool pcf_read_reg_locked(uint8_t reg, uint8_t *out, size_t n)
{
    return (i2c_txrx(PCF85063_ADDR, &reg, 1, out, n, I2C_OP_TIMEOUT_MS) == ESP_OK);
}
static bool pcf_write_reg_locked(uint8_t reg, const uint8_t *data, size_t n)
{
    uint8_t buf[1+7]; // 최대 7바이트 연속쓰기(시/분/초/…)
    if (n > 7) return false;
    buf[0] = reg;
    memcpy(&buf[1], data, n);
    return (i2c_txrx(PCF85063_ADDR, buf, n+1, NULL, 0, I2C_OP_TIMEOUT_MS) == ESP_OK);
}

static bool pcf_start_oscillator_locked(void)
{
    uint8_t ctrl1 = 0;
    if (!pcf_read_reg_locked(PCF_REG_CTRL1, &ctrl1, 1)) return false;
    ctrl1 &= ~(1<<5); // STOP=0
    return pcf_write_reg_locked(PCF_REG_CTRL1, &ctrl1, 1);
}

static bool pcf_set_time_tm_locked(const struct tm *tmin)
{
    if (!tmin) return false;
    uint8_t b[7];
    b[0] = to_bcd((uint8_t)tmin->tm_sec)   & 0x7F;
    b[1] = to_bcd((uint8_t)tmin->tm_min)   & 0x7F;
    b[2] = to_bcd((uint8_t)tmin->tm_hour)  & 0x3F;
    b[3] = to_bcd((uint8_t)tmin->tm_mday)  & 0x3F;
    b[4] = to_bcd((uint8_t)tmin->tm_wday)  & 0x07;
    b[5] = to_bcd((uint8_t)(tmin->tm_mon + 1)) & 0x1F;
    b[6] = to_bcd((uint8_t)(tmin->tm_year % 100));
    return pcf_write_reg_locked(PCF_REG_SEC, b, sizeof(b));
}

static bool pcf_get_time_tm_locked(struct tm *out)
{
    if (!out) return false;
    uint8_t b[7];
    if (!pcf_read_reg_locked(PCF_REG_SEC, b, sizeof(b))) return false;

    memset(out, 0, sizeof(*out));
    out->tm_sec  = from_bcd(b[0] & 0x7F);
    out->tm_min  = from_bcd(b[1] & 0x7F);
    out->tm_hour = from_bcd(b[2] & 0x3F);
    out->tm_mday = from_bcd(b[3] & 0x3F);
    out->tm_wday = from_bcd(b[4] & 0x07);
    out->tm_mon  = from_bcd(b[5] & 0x1F) - 1;
    out->tm_year = from_bcd(b[6]) + 100; // 2000~2099
    return true;
}

// ================= 공개 API 구현 =================
void i2c_chipset_init(void)
{
    if (!s_i2c_mutex)  s_i2c_mutex  = xSemaphoreCreateMutex();
    if (!s_data_mutex) s_data_mutex = xSemaphoreCreateMutex();

    // I2C 버스 설치
    ESP_ERROR_CHECK(i2c_bus_install());

    // 장치 초기화(뮤텍스 보호)
    xSemaphoreTake(s_i2c_mutex, portMAX_DELAY);
    (void)sht20_soft_reset_locked();
    (void)pcf_start_oscillator_locked();
    xSemaphoreGive(s_i2c_mutex);

    // 최초 샘플 시도 (실패해도 무시)
    float t=0, rh=0;
    struct tm tm_now;
    bool ok_t  = false, ok_tm = false;

    xSemaphoreTake(s_i2c_mutex, portMAX_DELAY);
    ok_t  = sht20_read_locked(&t, &rh);
    ok_tm = pcf_get_time_tm_locked(&tm_now);
    xSemaphoreGive(s_i2c_mutex);

    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    if (ok_t) {
        s_sht20_latest.temperature_c = t;
        s_sht20_latest.humidity_rh   = rh;
        s_sht20_latest.valid = true;
    } else {
        s_sht20_latest.valid = false;
    }
    if (ok_tm) s_rtc_latest = tm_now;
    xSemaphoreGive(s_data_mutex);
}

void i2c_chipset_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(1000);

    for (;;) {
        bool ok_t=false, ok_tm=false;
        float t=0, rh=0;
        struct tm tm_now;

        // ==== 측정 (I2C 보호) ====
        xSemaphoreTake(s_i2c_mutex, portMAX_DELAY);
        ok_t  = sht20_read_locked(&t, &rh);
        ok_tm = pcf_get_time_tm_locked(&tm_now);
        xSemaphoreGive(s_i2c_mutex);

        // ==== 캐시 갱신 (데이터 보호) ====
        xSemaphoreTake(s_data_mutex, portMAX_DELAY);
        if (ok_t) {
            s_sht20_latest.temperature_c = t;
            s_sht20_latest.humidity_rh   = rh;
            s_sht20_latest.valid = true;
        } else {
            s_sht20_latest.err_count++;
            s_sht20_latest.valid = false; // 실패 시 이전값 유지하고 valid=false
        }
        if (ok_tm) {
            s_rtc_latest = tm_now;
        }
        xSemaphoreGive(s_data_mutex);

        // ==== 로그 ====
        ESP_LOGI(TAG, "SHT20 %s T=%.2fC RH=%.2f%% | RTC %04d-%02d-%02d %02d:%02d:%02d",
                 s_sht20_latest.valid ? "OK":"ERR",
                 s_sht20_latest.temperature_c, s_sht20_latest.humidity_rh,
                 s_rtc_latest.tm_year+1900, s_rtc_latest.tm_mon+1, s_rtc_latest.tm_mday,
                 s_rtc_latest.tm_hour, s_rtc_latest.tm_min, s_rtc_latest.tm_sec);

        vTaskDelay(period);
    }
}

// ==== Thread-safe getters/setters ====
bool sht20_get_latest(sht20_data_t *out)
{
    if (!out) return false;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    *out = s_sht20_latest;
    xSemaphoreGive(s_data_mutex);
    return true;
}

bool rtc_get_time(struct tm *out_tm)
{
    if (!out_tm) return false;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    *out_tm = s_rtc_latest;
    xSemaphoreGive(s_data_mutex);
    return true;
}

bool rtc_set_time(const struct tm *in_tm)
{
    if (!in_tm) return false;
    bool ok = false;

    // I2C 보호 — 태스크의 주기적 읽기와 충돌 방지
    xSemaphoreTake(s_i2c_mutex, portMAX_DELAY);
    ok = pcf_set_time_tm_locked(in_tm);
    xSemaphoreGive(s_i2c_mutex);

    if (!ok) return false;

    // 내부 캐시 갱신
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    s_rtc_latest = *in_tm;
    xSemaphoreGive(s_data_mutex);
    return true;
}

uint32_t i2c_get_bus_reset_count(void)
{
    return s_bus_reset_count;
}
