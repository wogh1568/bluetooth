#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "uart1.h"   // _TPM_Info, _TPM_Power_Info, _ADC_Sensing, MAX_PERIOD 등 포함

#ifdef __cplusplus
extern "C" {
#endif

// ===== 설정 (필요시 변경) =====
#define WIFI_SSID      "MIRAE24G"
#define WIFI_PASSWORD  "0280185004"
#define WIFI_SERVER_IP "192.168.0.66"
#define WIFI_SERVER_PORT 12345

// 프로토콜 상수
#define STX                0x02
#define ETX                0x03
#define DEV_TYPE_CM4       0xA4   // CM4->ESP32 (참고)
#define DEV_TYPE_TX        0xA5   // ESP32->STM32, ESP32->CM4
#define DEV_TYPE_RX        0xA6   // STM32->ESP32
#define CMD1_REQ           'S'    // 요청은 'S'
#define CMD1_RSP           's'    // 응답은 's'

// cmd2 매핑 (Wi-Fi로 보낼 때)
#define CMD2_TPM_INFO      0x00
#define CMD2_TPM_POWER     0x01
#define CMD2_ADC_SENSING   0x02

// 타겟 ID (필요하면 외부에서 바꿀 수 있게 setter 만들어도 됨)
#define WIFI_TARGET_ID     0x11223344

// 초기화 & 태스크
void wifi_init_start(void);
void wifi_task(void *arg);

// UART task 쪽에서 최신 데이터 전달할 때 호출 (mutex로 보호)
void wifi_post_tpm_info(const _TPM_Info *p);
void wifi_post_power_info(const _TPM_Power_Info *p);
void wifi_post_adc_sensing(const _TPM_RAW *p);

#ifdef __cplusplus
}
#endif
