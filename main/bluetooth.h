#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 초기화 + NimBLE 호스트 태스크 시작
void bt_init(void);
void bt_task(void *arg);

// 외부에서 임의 데이터 Notify
bool bt_notify(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif
