#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>                 // strtol
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"              // ★ 광고 지연용 타이머

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// 예제들처럼 헤더 없이 extern 선언으로 사용
void ble_store_config_init(void);

static const char *TAG = "BLE";
#define FIXED_PASSKEY 123456
/* ====== UUIDs (Flutter와 동일) ======
 * Flutter:
 *   Service:  0100bc9a-7856-3412-f0de-bc9a78563412
 *   Char:     00002211-eedd-ccbb-aa99-887766554433
 * NimBLE BLE_UUID128_DECLARE는 128bit 전체를 리틀엔디언 바이트 순서로 넣어야 함(완전 역순).
 */
#define IOO_SERVICE_UUID128  BLE_UUID128_DECLARE( \
    0x12,0x34,0x56,0x78,0x9a,0xbc,0xde,0xf0,0x12,0x34,0x56,0x78,0x9a,0xbc,0x00,0x01)

#define IOO_CHAR_UUID128     BLE_UUID128_DECLARE( \
    0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0x11,0x22,0x00,0x00)

/* ====== NVS 설정 ====== */
#define NVS_NS    "ioo_cfg"   // 네임스페이스
#define KEY_SSID  "ssid"
#define KEY_PWD   "pwd"
#define KEY_CH    "ch"
#define KEY_SP    "sp"

/* ====== 내부 상태 ====== */
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_val_handle  = 0;  // Notify value handle
static bool s_bonding_started = false;  
/* ====== 광고 지연 타이머 ====== */
static esp_timer_handle_t s_adv_timer = NULL;

static void start_advertising(void); // forward
static void adv_timer_cb(void* arg){
    start_advertising();
}
static void restart_adv_later(uint32_t ms){
    if (!s_adv_timer){
        const esp_timer_create_args_t tcfg = {
            .callback = adv_timer_cb,
            .name = "adv_delay",
        };
        if (esp_timer_create(&tcfg, &s_adv_timer) != ESP_OK){
            ESP_LOGE(TAG, "esp_timer_create failed");
            return;
        }
    }
    esp_timer_stop(s_adv_timer);
    esp_timer_start_once(s_adv_timer, (uint64_t)ms * 1000ULL);
}

/* ====== NVS 유틸 ====== */
static esp_err_t cfg_set_str(const char *key, const char *val) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_str(h, key, val ? val : "");
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static esp_err_t cfg_set_i32(const char *key, int32_t v) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_i32(h, key, v);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static bool parse_i32(const char *s, int32_t *out) {
    if (!s || !*s) return false;
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s) return false;        // 숫자 아님
    *out = (int32_t)v;
    return true;
}

/* 간단 헬퍼(입력 필터에 사용) */
static inline void rtrim(char *s) {
    size_t n = strlen(s);
    while (n && (s[n-1]==' ' || s[n-1]=='\t')) s[--n] = '\0';
}
static inline void lskip(const char **p) { while (**p==' ' || **p=='\t') ++(*p); }
static bool printable_ascii_str(const char *s) {
    for (; *s; ++s) if (*s!= '\t' && (*s < 0x20 || *s > 0x7E)) return false;
    return true;
}

/* (선택) 부팅 시 저장된 값 로깅 */
static void cfg_log_current(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "NVS NS '%s' not found yet.", NVS_NS);
        return;
    }
    char ssid[64] = {0}, pwd[64] = {0};
    size_t len;

    len = sizeof(ssid);
    if (nvs_get_str(h, KEY_SSID, ssid, &len) == ESP_OK)
        ESP_LOGI(TAG, "NVS SSID: %s", ssid);

    len = sizeof(pwd);
    if (nvs_get_str(h, KEY_PWD, pwd, &len) == ESP_OK)
        ESP_LOGI(TAG, "NVS PWD: %s", pwd);

    int32_t ch, sp;
    if (nvs_get_i32(h, KEY_CH, &ch) == ESP_OK)
        ESP_LOGI(TAG, "NVS CH: %ld", (long)ch);

    if (nvs_get_i32(h, KEY_SP, &sp) == ESP_OK)
        ESP_LOGI(TAG, "NVS SP: %ld", (long)sp);

    nvs_close(h);
}

/* ====== 선언부 ====== */
static int  on_gap_event(struct ble_gap_event *ev, void *arg);
static int  on_rx_write(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg);

/* ====== 명령 파서 + NVS 저장 + 입력 필터 ======
 * req_disconnect: true면 응답을 notify로 보낸 뒤 연결 종료(예: BYE)
 */
static void build_response(const uint8_t *in, uint16_t inlen,
                           uint8_t *out, uint16_t *outlen,
                           bool *req_disconnect)
{
    if (req_disconnect) *req_disconnect = false;

    char line[240] = {0};    // 입력 1줄
    char key[32]   = {0};    // 명령 키 (대문자)
    const char *val = NULL;  // 값 시작 포인터(원본 대소문자 유지)

    // 1) 개행 제거하고 한 줄 만들기
    uint16_t n = (inlen < sizeof(line)-1) ? inlen : (sizeof(line)-1);
    for (uint16_t i=0, j=0; i<n; i++) {
        char c = (char)in[i];
        if (c=='\r' || c=='\n') break;
        line[j++] = c;
    }
    if (line[0] == '\0') {
        const char *resp = "UNKNOWN (try HELP)";
        *outlen = (uint16_t)strlen(resp);
        memcpy(out, resp, *outlen);
        return;
    }

    // 2) key, value 분리 (구분자: 첫 공백 또는 콜론)
    const char *sep = NULL;
    for (const char *p = line; *p; ++p) {
        if (*p == ' ' || *p == ':') { sep = p; break; }
    }

    size_t keylen = sep ? (size_t)(sep - line) : strlen(line);
    if (keylen >= sizeof(key)) keylen = sizeof(key) - 1;

    // key를 대문자로 복사
    for (size_t i=0; i<keylen; ++i) {
        char c = line[i];
        if (c >= 'a' && c <= 'z') c -= ('a' - 'A');
        key[i] = c;
    }
    key[keylen] = '\0';

    // 값 포인터 설정 (있다면 구분자 다음부터, 공백 건너뜀)
    if (sep) {
        val = sep + 1;
        lskip(&val);
    } else {
        val = "";
    }
    rtrim(line); // (선택) 끝 공백 제거

    // 3) 명령 처리
    char resp[240] = {0};
    esp_err_t err;
    int32_t iv;

    if (strcmp(key, "PING") == 0) {
        snprintf(resp, sizeof(resp), "PONG");

    } else if (strcmp(key, "HELLO") == 0) {
        snprintf(resp, sizeof(resp), "OK HELLO");

    } else if (strcmp(key, "HELP") == 0) {
        snprintf(resp, sizeof(resp), "CMDS: PING, HELLO, HELP, ECHO <txt>, SSID:<s>, PWD:<s>, CH:<n>, SP:<n>, BYE");

    } else if (strcmp(key, "ECHO") == 0) {
        if (!printable_ascii_str(val)) snprintf(resp,sizeof(resp),"ERR ECHO CHAR");
        else snprintf(resp,sizeof(resp),"%s", val);

    } else if (strcmp(key, "SSID") == 0) {
        size_t L = strlen(val);
        if (L==0 || L>63) { snprintf(resp,sizeof(resp),"ERR SSID LEN"); }
        else if (!printable_ascii_str(val)) { snprintf(resp,sizeof(resp),"ERR SSID CHAR"); }
        else { err = cfg_set_str(KEY_SSID,val); snprintf(resp,sizeof(resp),(err==ESP_OK)?"OK SSID":"ERR SSID %d",err); }

    } else if (strcmp(key, "PWD") == 0) {
        size_t L = strlen(val);
        if (L==0 || L>63) { snprintf(resp,sizeof(resp),"ERR PWD LEN"); }
        else if (!printable_ascii_str(val)) { snprintf(resp,sizeof(resp),"ERR PWD CHAR"); }
        else { err = cfg_set_str(KEY_PWD,val); snprintf(resp,sizeof(resp),(err==ESP_OK)?"OK PWD":"ERR PWD %d",err); }

    } else if (strcmp(key, "CH") == 0) {
        if (!parse_i32(val,&iv)) snprintf(resp,sizeof(resp),"ERR CH (not number)");
        else if (iv < 1 || iv > 10) snprintf(resp,sizeof(resp),"ERR CH RANGE");
        else { err = cfg_set_i32(KEY_CH,iv); snprintf(resp,sizeof(resp),(err==ESP_OK)?"OK CH":"ERR CH %d",err); }

    } else if (strcmp(key, "SP") == 0) {
        if (!parse_i32(val,&iv)) snprintf(resp,sizeof(resp),"ERR SP (not number)");
        else if (iv < 1 || iv > 12) snprintf(resp,sizeof(resp),"ERR SP RANGE");
        else { err = cfg_set_i32(KEY_SP,iv); snprintf(resp,sizeof(resp),(err==ESP_OK)?"OK SP":"ERR SP %d",err); }

    } else if (strcmp(key, "BYE") == 0) {
        // 앱이 BYE를 보내면 응답 후 연결 종료하도록 지시
        snprintf(resp, sizeof(resp), "BYE");
        if (req_disconnect) *req_disconnect = true;

    }else {
        snprintf(resp, sizeof(resp), "UNKNOWN (try HELP)");
    }
    *outlen = (uint16_t)strnlen(resp, sizeof(resp));
    memcpy(out, resp, *outlen);
}

static esp_timer_handle_t s_authok_timer = NULL;

static bool bt_notify(const uint8_t *data, uint16_t len)
{
    if (!data || !len) return false;
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_val_handle == 0) return false;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) return false;

    int rc = ble_gatts_notify_custom(s_conn_handle, s_val_handle, om);
    if (rc != 0) ESP_LOGW(TAG, "notify fail rc=%d (conn=%u val=%u)", rc, s_conn_handle, s_val_handle);
    return (rc == 0);
}

static void authok_timer_cb(void *arg) {
    const char ok[] = "AUTHOK";
    if (bt_notify((const uint8_t*)ok, sizeof(ok)-1))
        ESP_LOGI(TAG, "AUTHOK sent");
    else
        ESP_LOGW(TAG, "AUTHOK notify failed");
}

static void send_authok_delayed(uint32_t delay_ms) {
    if (!s_authok_timer) {
        const esp_timer_create_args_t tcfg = {
            .callback = authok_timer_cb,
            .name = "authok",
        };
        if (esp_timer_create(&tcfg, &s_authok_timer) != ESP_OK) {
            ESP_LOGE(TAG, "esp_timer_create(authok) failed");
            return;
        }
    }
    esp_timer_stop(s_authok_timer);
    esp_timer_start_once(s_authok_timer, (uint64_t)delay_ms * 1000ULL);
}

static inline void send_authno_now(void) {
    const char no[] = "AUTHNO";
    if (!bt_notify((const uint8_t*)no, sizeof(no)-1)) {
        ESP_LOGW(TAG, "AUTHNO notify failed");
    } else {
        ESP_LOGI(TAG, "AUTHNO sent");
    }
}



/* ====== GATT 정의: 단일 캐릭터리스틱(Write + Notify) ====== */
static struct ble_gatt_chr_def s_ioo_chrs[] = {
    {
        .uuid       = IOO_CHAR_UUID128,
        .access_cb  = on_rx_write,   // 앱에서 Write 시 호출
        .val_handle = &s_val_handle, // Notify 송신에 사용
        .flags      = BLE_GATT_CHR_F_WRITE |
                      BLE_GATT_CHR_F_WRITE_NO_RSP |
                      BLE_GATT_CHR_F_NOTIFY,
    },
    { 0 } // 종결자
};

static const struct ble_gatt_svc_def s_ioo_svcs[] = {
    {
        .type            = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid            = IOO_SERVICE_UUID128,
        .characteristics = s_ioo_chrs,
    },
    { 0 } // 종결자
};

/* ====== 유틸 ====== */
// static bool is_enc(uint16_t conn_handle)
// {
//     struct ble_gap_conn_desc d;
//     if (ble_gap_conn_find(conn_handle, &d) != 0) return false;
//     return d.sec_state.encrypted;
// }

/* ====== GAP 이벤트 ====== */
static int on_gap_event(struct ble_gap_event *ev, void *arg)
{
    (void)arg;
    switch (ev->type) {

    case BLE_GAP_EVENT_CONNECT:
        s_bonding_started = false;
        ESP_LOGI(TAG, "connect %s; status=%d",
                 ev->connect.status == 0 ? "ok" : "fail",
                 ev->connect.status);

        if (ev->connect.status == 0) {
            s_conn_handle = ev->connect.conn_handle;
        } else {
            // 연결 실패 → 광고 재시작
            restart_adv_later(3000);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnect; reason=%d", ev->disconnect.reason);
        send_authno_now();
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        // 광고 재시작 지연 (삼성 자동 재연결 완화)
        restart_adv_later(3000);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "adv complete; reason=%d", ev->adv_complete.reason);
        restart_adv_later(3000);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE: {
        ESP_LOGI(TAG, "subscribe: handle=%d cur_notif=%d cur_ind=%d",
                ev->subscribe.attr_handle,
                ev->subscribe.cur_notify,
                ev->subscribe.cur_indicate);

        if (ev->subscribe.attr_handle != s_val_handle) return 0; // 내 캐릭만 처리

        if (ev->subscribe.cur_notify) {
            struct ble_gap_conn_desc d;
            if (ble_gap_conn_find(s_conn_handle, &d) == 0) {
                if (!d.sec_state.encrypted) {
                    ESP_LOGI(TAG, "start security on subscribe");
                    ble_gap_security_initiate(s_conn_handle);
                } else {
                    // CCCD on + 이미 암호화 → 살짝 지연 후 AUTHOK
                    send_authok_delayed(100);
                }
            }
        }
        return 0;
    }

    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "enc change: status=%d", ev->enc_change.status);
        if (ev->enc_change.status == 0) {
            // 암호화 완료 직후에도 한 번 더 시도 (중복은 앱에서 무시해도 OK)
            send_authok_delayed(100);
        }
        if (ev->enc_change.status != 0) {
        ESP_LOGW(TAG, "enc failed=%d (pairing canceled/failed)", ev->enc_change.status);
        send_authno_now();
        // 보통 곧 disconnect가 옴. 직접 끊고 재광고해도 됨.
        ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
        return 0;

    case BLE_GAP_EVENT_PASSKEY_ACTION: {
        struct ble_sm_io io = {0};
        io.action = ev->passkey.params.action;

        switch (io.action) {
        case BLE_SM_IOACT_DISP:
            // ESP32가 “표시”할 PIN을 고정값으로 제공 → 폰이 123456 입력
            io.passkey = FIXED_PASSKEY;
            ESP_LOGI(TAG, "PASSKEY (display to user): %06u", (unsigned)io.passkey);
            break;

        case BLE_SM_IOACT_INPUT:
            // DISPLAY_ONLY면 보통 오지 않지만, 혹시 호스트 조합상 INPUT이 오면 동일 값 입력
            io.passkey = FIXED_PASSKEY;
            ESP_LOGI(TAG, "PASSKEY (input by device): %06u", (unsigned)io.passkey);
            break;

        case BLE_SM_IOACT_NUMCMP:
            // 숫자비교 방식이 오면 자동 승인(원치 않으면 0으로 거절)
            io.numcmp_accept = 1;
            ESP_LOGI(TAG, "NUMCMP: auto-accept");
            break;

        default:
            ESP_LOGW(TAG, "Unknown passkey action=%d", io.action);
            break;
        }

        ble_sm_inject_io(ev->passkey.conn_handle, &io);
        return 0;
    }

    default:
        return 0;
    }
}


/* ====== RX Write 콜백 ====== */
static int on_rx_write(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)attr_handle; (void)arg;
    s_conn_handle = conn_handle;

    const uint16_t rxlen = OS_MBUF_PKTLEN(ctxt->om);
    if (rxlen == 0 || rxlen > 240) {
        const char *err = (rxlen==0) ? "ERR EMPTY" : "ERR TOO LONG";
        (void)bt_notify((const uint8_t*)err, strlen(err));
        return 0;
    }

    uint8_t buf_in[244] = {0};
    ble_hs_mbuf_to_flat(ctxt->om, buf_in, sizeof(buf_in), NULL);

    // 비인쇄 문자 필터
    for (uint16_t i=0; i<rxlen; ++i) {
        uint8_t c = buf_in[i];
        if (c=='\r' || c=='\n' || c=='\t') continue;
        if (c < 0x20 || c > 0x7E) {
            const char *err = "ERR CHAR";
            (void)bt_notify((const uint8_t*)err, strlen(err));
            return 0;
        }
    }

    ESP_LOGI(TAG, "RX(%u): %.*s", rxlen, rxlen, (const char*)buf_in);

    uint8_t out[240]; uint16_t outlen=0;
    bool req_disc = false;
    build_response(buf_in, rxlen, out, &outlen, &req_disc);
    (void)bt_notify(out, outlen);

    if (req_disc && s_conn_handle != BLE_HS_CONN_HANDLE_NONE){
        vTaskDelay(pdMS_TO_TICKS(500)); 
        ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    return 0;
}


/* ====== 광고 시작 ====== */
static void start_advertising(void)
{
    char name[32] = {0};
    uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_BT);
    snprintf(name, sizeof(name), "FAL_%02X%02X%02X%02X", mac[2],mac[3],mac[4],mac[5]);

    ble_svc_gap_device_name_set(name);

    struct ble_hs_adv_fields adv = {0};
    adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv.name = (uint8_t*)name;
    adv.name_len = strlen(name);
    adv.name_is_complete = 1;
    adv.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    adv.tx_pwr_lvl_is_present = 1;

    int rc = ble_gap_adv_set_fields(&adv);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv set fields err=%d", rc);
        return;
    }

    struct ble_gap_adv_params ap = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min  = BLE_GAP_ADV_ITVL_MS(200),
        .itvl_max  = BLE_GAP_ADV_ITVL_MS(220),
    };
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &ap, on_gap_event, NULL);
    if (rc != 0) ESP_LOGE(TAG, "adv start err=%d", rc);
    else ESP_LOGI(TAG, "Advertising as \"%s\"", name);
}

/* ====== 등록 로그 ====== */
static void gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    (void)arg;
    char u[BLE_UUID_STR_LEN];
    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGI(TAG, "reg svc %s handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid,u), ctxt->svc.handle);
        break;
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGI(TAG, "reg chr %s def=%d val=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid,u),
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;
    default: break;
    }
}

/* ====== 호스트 콜백 구성 ====== */
static void on_reset(int reason){ ESP_LOGI(TAG,"nimble reset: %d", reason); }
static void on_sync(void){ start_advertising(); }

static void host_cfg_init(void)
{
    ble_hs_cfg.reset_cb           = on_reset;
    ble_hs_cfg.sync_cb            = on_sync;
    ble_hs_cfg.gatts_register_cb  = gatt_register_cb;
    // 보안(Just Works + Bonding + LE SC)
    ble_hs_cfg.sm_io_cap          = BLE_HS_IO_DISPLAY_ONLY;
    ble_hs_cfg.sm_bonding         = 1; // 본딩이 필요없다면 0으로 꺼도 됨
    ble_hs_cfg.sm_mitm            = 1;
    ble_hs_cfg.sm_sc              = 1;
    ble_hs_cfg.sm_our_key_dist   |= BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_store_config_init(); // 본딩/키 저장
}

/* ====== NimBLE 호스트 태스크 ====== */
static void nimble_host_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "nimble host task start");
    nimble_port_run();  // nimble_port_stop() 호출 시까지 블록
    nimble_port_freertos_deinit();
    vTaskDelete(NULL);
}

/* ====== 공개 API ====== */
void bt_init(void)
{
    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // (선택) 저장값 로깅
    cfg_log_current();

    // NimBLE 호스트 초기화
    ESP_ERROR_CHECK(nimble_port_init());

    // 호스트 설정/보안 구성
    host_cfg_init();

    // GAP/GATT 기본 서비스
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // 커스텀 서비스 등록
    int rc = ble_gatts_count_cfg(s_ioo_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed, rc=%d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(s_ioo_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed, rc=%d", rc);
        return;
    }
}

void bt_task(void *arg)
{
    (void)arg;
    // 호스트 태스크 실행
    xTaskCreate(nimble_host_task, "NimBLE Host", 4096, NULL, 5, NULL);
    for(;;) vTaskDelay(pdMS_TO_TICKS(1000));
}
