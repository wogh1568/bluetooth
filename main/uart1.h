#pragma once
#include <stdint.h>
#include <stdbool.h>


#define WAVE_CNT			16
#define CYCLE_SAMPLE_CNT 	128			// 60Hz * 128 = 7680Hz,      16.6ms / 128 = 130us
#define	MAX_PERIOD			(CYCLE_SAMPLE_CNT * WAVE_CNT)

// ---------- 주의 ----------
// STM32에서 구조체를 "통째로" 보낸다고 했기 때문에
// 패딩 차이를 줄이려면 가능하면 양쪽 모두에 pack 처리 권장.
// (둘 다 GCC라 보통 맞지만, 안전하게 하려면 #pragma pack(1) 사용 고려)
#pragma pack(push,1)
typedef struct {                // flash 저장내용
	char sn[8];
	int	 hw_ver;
	int  sw_ver;

	int ct_ratio;		// 1000  -  1000/5A    전류 ct 비
} _TPM_Info;

typedef struct {
	int  state;			// 상태비트
	int  run_time;		// 부팅후 운영시간 분단위 오버틀로우는 1분 부터
	int  bat_volt;		// 인산철 2500~3600 * 0.001 V 단위

	float L1V_rms, L2V_rms, L3V_rms;					// rms value 200.0 ~ 250.0 	V 
	float L1A_rms, L2A_rms, L3A_rms;					// rms value  10 ~ 30000 A 
	float L1_phase_deg, L2_phase_deg, L3_phase_deg;		// 전압 전류 위상각  도 단위
	float L1V_L2V_phase_deg,L2V_L3V_phase_deg;			// 전입 L1 L2 위상각
	float L1A_L2A_phase_deg,L2A_L3A_phase_deg;

	float L1_PowerFactor, L2_PowerFactor, L3_PowerFactor;				// power pactor 1000 * 0.1% 단위
	float L1_ActivePower, L2_ActivePower, L3_ActivePower;				// 유효전력   * 1W 단위
	float L1_ReactivePower, L2_ReactivePower, L3_ReactivePower;		// 무효전력   * 1W 단위
	float L1_ApparentPower, L2_ApparentPower, L3_ApparentPower;		// 피상전력   * 1W 단위 [ 유혀전력 + 무효전력] = 전압 * 전류
	float L1_Pwr, L2_Pwr, L3_Pwr;										// 통신 누적 전력 1w 단위

	float L1V_Thd, L2V_Thd, L3V_Thd;									// 종합 왜곡률
	float L1A_Thd, L2A_Thd, L3A_Thd;

	float L1V_harm_freq[10], L1V_harm_ratio[10], L1V_harm_volt[10];
	float L2V_harm_freq[10], L2V_harm_ratio[10], L2V_harm_volt[10];
	float L3V_harm_freq[10], L3V_harm_ratio[10], L3V_harm_volt[10];

	float L1A_harm_freq[10], L1A_harm_ratio[10], L1A_harm_curr[10];
	float L2A_harm_freq[10], L2A_harm_ratio[10], L2A_harm_curr[10];
	float L3A_harm_freq[10], L3A_harm_ratio[10], L3A_harm_curr[10];
} _TPM_Power_Info;

typedef struct
{
	signed short L1V[MAX_PERIOD];
	signed short L2V[MAX_PERIOD];
	signed short L3V[MAX_PERIOD];

	signed short L1A[MAX_PERIOD];
	signed short L2A[MAX_PERIOD];
	signed short L3A[MAX_PERIOD];
}_TPM_RAW;

typedef struct	// flash 저장내용
{
	char sn[8];
	int	 hw_ver;
	int  sw_ver;

	int zct_ratio;		// 100   -   100/5A    누설전류 ct 비
}_IOR_Info;

typedef struct
{
	int  state;			// 상태비트
	int  run_time;		// 부팅후 운영시간 분단위 오버틀로우는 1분 부터
	int  bat_volt;		// 인산철 2500~3600 * 0.001 V 단위

	float Leak_mA;		// io 일반 누설전류 * 0.001 mA
	float Ior_mA;			// ior 저항성 누설전류 * 0.001 mA
	float Ohm;			// 환산 저항 * 1 ohm

	float L1_phase_deg, L2_phase_deg, L3_phase_deg;	// 전압전류 위상각 * 0.1 도 단위
	float Io_curr_harm_freq[10];		//	전류 왜곡률
}_IOR_State;


typedef struct	// flash 저장내용
{
	char sn[8];
	int	 hw_ver;
	int  sw_ver;

	int zct_ratio;		// 100   -   100/5A    누설전류 ct 비
}_Vo_Info;

typedef struct
{
	int  state;			// 상태비트
	int  run_time;		// 부팅후 운영시간 분단위 오버틀로우는 1분 부터
	int  bat_volt;		// 인산철 2500~3600 * 0.001 V 단위

	float  volt;	// rms value 2000 ~ 2500  * 0.1V 단뒤
	float  volt_harm_freq[10];
	float L1_phase_deg, L2_phase_deg, L3_phase_deg;	// 전압전류 위상각 * 0.1 도 단위
}_Vo_State;


#pragma pack(pop)

// 수신 결과 저장용 전역 (사용자가 읽기만)
extern volatile _TPM_Info       g_tpm_info;
extern volatile _TPM_Power_Info g_tpm_power_info;
extern volatile _TPM_RAW        g_volt_curr_adc;
extern volatile _IOR_Info       g_ior_info;
extern volatile _IOR_State      g_ior_state;
extern volatile _Vo_Info        g_vo_info;
extern volatile _Vo_State       g_vo_state;

#ifdef __cplusplus
extern "C" {
#endif

// 초기화 + 드라이버 설치 (UART1, TXD=GPIO17, RXD=GPIO18, 460800bps)
void uart1_init(void);

// UART 통신 전용 Task (app_main에서 생성)
void uart1_task(void *arg);

// 동기 요청/응답 헬퍼 (요청 보내고, 해당 응답 올 때까지 대기)
// cmd2=0x00 -> TPM_Info, cmd2=0x01 -> TPM_Power_Info
bool uart1_request_info(uint32_t target_id, uint8_t cmd2, uint32_t timeout_ms);

// 로우레벨 프레임 송신 함수 (프로토콜 빌더)
// target_id는 0x11223344 이면 바이트 순서 11,22,33,44
bool uart1_send_frame(uint32_t target_id, uint8_t cmd1, uint8_t cmd2,
                      const void *payload, uint16_t payload_len);

#ifdef __cplusplus
}
#endif
