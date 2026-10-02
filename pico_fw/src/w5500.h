/*
 * W5500 최소 드라이버: TCP 서버 1 포트, 여러 소켓이 같은 포트에서 대기.
 * 새 연결이 ESTABLISHED 되면 이전 활성 연결은 끊는다 (Pi 재연결 우선).
 * 핀/주소/버퍼 크기는 모두 fw_config.h (YAML) 에서 온다.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    NET_EV_NONE = 0,
    NET_EV_CONNECTED,
    NET_EV_DISCONNECTED,
} net_event_t;

/* 칩 리셋, 버전 확인, 주소 설정, 소켓 대기. 실패하면 false (SPI 배선/전원 확인). */
bool w5500_init(void);
/* PHY 링크 (랜선) 상태 */
bool w5500_link_up(void);
/* 상태를 확인하고 연결/끊김 이벤트를 돌려준다. 받은 데이터는 buf 에 넣고 *len 에 길이. */
net_event_t w5500_poll(uint8_t *buf, uint16_t cap, uint16_t *len);
/* 활성 연결로 송신. 송신 버퍼가 모자라면 false (호출 측은 프레임을 버린다: 오래된 상태는 의미 없음). */
bool w5500_send(const uint8_t *data, uint16_t len);
/* 활성 연결을 끊는다 (링크 유휴 타임아웃 등). */
void w5500_drop_active(void);
bool w5500_has_client(void);
/* 주기 건강 검사 (VERSIONR, SIPR/SHAR 재확인, 소켓 상태). 연속 실패가 한계에 닿으면 false -> 재초기화 */
bool w5500_health_check(void);
/* 재초기화 직전에 호출: 횟수를 세고 필요하면 SPI 클럭을 낮춘다 */
void w5500_note_reinit(void);
uint32_t w5500_spi_baud(void);
