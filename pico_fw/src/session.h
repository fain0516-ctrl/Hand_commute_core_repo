/*
 * 연결 1개에 대한 프로토콜 처리 + Pico 자체 워치독.
 * 하드웨어 의존성 없음: 네트워크 송신과 액추에이터는 session_io_t 콜백으로 받는다.
 * 같은 코드를 펌웨어(main.c)와 호스트 시뮬레이터(host/sim_main.c)가 쓴다.
 *
 * 동작 규칙 (comm_core/fake_pico.py 와 같음):
 *   HELLO        -> cmd_timeout_ms 저장, HELLO_ACK 응답
 *   HEARTBEAT    -> HEARTBEAT 응답
 *   ACTUATOR_CMD -> 적용 후 STATE 1개 응답
 *   ESTOP        -> 전 축 토크 해제
 *   cmd_timeout_ms 동안 ACTUATOR_CMD 없음, 또는 연결 끊김 -> 전 축 토크 해제
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "fw_types.h"
#include "proto.h"

/* STATE.status 비트 */
#define ST_TORQUE_ENABLED 0x0001  /* 한 축이라도 토크가 걸려 있음 (fake_pico 의 status=1 과 호환) */
#define ST_WATCHDOG 0x0002        /* Pico 워치독으로 토크 해제됨 (다음 위치 지령까지 유지) */
#define ST_ESTOP 0x0004           /* ESTOP 으로 토크 해제됨 (다음 위치 지령까지 유지) */
#define ST_BUS_FAULT 0x0008       /* 응답 없는 서보가 있음 */
#define ST_HELLO_DONE 0x0010      /* 이 연결에서 HELLO 를 받음 */

/* ERROR.code */
enum {
    ERR_NONE = 0,
    ERR_BAD_PAYLOAD = 1,      /* payload 길이가 형식과 맞지 않음 */
    ERR_COUNT_MISMATCH = 2,   /* 지령 축 수가 펌웨어 설정과 다름 (앞쪽 min(count, N) 축만 적용) */
    ERR_UNSUPPORTED_MODE = 3, /* 토크(0 이 아닌 값)/속도 모드: STS3215 는 위치 서보라 지원하지 않음 */
    ERR_VALUE_CLAMPED = 4,    /* 범위를 벗어난 위치 지령을 잘라서 적용함 */
};

/* STATE 축별 flags 비트 */
#define AX_NO_RESPONSE 0x0001
#define AX_BAD_PACKET 0x0002
#define AX_SERVO_ERROR 0x0004 /* 서보 상태 바이트에 오류 비트 (과부하/과열/전압 등) */
#define AX_TORQUE_ON 0x0008
#define AX_CLAMPED 0x0010     /* 마지막 지령이 범위를 벗어나 잘림 */

typedef struct {
    void *ctx;
    /* 네트워크로 바이트를 보낸다 (현재 연결). */
    void (*send)(void *ctx, const uint8_t *data, uint16_t len);
    /* 축 i (mask 비트) 에 targets[i] 위치를 적용하고 토크를 건다. */
    void (*apply)(void *ctx, const int32_t *targets, uint32_t mask);
    /* mask 비트 축의 토크를 해제한다. */
    void (*release)(void *ctx, uint32_t mask);
    /* 현재 상태를 n 축만큼 채우고, 버스 고장 여부를 돌려준다. */
    bool (*read_state)(void *ctx, proto_axis_state_t *out, uint8_t n);
} session_io_t;

typedef struct {
    const fw_actuator_t *acts;
    uint8_t n_act;
    uint8_t n_sts;
    uint8_t n_pwm;
    uint16_t fw_version;
    uint32_t capabilities;
    uint16_t default_timeout_ms;
    uint16_t min_timeout_ms;
    uint16_t max_timeout_ms;
} session_cfg_t;

typedef struct {
    session_cfg_t cfg;
    session_io_t io;
    proto_decoder_t dec;
    bool connected;
    uint16_t tx_seq;
    uint16_t cmd_timeout_ms;
    uint32_t last_cmd_ms;
    uint32_t enabled_mask;
    uint16_t status;
    uint16_t last_error;
    uint32_t error_sent_mask; /* 연결당 같은 ERROR 를 한 번만 보낸다 (스팸 방지) */
    uint32_t clamp_mask;
    uint32_t now_ms;
    uint32_t now_us;
    /* 통계 */
    uint32_t rx_frames, tx_frames, cmds, watchdog_trips, estops;
    uint8_t txbuf[PROTO_MAX_FRAME];
} session_t;

void session_init(session_t *s, const session_cfg_t *cfg, const session_io_t *io);
void session_on_connect(session_t *s, uint32_t now_ms);
void session_on_disconnect(session_t *s);
void session_feed(session_t *s, const uint8_t *data, uint16_t len, uint32_t now_ms, uint32_t now_us);
/* 주기적으로 호출. 워치독 검사. */
void session_tick(session_t *s, uint32_t now_ms);
