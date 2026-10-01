/*
 * Pi 5 <-> Pico 2 TCP 프레임 프로토콜 (comm_core/protocol.py 와 1:1 대응).
 * 하드웨어 의존성 없음. 호스트 테스트(pico_fw/host)에서도 같은 파일을 컴파일한다.
 *
 *   off size 내용
 *   0   2    magic 0xAA 0x55
 *   2   1    version
 *   3   1    msg_type
 *   4   1    flags
 *   5   1    reserved
 *   6   2    seq
 *   8   4    timestamp (us)
 *   12  2    payload_len (<= PROTO_MAX_PAYLOAD)
 *   14  N    payload
 *   14+N 2   CRC-16/CCITT-FALSE (header + payload)
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PROTO_VERSION 1
#define PROTO_HEADER_SIZE 14
#define PROTO_CRC_SIZE 2
#define PROTO_MAX_PAYLOAD 1024
#define PROTO_MAX_FRAME (PROTO_HEADER_SIZE + PROTO_MAX_PAYLOAD + PROTO_CRC_SIZE)
#define PROTO_MAX_ACTUATORS 32
#define PROTO_AXIS_STATE_SIZE 16

enum {
    MSG_HELLO = 0x01,
    MSG_HELLO_ACK = 0x02,
    MSG_HEARTBEAT = 0x03,
    MSG_ACTUATOR_CMD = 0x10,
    MSG_ESTOP = 0x11,
    MSG_STATE = 0x20,
    MSG_ERROR = 0x7F,
};

enum {
    CMD_MODE_POSITION = 0, /* STS tick / PWM us */
    CMD_MODE_TORQUE = 1,   /* mNm. 0 = 해당 축 토크 해제 */
    CMD_MODE_VELOCITY = 2, /* 예약 */
};

#define CMD_FLAG_WATCHDOG_TRIPPED 0x01

typedef struct {
    uint8_t version;
    uint8_t msg_type;
    uint8_t flags;
    uint16_t seq;
    uint32_t timestamp_us;
    uint16_t payload_len;
    const uint8_t *payload; /* 디코더 내부 버퍼를 가리킨다. 콜백 안에서만 유효 */
} proto_frame_t;

typedef struct {
    int32_t position;
    int32_t velocity;
    int32_t effort;
    int16_t temperature_c10;
    uint16_t flags;
} proto_axis_state_t;

uint16_t proto_crc16(const uint8_t *data, size_t len, uint16_t crc);

/* out 에 프레임을 쓰고 길이를 돌려준다. out 은 PROTO_HEADER_SIZE + payload_len + 2 바이트 이상. */
size_t proto_encode(uint8_t *out, uint8_t msg_type, uint8_t flags, uint16_t seq, uint32_t timestamp_us,
                    const uint8_t *payload, uint16_t payload_len);

/* TCP 스트림 디코더. 깨진 데이터는 1 바이트씩 버리고 다음 magic 에서 재동기화한다 (Python 과 같은 규칙). */
typedef void (*proto_frame_cb)(void *ctx, const proto_frame_t *frame);

typedef struct {
    uint8_t buf[PROTO_MAX_FRAME * 2];
    size_t len;
    uint32_t crc_errors;
    uint32_t dropped_bytes;
} proto_decoder_t;

void proto_decoder_reset(proto_decoder_t *d);
void proto_decoder_feed(proto_decoder_t *d, const uint8_t *data, size_t len, proto_frame_cb cb, void *ctx);

/* Little-endian 도우미 */
static inline void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void put_u32(uint8_t *p, uint32_t v) { put_u16(p, (uint16_t)v); put_u16(p + 2, (uint16_t)(v >> 16)); }
static inline uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t get_u32(const uint8_t *p) { return (uint32_t)get_u16(p) | ((uint32_t)get_u16(p + 2) << 16); }
