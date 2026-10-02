/*
 * FeeTech STS3215 (SCS 프로토콜) 반이중 UART 버스.
 * 패킷: FF FF ID LEN INSTR PARAM... CHK,  CHK = ~(ID + LEN + INSTR + PARAM...)
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* 메모리 맵 (STS3215) */
#define STS_TORQUE_ENABLE 40
#define STS_ACC 41
#define STS_GOAL_POSITION 42 /* 2B, 이어서 GOAL_TIME 2B, GOAL_SPEED 2B */
#define STS_PRESENT_POSITION 56 /* 2B pos, 2B speed, 2B load, 1B voltage, 1B temperature */
#define STS_PRESENT_BLOCK_LEN 8
#define STS_VERIFY_BLOCK_LEN 4 /* 40 TORQUE_ENABLE, 41 ACC, 42-43 GOAL_POSITION: 지령 재확인용 */

#define STS_BROADCAST 0xFE

typedef enum {
    STS_OK = 0,
    STS_TIMEOUT,
    STS_BAD_PACKET,
} sts_result_t;

void sts_bus_init(void);
/* 응답 없는 쓰기 (ID 하나). 에코가 틀리면 robustness.servo_bus.write_retries 번 재전송 */
void sts_write(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t len);
/* SYNC WRITE: ids[n] 각각에 data[i*len .. ] 를 같은 주소에 쓴다. 응답 없음. */
void sts_sync_write(uint8_t addr, uint8_t len, const uint8_t *ids, const uint8_t *data, uint8_t n);
/* READ: 응답 대기 (헤더 재동기화, ID/길이/체크섬 검사, read_retries 번 재시도). *servo_error 에 서보 상태 바이트 */
sts_result_t sts_read(uint8_t id, uint8_t addr, uint8_t len, uint8_t *out, uint8_t *servo_error);

/* 부호 비트 형식 변환 (STS: 위치/속도는 bit15, 부하는 bit10 이 부호) */
static inline int32_t sts_decode_sign(uint16_t raw, uint8_t sign_bit) {
    uint16_t mag = raw & (uint16_t)((1u << sign_bit) - 1u);
    return (raw & (1u << sign_bit)) ? -(int32_t)mag : (int32_t)mag;
}
static inline uint16_t sts_encode_pos(int32_t v) {
    return v < 0 ? (uint16_t)((-v) | 0x8000) : (uint16_t)v;
}
