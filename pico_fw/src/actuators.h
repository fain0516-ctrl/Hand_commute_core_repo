/*
 * 액추에이터 루프 (core1): STS3215 버스 + PWM 출력.
 * core0 (네트워크/세션) 은 목표값과 토크 마스크를 우편함에 넣고, 상태 스냅샷을 가져가기만 한다.
 * 버스 통신(수 ms)이 네트워크 응답을 막지 않도록 코어를 나눈다.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "proto.h"

void actuators_init(void);       /* core0 에서 호출: 핀/우편함 초기화 후 core1 시작 */
void actuators_apply(const int32_t *targets, uint32_t mask);
void actuators_release(uint32_t mask);
bool actuators_read_state(proto_axis_state_t *out, uint8_t n); /* 버스 고장이면 true */
uint32_t actuators_loop_count(void);

/* core1 루프를 나눈 단계 (호스트 시뮬레이터가 core1 없이 직접 돌릴 때 사용) */
void actuators_setup(void);     /* 우편함/PWM 초기화 (core1 시작 안 함) */
void actuators_bus_start(void); /* 서보 버스 초기화 + 전 축 토크 해제 */
void actuators_step(void);      /* 루프 1회 */
