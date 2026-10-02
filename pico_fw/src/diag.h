/* 진단 카운터 (DIAG 메시지로 Pi 에 주기 보고). 필드 목록은 proto.h 의 DIAG_FIELDS.
 * 필드마다 쓰는 코어가 하나뿐이라 (버스 = core1, 네트워크/세션 = core0) 잠금 없이 증가시킨다. */
#pragma once
#include <stdint.h>

#include "proto.h"

extern volatile uint32_t g_diag[DIAG_COUNT];

static inline void diag_inc(int field) { g_diag[field]++; }
static inline void diag_max(int field, uint32_t v) {
    if (v > g_diag[field])
        g_diag[field] = v;
}
