/* 생성된 fw_config.h 와 코드가 함께 쓰는 타입. 하드웨어 의존성 없음 (호스트 테스트에서도 사용). */
#pragma once
#include <stdint.h>

typedef enum {
    FW_ACT_STS = 0, /* FeeTech STS3215: id = 서보 버스 ID, 값 = tick (0~4095) */
    FW_ACT_PWM = 1, /* PWM 서보: id = 채널 번호, 값 = 펄스 폭 us */
} fw_act_kind_t;

typedef struct {
    uint8_t kind;    /* fw_act_kind_t */
    uint8_t id;
    int32_t raw_min; /* 위치 지령 클램핑 범위 */
    int32_t raw_max;
    int8_t pin;      /* PWM 출력 GPIO (STS 는 -1) */
} fw_actuator_t;
