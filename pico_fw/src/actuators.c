#include "actuators.h"

#include <string.h>

#include "fw_config.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "session.h"
#include "sts_bus.h"

static const fw_actuator_t ACTS[FW_N_ACT] = FW_ACTUATORS_INIT;

/* ---------------------------------------------------------------- core0 <-> core1 우편함 */

typedef struct {
    int32_t targets[FW_N_ACT];
    uint32_t enable_mask;   /* 토크를 걸 축 */
    uint32_t new_target;    /* core1 이 아직 반영하지 않은 목표값이 있는 축 */
} mailbox_t;

static spin_lock_t *lock;
static mailbox_t mbox;
static proto_axis_state_t shared_state[FW_N_ACT];
static bool shared_fault;
static volatile uint32_t loops;

void actuators_apply(const int32_t *targets, uint32_t mask) {
    uint32_t irq = spin_lock_blocking(lock);
    for (int i = 0; i < FW_N_ACT; i++) {
        if (mask & (1u << i))
            mbox.targets[i] = targets[i];
    }
    mbox.enable_mask |= mask;
    mbox.new_target |= mask;
    spin_unlock(lock, irq);
}

void actuators_release(uint32_t mask) {
    uint32_t irq = spin_lock_blocking(lock);
    mbox.enable_mask &= ~mask;
    spin_unlock(lock, irq);
}

bool actuators_read_state(proto_axis_state_t *out, uint8_t n) {
    uint32_t irq = spin_lock_blocking(lock);
    memcpy(out, shared_state, sizeof(proto_axis_state_t) * (n < FW_N_ACT ? n : FW_N_ACT));
    bool fault = shared_fault;
    spin_unlock(lock, irq);
    for (uint8_t i = FW_N_ACT; i < n; i++)
        memset(&out[i], 0, sizeof out[i]);
    return fault;
}

uint32_t actuators_loop_count(void) {
    return loops;
}

/* ---------------------------------------------------------------- PWM */

static void pwm_setup(void) {
    /* 1 카운트 = 1 us 가 되도록 분주: 레벨 값을 펄스 폭(us) 그대로 쓴다 */
    float div = (float)clock_get_hz(clk_sys) / 1e6f;
    uint32_t wrap = 1000000u / FW_PWM_FREQUENCY_HZ - 1u;
    for (int i = 0; i < FW_N_ACT; i++) {
        if (ACTS[i].kind != FW_ACT_PWM)
            continue;
        uint pin = (uint)ACTS[i].pin;
        uint slice = pwm_gpio_to_slice_num(pin);
        gpio_set_function(pin, GPIO_FUNC_PWM);
        pwm_set_clkdiv(slice, div);
        pwm_set_wrap(slice, (uint16_t)wrap);
        pwm_set_gpio_level(pin, 0); /* 시작 시 펄스 없음 = 토크 해제 */
        pwm_set_enabled(slice, true);
    }
}

/* ---------------------------------------------------------------- core1 */

typedef struct {
    uint8_t idx[FW_N_ACT > 0 ? FW_N_ACT : 1]; /* STS 축의 액추에이터 인덱스 */
    uint8_t ids[FW_N_ACT > 0 ? FW_N_ACT : 1];
    uint8_t n;
} sts_list_t;

static sts_list_t sts;

static void sts_torque(uint32_t axes, uint8_t on) {
    uint8_t ids[FW_N_ACT], data[FW_N_ACT], n = 0;
    for (uint8_t k = 0; k < sts.n; k++) {
        if (axes & (1u << sts.idx[k])) {
            ids[n] = sts.ids[k];
            data[n++] = on;
        }
    }
    if (n)
        sts_sync_write(STS_TORQUE_ENABLE, 1, ids, data, n);
}

static void sts_goal(uint32_t axes, const int32_t *targets) {
    uint8_t ids[FW_N_ACT], data[FW_N_ACT * 6], n = 0;
    for (uint8_t k = 0; k < sts.n; k++) {
        uint8_t i = sts.idx[k];
        if (!(axes & (1u << i)))
            continue;
        uint16_t pos = sts_encode_pos(targets[i]);
        uint8_t *d = &data[n * 6];
        d[0] = (uint8_t)pos; /* STS 레지스터는 Little-endian */
        d[1] = (uint8_t)(pos >> 8);
        d[2] = d[3] = 0;     /* GOAL_TIME */
        d[4] = (uint8_t)FW_STS_GOAL_SPEED;
        d[5] = (uint8_t)(FW_STS_GOAL_SPEED >> 8);
        ids[n++] = sts.ids[k];
    }
    if (n)
        sts_sync_write(STS_GOAL_POSITION, 6, ids, data, n);
}

static void core1_main(void) {
    proto_axis_state_t st[FW_N_ACT];
    uint8_t missed[FW_N_ACT];
    memset(st, 0, sizeof st);
    memset(missed, 0, sizeof missed);
    int32_t targets[FW_N_ACT];
    uint32_t enabled = 0;  /* 실제로 토크를 건 축 */
    uint8_t rr = 0;        /* 라운드로빈 읽기 위치 */
    uint64_t last_off_repeat = 0;

    sts_bus_init();
    /* 시작 시 모든 STS 토크 해제 + 가속도 설정 */
    sts_torque(0xFFFFFFFFu, 0);
    for (uint8_t k = 0; k < sts.n; k++) {
        uint8_t acc = FW_STS_GOAL_ACC;
        sts_write(sts.ids[k], STS_ACC, &acc, 1);
    }

    absolute_time_t next = get_absolute_time();
    for (;;) {
        /* 1) 우편함 가져오기 */
        uint32_t irq = spin_lock_blocking(lock);
        uint32_t want = mbox.enable_mask, fresh = mbox.new_target;
        memcpy(targets, mbox.targets, sizeof targets);
        mbox.new_target = 0;
        spin_unlock(lock, irq);

        /* 2) 토크 해제 (먼저, 안전 우선) */
        uint32_t turn_off = enabled & ~want;
        if (turn_off)
            sts_torque(turn_off, 0);
        uint64_t now = time_us_64();
        uint32_t all_off = ~want & ((FW_N_ACT >= 32) ? 0xFFFFFFFFu : ((1u << FW_N_ACT) - 1u));
        if (all_off && now - last_off_repeat >= (uint64_t)FW_STS_TORQUE_OFF_REPEAT_MS * 1000u) {
            sts_torque(all_off, 0); /* 응답 없는 패킷이라 주기적으로 반복 */
            last_off_repeat = now;
        }

        /* 3) 목표 위치 -> 토크 켜기 (켜기 전에 목표를 먼저 써서 옛 목표로 튀지 않게) */
        uint32_t goal = fresh & want;
        if (goal)
            sts_goal(goal, targets);
        uint32_t turn_on = want & ~enabled;
        if (turn_on)
            sts_torque(turn_on, 1);
        enabled = want;

        /* 4) PWM */
        for (int i = 0; i < FW_N_ACT; i++) {
            if (ACTS[i].kind != FW_ACT_PWM)
                continue;
            bool on = want & (1u << i);
            if (on)
                pwm_set_gpio_level((uint)ACTS[i].pin, (uint16_t)targets[i]);
            else if (!FW_PWM_RELEASE_HOLD)
                pwm_set_gpio_level((uint)ACTS[i].pin, 0);
            st[i].position = on || FW_PWM_RELEASE_HOLD ? targets[i] : 0;
            st[i].flags = 0;
        }

        /* 5) 서보 상태 읽기 (라운드로빈) */
        for (uint8_t r = 0; r < FW_STS_READS_PER_LOOP && sts.n; r++, rr = (uint8_t)((rr + 1) % sts.n)) {
            uint8_t i = sts.idx[rr], buf[STS_PRESENT_BLOCK_LEN], err = 0;
            sts_result_t res = sts_read(sts.ids[rr], STS_PRESENT_POSITION, STS_PRESENT_BLOCK_LEN, buf, &err);
            if (res == STS_OK) {
                missed[i] = 0;
                st[i].position = sts_decode_sign((uint16_t)(buf[0] | buf[1] << 8), 15);
                st[i].velocity = sts_decode_sign((uint16_t)(buf[2] | buf[3] << 8), 15);
                st[i].effort = sts_decode_sign((uint16_t)(buf[4] | buf[5] << 8), 10);
                st[i].temperature_c10 = (int16_t)(buf[7] * 10);
                st[i].flags = err ? AX_SERVO_ERROR : 0;
            } else {
                if (missed[i] < 255)
                    missed[i]++;
                uint16_t fl = st[i].flags & (uint16_t)~(AX_NO_RESPONSE | AX_BAD_PACKET);
                if (res == STS_BAD_PACKET)
                    fl |= AX_BAD_PACKET;
                if (missed[i] >= FW_STS_MAX_MISSED_READS)
                    fl |= AX_NO_RESPONSE;
                st[i].flags = fl;
            }
        }

        /* 6) 상태 공개 */
        bool fault = false;
        for (uint8_t k = 0; k < sts.n; k++)
            fault |= (st[sts.idx[k]].flags & AX_NO_RESPONSE) != 0;
        irq = spin_lock_blocking(lock);
        memcpy(shared_state, st, sizeof st);
        shared_fault = fault;
        spin_unlock(lock, irq);
        loops++;

        next = delayed_by_us(next, FW_STS_LOOP_PERIOD_US);
        if (time_reached(next))
            next = get_absolute_time(); /* 밀렸으면 따라잡지 않고 다시 맞춘다 */
        else
            sleep_until(next);
    }
}

void actuators_init(void) {
    lock = spin_lock_init(spin_lock_claim_unused(true));
    memset(&mbox, 0, sizeof mbox);
    sts.n = 0;
    for (int i = 0; i < FW_N_ACT; i++) {
        if (ACTS[i].kind == FW_ACT_STS) {
            sts.idx[sts.n] = (uint8_t)i;
            sts.ids[sts.n++] = ACTS[i].id;
        }
    }
    pwm_setup();
    multicore_launch_core1(core1_main);
}
