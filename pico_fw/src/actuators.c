/*
 * 액추에이터 루프 (core1). 강건성 처리:
 *   - 지령 변화율 제한 (slew): 큰 계단/튄 지령에도 급격히 움직이지 않음. 토크를 켤 때는 측정 위치에서 출발
 *   - 피드백 타당성 검사: 체크섬(8비트)을 우연히 통과한 깨진 샘플을 위치 점프/온도/전압 범위로 걸러냄
 *   - 관측기: 측정이 빠진 동안 목표를 향한 1차 지연 모델로 위치 추정 (ESTIMATED 표시, estimate_max_ms 까지)
 *   - 신선도: 축별 age_ms, STALE -> NO_RESPONSE -> lost_action(hold/torque_off), 버스 전체 무응답 -> bus_dead_action
 *   - 지령 재확인: 응답 없는 SYNC WRITE 가 사라졌거나 서보가 저전압 리셋된 경우 레지스터를 읽어 다시 씀
 *   - 보호: 과열/과부하가 protect_ms 동안 계속되면 그 축 토크 해제 (Pi 가 토크를 해제했다가 다시 켤 때까지 유지)
 * 모든 임계값은 config/controller.yaml 의 robustness 에서 온다.
 */
#include "actuators.h"

#include <stdlib.h>
#include <string.h>

#include "diag.h"
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
#define ALL_MASK ((FW_N_ACT >= 32) ? 0xFFFFFFFFu : ((1u << FW_N_ACT) - 1u))

/* ---------------------------------------------------------------- core0 <-> core1 우편함 */

typedef struct {
    int32_t targets[FW_N_ACT];
    uint32_t enable_mask; /* 토크를 걸 축 (Pi 지령 기준) */
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

/* ---------------------------------------------------------------- 축 상태 (core1 전용) */

typedef struct {
    /* 지령 */
    int32_t cur;          /* 변화율 제한을 거친 실제 출력 목표 */
    int32_t sent;         /* 서보에 마지막으로 쓴 목표 */
    bool on;              /* 실제로 토크를 걸었는지 */
    bool sent_valid;
    bool slewed;
    /* 측정 */
    bool have_good;
    uint64_t last_good_us;
    int32_t meas_pos, meas_vel, meas_load;
    uint8_t meas_volt, meas_temp, servo_err;
    uint8_t reject_streak;
    bool last_bad_packet, last_implausible;
    /* 추정 */
    float est;
    float est_vel;
    /* 보호/재확인 */
    uint64_t over_since_us;
    bool protect;          /* 보호로 토크 해제 (Pi 가 해제 후 다시 켤 때까지) */
    bool mismatch;
    bool held;
    bool fb_off;           /* 피드백 끊김으로 토크 해제 (신선한 측정이 다시 들어올 때까지) */
} axis_t;

static axis_t ax[FW_N_ACT];
static uint8_t sts_idx[FW_N_ACT];
static uint8_t n_sts;
static uint8_t rr, vr;              /* 읽기 / 재확인 라운드로빈 위치 */
static uint64_t last_off_repeat, last_step_us, last_verify_us;

static uint32_t sts_mask(void) {
    uint32_t m = 0;
    for (uint8_t k = 0; k < n_sts; k++)
        m |= 1u << sts_idx[k];
    return m;
}

static void sts_torque(uint32_t axes, uint8_t on) {
    uint8_t ids[FW_N_ACT], data[FW_N_ACT], n = 0;
    for (uint8_t k = 0; k < n_sts; k++) {
        uint8_t i = sts_idx[k];
        if (axes & (1u << i)) {
            ids[n] = ACTS[i].id;
            data[n++] = on;
        }
    }
    if (n)
        sts_sync_write(STS_TORQUE_ENABLE, 1, ids, data, n);
}

static void pack_goal(uint8_t *d, int32_t target) {
    uint16_t pos = sts_encode_pos(target);
    d[0] = (uint8_t)pos; /* STS 레지스터는 Little-endian */
    d[1] = (uint8_t)(pos >> 8);
    d[2] = d[3] = 0; /* GOAL_TIME */
    d[4] = (uint8_t)FW_STS_GOAL_SPEED;
    d[5] = (uint8_t)(FW_STS_GOAL_SPEED >> 8);
}

static void sts_goal(uint32_t axes) {
    uint8_t ids[FW_N_ACT], data[FW_N_ACT * 6], n = 0;
    for (uint8_t k = 0; k < n_sts; k++) {
        uint8_t i = sts_idx[k];
        if (!(axes & (1u << i)))
            continue;
        pack_goal(&data[n * 6], ax[i].cur);
        ids[n++] = ACTS[i].id;
        ax[i].sent = ax[i].cur;
        ax[i].sent_valid = true;
    }
    if (n)
        sts_sync_write(STS_GOAL_POSITION, 6, ids, data, n);
}

static uint32_t age_ms(const axis_t *a, uint64_t now) {
    return a->have_good ? (uint32_t)((now - a->last_good_us) / 1000u) : 0xFFFFFFFFu;
}

/* 변화율 제한: cur 를 req 쪽으로 최대 rate*dt 만큼 */
static void slew(axis_t *a, int32_t req, uint32_t rate_per_s, uint64_t dt_us) {
    int64_t max_step = (int64_t)rate_per_s * (int64_t)dt_us / 1000000;
    if (max_step < 1)
        max_step = 1;
    int64_t d = (int64_t)req - a->cur;
    a->slewed = false;
    if (d > max_step) {
        a->cur += (int32_t)max_step;
        a->slewed = true;
    } else if (d < -max_step) {
        a->cur -= (int32_t)max_step;
        a->slewed = true;
    } else {
        a->cur = req;
    }
    if (a->slewed)
        diag_inc(DIAG_SLEW_LIMITED);
}

/* 서보 1개 읽기 + 타당성 검사 */
static void read_servo(uint8_t i, uint64_t now) {
    axis_t *a = &ax[i];
    uint8_t buf[STS_PRESENT_BLOCK_LEN], err = 0;
    sts_result_t res = sts_read(ACTS[i].id, STS_PRESENT_POSITION, STS_PRESENT_BLOCK_LEN, buf, &err);
    a->last_bad_packet = res == STS_BAD_PACKET;
    if (res != STS_OK)
        return;
    int32_t pos = sts_decode_sign((uint16_t)(buf[0] | buf[1] << 8), 15);
    int32_t vel = sts_decode_sign((uint16_t)(buf[2] | buf[3] << 8), 15);
    int32_t load = sts_decode_sign((uint16_t)(buf[4] | buf[5] << 8), 10);
    uint8_t volt = buf[6], temp = buf[7];

    bool ok = pos >= -FW_R_PLAUSIBILITY_JUMP_MARGIN_TICKS && pos <= 4095 + FW_R_PLAUSIBILITY_JUMP_MARGIN_TICKS &&
              temp <= FW_R_PLAUSIBILITY_TEMP_VALID_MAX_C && volt >= FW_R_PLAUSIBILITY_VOLTAGE_VALID_DV_MIN &&
              volt <= FW_R_PLAUSIBILITY_VOLTAGE_VALID_DV_MAX && abs(load) <= 1000;
    if (ok && a->have_good) {
        uint64_t dt = now - a->last_good_us;
        int64_t allowed = (int64_t)FW_R_PLAUSIBILITY_MAX_SPEED_TICKS_PER_S * (int64_t)dt / 1000000 +
                          FW_R_PLAUSIBILITY_JUMP_MARGIN_TICKS;
        if (llabs((int64_t)pos - a->meas_pos) > allowed)
            ok = a->reject_streak >= FW_R_PLAUSIBILITY_MAX_REJECT_STREAK; /* 계속 같으면 실제 변화로 인정 */
    }
    if (!ok) {
        a->reject_streak++;
        a->last_implausible = true;
        diag_inc(DIAG_IMPLAUSIBLE_SAMPLES);
        return;
    }
    a->reject_streak = 0;
    a->last_implausible = false;
    a->have_good = true;
    a->last_good_us = now;
    a->meas_pos = pos;
    a->meas_vel = vel;
    a->meas_load = load;
    a->meas_volt = volt;
    a->meas_temp = temp;
    a->servo_err = err;
    a->est = (float)pos;
    a->est_vel = (float)vel;
}

/* 지령 재확인: 서보 1개의 TORQUE_ENABLE/GOAL 을 읽어 우리가 보낸 값과 비교, 다르면 다시 쓴다 */
static void verify_servo(uint8_t i) {
    axis_t *a = &ax[i];
    uint8_t buf[STS_VERIFY_BLOCK_LEN], err = 0;
    if (sts_read(ACTS[i].id, STS_TORQUE_ENABLE, STS_VERIFY_BLOCK_LEN, buf, &err) != STS_OK)
        return;
    bool torque_ok = buf[0] == (a->on ? 1 : 0);
    int32_t goal = sts_decode_sign((uint16_t)(buf[2] | buf[3] << 8), 15);
    bool goal_ok = !a->on || !a->sent_valid || abs(goal - a->sent) <= FW_R_SERVO_BUS_GOAL_TOLERANCE_TICKS;
    a->mismatch = !(torque_ok && goal_ok);
    if (!a->mismatch)
        return;
    diag_inc(DIAG_VERIFY_MISMATCHES);
    if (a->on) { /* 목표 먼저, 그다음 토크 (옛 목표로 튀지 않게) */
        uint8_t d[6];
        pack_goal(d, a->sent_valid ? a->sent : a->cur);
        sts_write(ACTS[i].id, STS_GOAL_POSITION, d, 6);
    }
    uint8_t t = a->on ? 1 : 0;
    sts_write(ACTS[i].id, STS_TORQUE_ENABLE, &t, 1);
}

/* ---------------------------------------------------------------- core1 루프 */

void actuators_bus_start(void) {
    sts_bus_init();
    /* 시작 시 모든 STS 토크 해제 + 가속도 설정 */
    sts_torque(0xFFFFFFFFu, 0);
    last_off_repeat = last_step_us = last_verify_us = time_us_64();
    for (uint8_t k = 0; k < n_sts; k++) {
        uint8_t acc = FW_STS_GOAL_ACC;
        sts_write(ACTS[sts_idx[k]].id, STS_ACC, &acc, 1);
    }
}

void actuators_step(void) {
    uint64_t t_start = time_us_64();
    uint64_t now = t_start;
    uint64_t dt = now - last_step_us;
    last_step_us = now;

    /* 1) 우편함 */
    int32_t req[FW_N_ACT];
    uint32_t irq = spin_lock_blocking(lock);
    uint32_t want = mbox.enable_mask;
    memcpy(req, mbox.targets, sizeof req);
    spin_unlock(lock, irq);

    /* 2) 피드백 상태로 실제로 켤 축 결정 (단계적 대응) */
    uint32_t stsm = sts_mask();
    bool bus_dead = n_sts > 0;
    for (uint8_t k = 0; k < n_sts; k++)
        if (age_ms(&ax[sts_idx[k]], now) <= FW_R_FEEDBACK_BUS_DEAD_MS)
            bus_dead = false;
    uint32_t eff = want;
    for (int i = 0; i < FW_N_ACT; i++) {
        axis_t *a = &ax[i];
        uint32_t bit = 1u << i;
        if (!(want & bit))
            a->protect = false; /* Pi 가 토크를 해제하면 보호 래치 해제 */
        if (a->protect)
            eff &= ~bit;
        a->held = false;
        if (!(stsm & bit))
            continue;
        uint32_t age = age_ms(a, now);
        if (a->fb_off && age <= FW_R_FEEDBACK_STALE_MS)
            a->fb_off = false; /* 측정이 돌아옴: 다시 켤 수 있다 (켤 때는 측정 위치에서 출발) */
        if (!(want & bit))
            continue;
        bool lost = age > FW_R_FEEDBACK_LOST_MS && a->on; /* 켜기 전에는 측정이 없어도 된다 */
        if (bus_dead && a->on) {
            if (FW_R_FEEDBACK_BUS_DEAD_ACTION_OFF)
                a->fb_off = true;
            else
                a->held = true;
        } else if (lost) {
            if (FW_R_FEEDBACK_LOST_ACTION_OFF)
                a->fb_off = true;
            else
                a->held = true;
        }
        if (a->fb_off)
            eff &= ~bit;
    }

    /* 3) 토크 해제 (먼저, 안전 우선) + 해제 상태 주기 반복 */
    uint32_t enabled = 0;
    for (int i = 0; i < FW_N_ACT; i++)
        if (ax[i].on)
            enabled |= 1u << i;
    uint32_t turn_off = enabled & ~eff;
    if (turn_off & stsm) {
        sts_torque(turn_off, 0);
        last_off_repeat = now;
    } else if ((~eff & ALL_MASK & stsm) && now - last_off_repeat >= (uint64_t)FW_STS_TORQUE_OFF_REPEAT_MS * 1000u) {
        sts_torque(~eff & ALL_MASK, 0); /* 응답 없는 패킷이라 주기적으로 반복 */
        last_off_repeat = now;
    }

    /* 4) 목표 계산 (변화율 제한, 고정) -> 목표 쓰기 -> 토크 켜기 */
    uint32_t goal = 0, turn_on = eff & ~enabled;
    for (int i = 0; i < FW_N_ACT; i++) {
        axis_t *a = &ax[i];
        uint32_t bit = 1u << i;
        if (!(eff & bit)) {
            a->on = false;
            a->slewed = false;
            continue;
        }
        bool sts = ACTS[i].kind == FW_ACT_STS;
        if (turn_on & bit) /* 켤 때는 현재 측정 위치에서 출발 (측정이 신선하면) */
            a->cur = (sts && a->have_good && age_ms(a, now) <= FW_R_FEEDBACK_STALE_MS) ? a->meas_pos : req[i];
        if (!a->held)
            slew(a, req[i], sts ? FW_R_SLEW_STS_TICKS_PER_S : FW_R_SLEW_PWM_US_PER_S, dt);
        a->on = true;
        if (sts && (!a->sent_valid || a->cur != a->sent || (turn_on & bit)))
            goal |= bit;
    }
    if (goal)
        sts_goal(goal);
    if (turn_on & stsm)
        sts_torque(turn_on, 1);

    /* 5) PWM */
    for (int i = 0; i < FW_N_ACT; i++) {
        if (ACTS[i].kind != FW_ACT_PWM)
            continue;
        if (ax[i].on)
            pwm_set_gpio_level((uint)ACTS[i].pin, (uint16_t)ax[i].cur);
        else if (!FW_PWM_RELEASE_HOLD)
            pwm_set_gpio_level((uint)ACTS[i].pin, 0);
    }

    /* 6) 서보 상태 읽기 (라운드로빈) + 지령 재확인 */
    for (uint8_t r = 0; r < FW_STS_READS_PER_LOOP && n_sts; r++, rr = (uint8_t)((rr + 1) % n_sts)) {
        if (r && time_us_64() - t_start > FW_R_SERVO_BUS_READ_BUDGET_US)
            break; /* 타임아웃/재시도로 시간이 많이 갔다: 나머지는 다음 루프 */
        read_servo(sts_idx[rr], time_us_64());
    }
    if (n_sts && now - last_verify_us >= (uint64_t)FW_R_SERVO_BUS_VERIFY_PERIOD_MS * 1000u &&
        time_us_64() - t_start <= FW_R_SERVO_BUS_READ_BUDGET_US) {
        last_verify_us = now;
        verify_servo(sts_idx[vr]);
        vr = (uint8_t)((vr + 1) % n_sts);
    }

    /* 7) 관측기 + 플래그/단계 + 보호 */
    now = time_us_64();
    proto_axis_state_t st[FW_N_ACT];
    bool fault = false;
    float alpha = (float)dt / ((float)FW_R_FEEDBACK_OBSERVER_TAU_MS * 1000.0f + (float)dt);
    for (int i = 0; i < FW_N_ACT; i++) {
        axis_t *a = &ax[i];
        proto_axis_state_t *o = &st[i];
        memset(o, 0, sizeof *o);
        uint16_t fl = 0;
        uint8_t level = LEVEL_OK;
        if (a->on)
            fl |= AX_TORQUE_ON;
        if (a->slewed)
            fl |= AX_SLEW_LIMITED;
        if (ACTS[i].kind == FW_ACT_PWM) {
            /* PWM 은 피드백이 없다: 위치 = 출력 중인 펄스 폭 */
            o->position = (a->on || FW_PWM_RELEASE_HOLD) ? a->cur : 0;
            o->age_ms = 0;
        } else {
            uint32_t age = age_ms(a, now);
            if (a->have_good && age > FW_R_FEEDBACK_STALE_MS && age <= FW_R_FEEDBACK_ESTIMATE_MAX_MS) {
                /* 측정 누락 구간: 목표를 향한 1차 지연으로 추정 (측정이 신선하면 측정값 그대로) */
                float target = a->on ? (float)a->cur : a->est;
                float prev = a->est;
                a->est += (target - a->est) * alpha;
                a->est_vel = dt ? (a->est - prev) * 1e6f / (float)dt : 0.0f;
            }
            o->position = a->have_good ? (int32_t)(a->est + (a->est >= 0 ? 0.5f : -0.5f)) : 0;
            o->velocity = (int32_t)a->est_vel;
            o->effort = a->meas_load;
            o->temperature_c10 = (int16_t)(a->meas_temp * 10);
            o->voltage_dv = a->meas_volt;
            o->age_ms = a->have_good ? (uint16_t)(age > 65534u ? 65534u : age) : AGE_UNKNOWN;
            if (a->servo_err)
                fl |= AX_SERVO_ERROR;
            if (a->last_bad_packet)
                fl |= AX_BAD_PACKET;
            if (a->last_implausible) {
                fl |= AX_IMPLAUSIBLE;
                level = LEVEL_DEGRADED;
            }
            if (a->mismatch) {
                fl |= AX_CMD_MISMATCH;
                level = LEVEL_DEGRADED;
            }
            if (!a->have_good || age > FW_R_FEEDBACK_STALE_MS) {
                fl |= AX_STALE | AX_ESTIMATED;
                if (level < LEVEL_DEGRADED)
                    level = LEVEL_DEGRADED;
            }
            if (a->have_good ? age > FW_R_FEEDBACK_LOST_MS : a->on) {
                fl |= AX_NO_RESPONSE;
                fault = true;
                if (level < LEVEL_HOLD)
                    level = LEVEL_HOLD;
            }
            if (a->have_good) { /* 경고/보호 */
                bool hot = a->meas_temp >= FW_R_PROTECTION_TEMP_WARN_C;
                bool volt = a->meas_volt < FW_R_PROTECTION_VOLTAGE_WARN_DV_MIN ||
                            a->meas_volt > FW_R_PROTECTION_VOLTAGE_WARN_DV_MAX;
                bool load = abs(a->meas_load) >= FW_R_PROTECTION_LOAD_WARN_PERMILLE;
                if (hot)
                    fl |= AX_OVERTEMP;
                if (volt)
                    fl |= AX_VOLTAGE;
                if (load)
                    fl |= AX_OVERLOAD;
                if ((hot || volt || load) && level < LEVEL_DEGRADED)
                    level = LEVEL_DEGRADED;
                bool severe = a->meas_temp >= FW_R_PROTECTION_TEMP_OFF_C ||
                              abs(a->meas_load) >= FW_R_PROTECTION_LOAD_OFF_PERMILLE;
                if (!severe || !a->on) {
                    a->over_since_us = 0;
                } else if (!a->over_since_us) {
                    a->over_since_us = now;
                } else if (now - a->over_since_us >= (uint64_t)FW_R_PROTECTION_PROTECT_MS * 1000u && !a->protect) {
                    a->protect = true; /* 다음 루프에서 토크 해제 */
                    diag_inc(DIAG_PROTECT_TRIPS);
                }
            }
        }
        if (a->held) {
            fl |= AX_HOLD;
            if (level < LEVEL_HOLD)
                level = LEVEL_HOLD;
        }
        if (a->protect) {
            fl |= AX_PROTECT_OFF;
            level = LEVEL_SAFE_OFF;
        }
        if (a->fb_off)
            level = LEVEL_SAFE_OFF; /* 피드백 끊김으로 해제 (flags 에는 NO_RESPONSE) */
        o->flags = fl;
        o->level = level;
    }

    /* 8) 상태 공개 + 루프 시간 */
    irq = spin_lock_blocking(lock);
    memcpy(shared_state, st, sizeof st);
    shared_fault = fault;
    spin_unlock(lock, irq);
    loops++;
    uint32_t took = (uint32_t)(time_us_64() - t_start);
    diag_max(DIAG_LOOP_MAX_US, took);
    if (took > FW_STS_LOOP_PERIOD_US)
        diag_inc(DIAG_LOOP_OVERRUNS);
}

static void core1_main(void) {
    actuators_bus_start();
    absolute_time_t next = get_absolute_time();
    for (;;) {
        actuators_step();
        next = delayed_by_us(next, FW_STS_LOOP_PERIOD_US);
        if (time_reached(next))
            next = get_absolute_time(); /* 밀렸으면 따라잡지 않고 다시 맞춘다 */
        else
            sleep_until(next);
    }
}

void actuators_setup(void) {
    lock = spin_lock_init(spin_lock_claim_unused(true));
    memset(&mbox, 0, sizeof mbox);
    memset(ax, 0, sizeof ax);
    n_sts = 0;
    for (int i = 0; i < FW_N_ACT; i++) {
        if (ACTS[i].kind == FW_ACT_STS)
            sts_idx[n_sts++] = (uint8_t)i;
    }
    pwm_setup();
}

void actuators_init(void) {
    actuators_setup();
    multicore_launch_core1(core1_main);
}
