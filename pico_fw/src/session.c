#include "session.h"

#include <string.h>

#include "diag.h"

static uint32_t all_mask(const session_t *s) {
    return s->cfg.n_act >= 32 ? 0xFFFFFFFFu : ((1u << s->cfg.n_act) - 1u);
}

static void send_frame(session_t *s, uint8_t type, uint8_t flags, const uint8_t *payload, uint16_t len) {
    if (!s->connected)
        return;
    s->tx_seq++;
    size_t n = proto_encode(s->txbuf, type, flags, s->tx_seq, s->now_us, payload, len);
    s->io.send(s->io.ctx, s->txbuf, (uint16_t)n);
    s->tx_frames++;
}

static void send_error(session_t *s, uint16_t code, const char *msg) {
    s->last_error = code;
    uint32_t bit = 1u << (code & 31);
    if (s->error_sent_mask & bit)
        return;
    s->error_sent_mask |= bit;
    uint8_t p[2 + 64];
    size_t m = strlen(msg);
    if (m > 64)
        m = 64;
    put_u16(p, code);
    memcpy(p + 2, msg, m);
    send_frame(s, MSG_ERROR, 0, p, (uint16_t)(2 + m));
}

static void release(session_t *s, uint32_t mask) {
    mask &= all_mask(s);
    s->io.release(s->io.ctx, mask);
    s->enabled_mask &= ~mask;
}

static void release_all(session_t *s) {
    release(s, all_mask(s));
}

/* 일시적 이상(seq gap, 지령 지터)을 degrade_hold_ms 동안 DEGRADED 로 유지 */
static void mark_degraded(session_t *s) {
    s->had_gap = true;
    s->last_gap_ms = s->now_ms;
}

static uint16_t compute_status(session_t *s, const proto_axis_state_t *ax, uint8_t n, bool fault) {
    uint16_t status = s->status & (ST_WATCHDOG | ST_ESTOP | ST_HELLO_DONE);
    if (s->enabled_mask)
        status |= ST_TORQUE_ENABLED;
    if (fault)
        status |= ST_BUS_FAULT;
    uint8_t level = LEVEL_OK;
    for (uint8_t i = 0; i < n; i++)
        if (ax[i].level > level)
            level = ax[i].level;
    if (s->had_gap && (uint32_t)(s->now_ms - s->last_gap_ms) < s->cfg.degrade_hold_ms) {
        status |= ST_SEQ_GAP;
        if (level < LEVEL_DEGRADED)
            level = LEVEL_DEGRADED;
    }
    if (status & (ST_WATCHDOG | ST_ESTOP))
        level = LEVEL_SAFE_OFF;
    s->level = level;
    g_diag[DIAG_FAULT_LEVEL] = level;
    return (uint16_t)(status | (level << ST_LEVEL_SHIFT));
}

static void send_state(session_t *s) {
    static proto_axis_state_t ax[PROTO_MAX_ACTUATORS];
    uint8_t n = s->cfg.n_act;
    bool fault = s->io.read_state(s->io.ctx, ax, n);
    uint16_t status = compute_status(s, ax, n, fault);

    uint8_t *p = s->txbuf + PROTO_HEADER_SIZE; /* payload 를 송신 버퍼에 바로 만든다 */
    put_u16(p, status);
    put_u16(p + 2, s->last_error);
    p[4] = n;
    p[5] = PROTO_AXIS_STATE_SIZE;
    put_u16(p + 6, s->last_cmd_seq);
    uint8_t *q = p + PROTO_STATE_HEAD_SIZE;
    for (uint8_t i = 0; i < n; i++, q += PROTO_AXIS_STATE_SIZE) {
        uint16_t fl = ax[i].flags; /* AX_TORQUE_ON 은 액추에이터 쪽 실제 상태 (보호/피드백 끊김 해제 반영) */
        if (s->clamp_mask & (1u << i))
            fl |= AX_CLAMPED;
        put_u32(q, (uint32_t)ax[i].position);
        put_u32(q + 4, (uint32_t)ax[i].velocity);
        put_u32(q + 8, (uint32_t)ax[i].effort);
        put_u16(q + 12, (uint16_t)ax[i].temperature_c10);
        put_u16(q + 14, fl);
        put_u16(q + 16, ax[i].age_ms);
        q[18] = ax[i].voltage_dv;
        q[19] = ax[i].level;
        q[20] = q[21] = q[22] = q[23] = 0;
    }
    send_frame(s, MSG_STATE, 0, p, (uint16_t)(PROTO_STATE_HEAD_SIZE + n * PROTO_AXIS_STATE_SIZE));
}

static void send_diag(session_t *s) {
    uint8_t p[4 + 4 * DIAG_COUNT];
    p[0] = DIAG_COUNT;
    p[1] = p[2] = p[3] = 0;
    g_diag[DIAG_RX_FRAMES] = s->rx_frames;
    g_diag[DIAG_CRC_ERRORS] = s->dec.crc_errors;
    g_diag[DIAG_DROPPED_BYTES] = s->dec.dropped_bytes;
    g_diag[DIAG_WATCHDOG_TRIPS] = s->watchdog_trips;
    g_diag[DIAG_ESTOPS] = s->estops;
    g_diag[DIAG_CMD_INTERVAL_MEAN_US] = s->interval_n ? (uint32_t)(s->interval_sum_us / s->interval_n) : 0;
    for (int i = 0; i < DIAG_COUNT; i++)
        put_u32(p + 4 + 4 * i, g_diag[i]);
    send_frame(s, MSG_DIAG, 0, p, sizeof p);
    /* 구간 통계는 보고 후 초기화 (누적 카운터는 유지) */
    g_diag[DIAG_CMD_INTERVAL_MAX_US] = 0;
    g_diag[DIAG_LOOP_MAX_US] = 0;
    s->interval_sum_us = 0;
    s->interval_n = 0;
}

static void handle_hello(session_t *s, const proto_frame_t *f) {
    if (f->payload_len < 4) {
        send_error(s, ERR_BAD_PAYLOAD, "HELLO payload too short");
        return;
    }
    uint16_t t = get_u16(f->payload + 2);
    if (t < s->cfg.min_timeout_ms)
        t = s->cfg.min_timeout_ms;
    if (t > s->cfg.max_timeout_ms)
        t = s->cfg.max_timeout_ms;
    s->cmd_timeout_ms = t;
    s->status |= ST_HELLO_DONE;
    uint8_t p[8];
    put_u16(p, s->cfg.fw_version);
    p[2] = s->cfg.n_sts;
    p[3] = s->cfg.n_pwm;
    put_u32(p + 4, s->cfg.capabilities);
    send_frame(s, MSG_HELLO_ACK, 0, p, sizeof p);
    if (s->cfg.diag_period_ms)
        send_diag(s); /* 연결 직후 리셋 원인 등을 바로 알린다 */
}

static void track_cmd_interval(session_t *s) {
    if (s->have_cmd_us) {
        uint32_t dt = s->now_us - s->last_cmd_us;
        diag_max(DIAG_CMD_INTERVAL_MAX_US, dt);
        s->interval_sum_us += dt;
        s->interval_n++;
        if (dt > (uint32_t)s->cfg.cmd_jitter_warn_ms * 1000u)
            mark_degraded(s);
    }
    s->last_cmd_us = s->now_us;
    s->have_cmd_us = true;
}

static void handle_cmd(session_t *s, const proto_frame_t *f) {
    if (f->payload_len < 4) {
        send_error(s, ERR_BAD_PAYLOAD, "ACTUATOR_CMD payload too short");
        return;
    }
    uint8_t mode = f->payload[0];
    uint8_t count = f->payload[1];
    if (count > PROTO_MAX_ACTUATORS || f->payload_len < 4u + 4u * count) {
        send_error(s, ERR_BAD_PAYLOAD, "ACTUATOR_CMD count/length mismatch");
        return;
    }
    s->cmds++;
    s->last_cmd_ms = s->now_ms;
    s->last_cmd_seq = f->seq;
    track_cmd_interval(s);
    if (count != s->cfg.n_act)
        send_error(s, ERR_COUNT_MISMATCH, "actuator count differs from firmware config");
    uint8_t n = count < s->cfg.n_act ? count : s->cfg.n_act;
    const uint8_t *v = f->payload + 4;

    if (mode == CMD_MODE_POSITION) {
        int32_t targets[PROTO_MAX_ACTUATORS];
        uint32_t mask = 0, clamped = 0;
        for (uint8_t i = 0; i < n; i++) {
            int32_t x = (int32_t)get_u32(v + 4 * i);
            const fw_actuator_t *a = &s->cfg.acts[i];
            if (x < a->raw_min || x > a->raw_max) {
                x = x < a->raw_min ? a->raw_min : a->raw_max;
                clamped |= 1u << i;
            }
            targets[i] = x;
            mask |= 1u << i;
        }
        s->clamp_mask = clamped;
        if (clamped)
            send_error(s, ERR_VALUE_CLAMPED, "position command clamped to raw_min/raw_max");
        s->io.apply(s->io.ctx, targets, mask);
        s->enabled_mask |= mask;
        s->status &= (uint16_t)~(ST_WATCHDOG | ST_ESTOP);
    } else if (mode == CMD_MODE_TORQUE) {
        /* STS3215 에는 토크 제어 모드가 없다. 0 (= 토크 해제) 만 지원 */
        uint32_t zero = 0;
        bool nonzero = false;
        for (uint8_t i = 0; i < n; i++) {
            if ((int32_t)get_u32(v + 4 * i) == 0)
                zero |= 1u << i;
            else
                nonzero = true;
        }
        release(s, zero);
        if (nonzero)
            send_error(s, ERR_UNSUPPORTED_MODE, "nonzero torque not supported (position servos); axis unchanged");
        if (f->flags & CMD_FLAG_WATCHDOG_TRIPPED)
            s->status |= ST_WATCHDOG;
    } else {
        send_error(s, ERR_UNSUPPORTED_MODE, "unsupported command mode");
    }
    send_state(s);
}

static void on_frame(void *ctx, const proto_frame_t *f) {
    session_t *s = ctx;
    s->rx_frames++;
    /* Pi 는 모든 프레임에 seq 를 1씩 붙인다. 건너뛰면 그 사이 프레임이 CRC 오류로 버려진 것 */
    if (s->have_rx_seq && f->seq != (uint16_t)(s->last_rx_seq + 1)) {
        diag_inc(DIAG_SEQ_GAPS);
        mark_degraded(s);
    }
    s->have_rx_seq = true;
    s->last_rx_seq = f->seq;
    switch (f->msg_type) {
    case MSG_HELLO:
        handle_hello(s, f);
        break;
    case MSG_HEARTBEAT:
        send_frame(s, MSG_HEARTBEAT, 0, NULL, 0);
        break;
    case MSG_ACTUATOR_CMD:
        handle_cmd(s, f);
        break;
    case MSG_ESTOP:
        s->estops++;
        release_all(s);
        s->status |= ST_ESTOP;
        break;
    default:
        break; /* 모르는 타입은 무시 (향후 확장) */
    }
}

void session_init(session_t *s, const session_cfg_t *cfg, const session_io_t *io) {
    memset(s, 0, sizeof *s);
    s->cfg = *cfg;
    s->io = *io;
    s->cmd_timeout_ms = cfg->default_timeout_ms;
    proto_decoder_reset(&s->dec);
}

void session_on_connect(session_t *s, uint32_t now_ms) {
    proto_decoder_reset(&s->dec);
    s->connected = true;
    s->now_ms = now_ms;
    s->last_cmd_ms = now_ms;
    s->last_diag_ms = now_ms;
    s->error_sent_mask = 0;
    s->cmd_timeout_ms = s->cfg.default_timeout_ms;
    s->status &= (uint16_t)~ST_HELLO_DONE;
    s->have_rx_seq = false;
    s->have_cmd_us = false;
}

void session_on_disconnect(session_t *s) {
    if (s->connected)
        diag_inc(DIAG_LINK_DROPS);
    s->connected = false;
    release_all(s);
}

void session_feed(session_t *s, const uint8_t *data, uint16_t len, uint32_t now_ms, uint32_t now_us) {
    s->now_ms = now_ms;
    s->now_us = now_us;
    proto_decoder_feed(&s->dec, data, len, on_frame, s);
}

void session_tick(session_t *s, uint32_t now_ms) {
    s->now_ms = now_ms;
    if (s->enabled_mask && (uint32_t)(now_ms - s->last_cmd_ms) > s->cmd_timeout_ms) {
        release_all(s);
        s->status |= ST_WATCHDOG;
        s->watchdog_trips++;
    }
    if (s->connected && s->cfg.diag_period_ms && (uint32_t)(now_ms - s->last_diag_ms) >= s->cfg.diag_period_ms) {
        s->last_diag_ms = now_ms;
        send_diag(s);
    }
}
