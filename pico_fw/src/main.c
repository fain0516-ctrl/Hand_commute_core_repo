/*
 * Pico 2 (RP2350) 컨트롤러 펌웨어: Pi 5 comm_core <-TCP/W5500-> STS3215 + PWM 서보.
 *
 * core0: W5500 폴링, 프레임 처리(session), Pico 자체 워치독, 상태 LED, 하드웨어 워치독
 * core1: 서보 버스와 PWM (actuators.c)
 *
 * 핀과 파라미터는 모두 fw_config.h (YAML 에서 생성) 에서 온다.
 */
#include <stdio.h>

#include "actuators.h"
#include "diag.h"
#include "fw_config.h"
#include "hardware/gpio.h"
#include "hardware/structs/powman.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "session.h"
#include "w5500.h"

static const fw_actuator_t ACTS[FW_N_ACT] = FW_ACTUATORS_INIT;
static session_t sess;
static uint8_t rxbuf[2048];

static void io_send(void *ctx, const uint8_t *data, uint16_t len) {
    (void)ctx;
    w5500_send(data, len); /* 버퍼가 모자라면 버린다: 다음 주기 상태가 곧 다시 간다 */
}
static void io_apply(void *ctx, const int32_t *t, uint32_t mask) {
    (void)ctx;
    actuators_apply(t, mask);
}
static void io_release(void *ctx, uint32_t mask) {
    (void)ctx;
    actuators_release(mask);
}
static bool io_read_state(void *ctx, proto_axis_state_t *out, uint8_t n) {
    (void)ctx;
    return actuators_read_state(out, n);
}

static void led_init(void) {
#if FW_LED_PIN >= 0
    gpio_init(FW_LED_PIN);
    gpio_set_dir(FW_LED_PIN, GPIO_OUT);
#endif
}

/* 상태 LED: 꺼짐 = W5500 오류, 느린 점멸 = 연결 대기, 빠른 점멸 = 연결됨, 켜짐 = 토크 걸림 */
static void led_update(uint32_t now_ms, bool hw_ok) {
#if FW_LED_PIN >= 0
    bool on;
    if (!hw_ok)
        on = false;
    else if (sess.enabled_mask)
        on = true;
    else if (sess.connected)
        on = (now_ms / 100) & 1;
    else
        on = (now_ms / 500) & 1;
    gpio_put(FW_LED_PIN, on ? FW_LED_ACTIVE_LEVEL : !FW_LED_ACTIVE_LEVEL);
#else
    (void)now_ms;
    (void)hw_ok;
#endif
}

/*
 * 리셋 원인: POWMAN CHIP_RESET 의 HAD_* 비트 (bit16 POR, bit17 BOR=저전압, bit18 RUN 핀, bit26 글리치 검출,
 * bit22-24/28 워치독 계열) 를 그대로 보고하고, SDK 의 watchdog_caused_reboot() 를 bit0 에 더한다.
 * 저전압(BOR)/글리치 리셋이 보이면 전원 잡음이나 서보 돌입 전류를 의심한다.
 */
#define RESET_CAUSE_SDK_WATCHDOG 0x1u
static uint32_t read_reset_cause(void) {
    uint32_t v = powman_hw->chip_reset & 0xFFFF0000u;
    if (watchdog_caused_reboot())
        v |= RESET_CAUSE_SDK_WATCHDOG;
    return v;
}

static bool health_failed(uint32_t now_ms, uint32_t *last) {
    if (now_ms - *last < FW_R_W5500_HEALTH_PERIOD_MS)
        return false;
    *last = now_ms;
    return !w5500_health_check();
}

int main(void) {
    stdio_init_all();
    uint32_t reset_cause = read_reset_cause();
    g_diag[DIAG_RESET_CAUSE] = reset_cause;
    led_init();
    actuators_init(); /* 시작 상태 = 전 축 토크 해제 */

    const session_cfg_t cfg = {
        .acts = ACTS,
        .n_act = FW_N_ACT,
        .n_sts = FW_N_STS,
        .n_pwm = FW_N_PWM,
        .fw_version = FW_VERSION,
        .capabilities = CAP_STATE_EXT | CAP_DIAG | CAP_CMD_SEQ_ECHO,
        .default_timeout_ms = FW_DEFAULT_CMD_TIMEOUT_MS,
        .min_timeout_ms = FW_MIN_CMD_TIMEOUT_MS,
        .max_timeout_ms = FW_MAX_CMD_TIMEOUT_MS,
        .diag_period_ms = FW_R_LINK_DIAG_PERIOD_MS,
        .degrade_hold_ms = FW_R_LINK_DEGRADE_HOLD_MS,
        .cmd_jitter_warn_ms = FW_R_LINK_CMD_JITTER_WARN_MS,
    };
    const session_io_t io = {NULL, io_send, io_apply, io_release, io_read_state};
    session_init(&sess, &cfg, &io);

    bool hw_ok = false;
    uint32_t last_init_try = 0, last_rx_ms = 0, last_log = 0, last_health = 0;
    bool link = false;
    watchdog_enable(FW_HW_WATCHDOG_MS, true);
    printf("pico_fw %04x board=%s reset_cause=%08lx\n", FW_VERSION, FW_BOARD_NAME, (unsigned long)reset_cause);

    for (;;) {
        watchdog_update();
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        uint32_t now_us = time_us_32();
        g_diag[DIAG_UPTIME_MS] = now_ms;

        if (!hw_ok) {
            if (now_ms - last_init_try >= 1000 || last_init_try == 0) {
                last_init_try = now_ms ? now_ms : 1;
                hw_ok = w5500_init();
                printf("w5500 init %s\n", hw_ok ? "ok" : "FAILED (SPI 배선/전원 확인)");
            }
        } else if (health_failed(now_ms, &last_health)) {
            /* SPI 가 계속 이상하거나 칩이 리셋됨: 연결을 버리고 (= 토크 해제) 칩을 다시 초기화 */
            printf("w5500 health check failed, re-init\n");
            if (sess.connected)
                session_on_disconnect(&sess);
            w5500_note_reinit();
            hw_ok = w5500_init();
            link = false;
            last_init_try = now_ms ? now_ms : 1;
        } else {
            bool l = w5500_link_up();
            if (l != link) {
                link = l;
                printf("ethernet link %s\n", l ? "up" : "down");
                if (!l && w5500_has_client()) {
                    w5500_drop_active();
                    session_on_disconnect(&sess);
                }
            }
            uint16_t n = 0;
            net_event_t ev = w5500_poll(rxbuf, sizeof rxbuf, &n);
            if (ev == NET_EV_CONNECTED) {
                if (sess.connected)
                    session_on_disconnect(&sess); /* 이전 연결 대체 */
                session_on_connect(&sess, now_ms);
                last_rx_ms = now_ms;
                printf("client connected\n");
            } else if (ev == NET_EV_DISCONNECTED) {
                session_on_disconnect(&sess);
                printf("client disconnected\n");
            }
            if (n) {
                last_rx_ms = now_ms;
                session_feed(&sess, rxbuf, n, now_ms, now_us);
            }
            if (sess.connected && now_ms - last_rx_ms > FW_NET_LINK_IDLE_MS) {
                printf("link idle %u ms, dropping client\n", (unsigned)(now_ms - last_rx_ms));
                w5500_drop_active();
                session_on_disconnect(&sess);
            }
        }
        session_tick(&sess, now_ms);
        led_update(now_ms, hw_ok);

        if (now_ms - last_log >= 5000) {
            last_log = now_ms;
            printf("stat rx=%lu tx=%lu cmd=%lu wd=%lu estop=%lu crc=%lu loops=%lu en=%08lx\n",
                   (unsigned long)sess.rx_frames, (unsigned long)sess.tx_frames, (unsigned long)sess.cmds,
                   (unsigned long)sess.watchdog_trips, (unsigned long)sess.estops,
                   (unsigned long)sess.dec.crc_errors, (unsigned long)actuators_loop_count(),
                   (unsigned long)sess.enabled_mask);
        }
    }
}
