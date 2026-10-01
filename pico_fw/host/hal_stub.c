/*
 * 호스트 시뮬레이터용 하드웨어 대체 구현.
 * - UART: 펌웨어가 서보 버스로 보낸 바이트를 "bus <ms> <hex>" 로 기록하고, 가짜 STS3215 서보가 응답한다.
 *         FW_STS_ECHO 이면 실제 회로처럼 자기 송신 바이트가 먼저 RX 로 돌아온다.
 * - PWM : 레벨(펄스 폭 us)이 바뀔 때 "pwm <ms> <gpio> <level>" 로 기록한다.
 */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "fw_config.h"
#include "host_sdk.h"

struct uart_inst { int n; };
static struct uart_inst u0 = {0}, u1 = {1};
uart_inst_t *const uart0_inst = &u0, *const uart1_inst = &u1;

uint64_t time_us_64(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

void sleep_us(uint64_t us) {
    struct timespec ts = {(time_t)(us / 1000000u), (long)(us % 1000000u) * 1000};
    nanosleep(&ts, NULL);
}

static uint32_t now_ms(void) { return (uint32_t)(time_us_64() / 1000u); }

void gpio_init(uint pin) { (void)pin; }
void gpio_set_dir(uint pin, bool out) { (void)pin; (void)out; }
void gpio_put(uint pin, bool v) { (void)pin; (void)v; }
void gpio_pull_up(uint pin) { (void)pin; }
void gpio_set_function(uint pin, int fn) { (void)pin; (void)fn; }

uint32_t clock_get_hz(int clk) { (void)clk; return 150000000u; }
uint pwm_gpio_to_slice_num(uint pin) { return (pin >> 1) & 7u; }
void pwm_set_clkdiv(uint slice, float div) { (void)slice; (void)div; }
void pwm_set_wrap(uint slice, uint16_t wrap) { (void)slice; (void)wrap; }
void pwm_set_enabled(uint slice, bool en) { (void)slice; (void)en; }

static int pwm_level[64];
static bool pwm_seen[64];
void pwm_set_gpio_level(uint pin, uint16_t level) {
    if (pin < 64 && (!pwm_seen[pin] || pwm_level[pin] != level)) {
        pwm_seen[pin] = true;
        pwm_level[pin] = level;
        printf("pwm %u %u %u\n", now_ms(), pin, level);
    }
}

/* ---------------------------------------------------------------- 가짜 STS3215 */

typedef struct {
    bool present;
    uint8_t torque;
    int32_t goal, pos;
} fake_servo_t;

static fake_servo_t servo[254];
static uint8_t rxq[4096];
static size_t rx_head, rx_tail;

static void rx_push(uint8_t b) {
    rxq[rx_tail++ % sizeof rxq] = b;
}

static void servo_reply(uint8_t id, const uint8_t *data, uint8_t len) {
    uint8_t sum = (uint8_t)(id + len + 2);
    rx_push(0xFF);
    rx_push(0xFF);
    rx_push(id);
    rx_push((uint8_t)(len + 2));
    rx_push(0); /* 오류 없음 */
    for (uint8_t i = 0; i < len; i++) {
        rx_push(data[i]);
        sum = (uint8_t)(sum + data[i]);
    }
    rx_push((uint8_t)~sum);
}

static void servo_write(uint8_t id, uint8_t addr, const uint8_t *d, uint8_t len) {
    fake_servo_t *s = &servo[id];
    if (!s->present)
        return;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t a = (uint8_t)(addr + i);
        if (a == 40) {
            s->torque = d[i];
        } else if (a == 42 && i + 1 < len) {
            uint16_t raw = (uint16_t)(d[i] | d[i + 1] << 8);
            s->goal = (raw & 0x8000) ? -(int32_t)(raw & 0x7FFF) : raw;
        }
    }
    if (s->torque)
        s->pos = s->goal; /* 이상적 추종 */
}

static void servo_handle(const uint8_t *p, size_t n) {
    if (n < 6 || p[0] != 0xFF || p[1] != 0xFF)
        return;
    uint8_t id = p[2], len = p[3], ins = p[4];
    if ((size_t)len + 4 != n)
        return;
    if (ins == 0x03) {
        servo_write(id, p[5], p + 6, (uint8_t)(len - 3));
    } else if (ins == 0x83) {
        uint8_t addr = p[5], dl = p[6];
        for (size_t o = 7; o + 1 + dl <= n - 1; o += 1u + dl)
            servo_write(p[o], addr, p + o + 1, dl);
    } else if (ins == 0x02 && id < 254 && servo[id].present) {
        uint8_t out[8] = {0};
        fake_servo_t *s = &servo[id];
        uint16_t pos = s->pos < 0 ? (uint16_t)((-s->pos) | 0x8000) : (uint16_t)s->pos;
        out[0] = (uint8_t)pos;
        out[1] = (uint8_t)(pos >> 8);
        out[6] = 120; /* 12.0 V */
        out[7] = 30;  /* 30 C */
        servo_reply(id, out, p[6] < 8 ? p[6] : 8);
    }
}

uint uart_init(uart_inst_t *u, uint baud) {
    (void)u;
    static const fw_actuator_t acts[FW_N_ACT] = FW_ACTUATORS_INIT;
    for (int i = 0; i < FW_N_ACT; i++) {
        if (acts[i].kind == FW_ACT_STS) {
            servo[acts[i].id].present = true;
            servo[acts[i].id].pos = servo[acts[i].id].goal = 2048;
        }
    }
    return baud;
}
void uart_set_format(uart_inst_t *u, uint bits, uint stop, int parity) { (void)u; (void)bits; (void)stop; (void)parity; }
void uart_set_fifo_enabled(uart_inst_t *u, bool en) { (void)u; (void)en; }
bool uart_is_readable(uart_inst_t *u) { (void)u; return rx_head != rx_tail; }
char uart_getc(uart_inst_t *u) { (void)u; return (char)rxq[rx_head++ % sizeof rxq]; }
void uart_tx_wait_blocking(uart_inst_t *u) { (void)u; }

void uart_write_blocking(uart_inst_t *u, const uint8_t *src, size_t len) {
    (void)u;
    printf("bus %u ", now_ms());
    for (size_t i = 0; i < len; i++)
        printf("%02x", src[i]);
    printf("\n");
#if FW_STS_ECHO
    for (size_t i = 0; i < len; i++)
        rx_push(src[i]);
#endif
    servo_handle(src, len);
}
