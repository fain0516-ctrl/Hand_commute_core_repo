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

/* ---------------------------------------------------------------- 가짜 STS3215 + 고장 주입 */

typedef struct {
    bool present;
    bool dead;            /* 응답 안 함 (단선/전원 끊김) */
    uint8_t reg[64];      /* 40 TORQUE, 41 ACC, 42-43 GOAL, 56-63 PRESENT 블록 */
    double pos;           /* 실제 위치 (ticks) */
    double vel;
    uint64_t last_us;
    int spike_left;       /* 남은 튄 샘플 수 */
    int32_t spike_ticks;
    int ignore_writes;    /* 이 서보가 무시할 다음 쓰기 패킷 수 (잡음으로 놓친 SYNC WRITE) */
    uint8_t temp, volt;
    int16_t load;         /* 천분율 */
} fake_servo_t;

static fake_servo_t servo[254];
static uint8_t rxq[8192];
static size_t rx_head, rx_tail;

/* 고장 주입 설정 (sim_main 의 stdin "fault ..." 명령으로 바뀜) */
static uint32_t f_uart_flip_ppm, f_uart_drop_ppm, f_echo_flip_ppm, f_noise_ppm;
static uint64_t rng = 0x9E3779B97F4A7C15ull;
static uint32_t rnd(void) {
    rng ^= rng << 13;
    rng ^= rng >> 7;
    rng ^= rng << 17;
    return (uint32_t)(rng >> 16);
}
static bool chance(uint32_t ppm) { return ppm && rnd() % 1000000u < ppm; }

static void rx_push_raw(uint8_t b) {
    rxq[rx_tail++ % sizeof rxq] = b;
}
/* 선로 잡음: 비트 뒤집힘, 바이트 유실, 버스 유휴 중 잡음 바이트 */
static void rx_push(uint8_t b, uint32_t flip_ppm) {
    if (chance(f_uart_drop_ppm))
        return;
    if (chance(flip_ppm))
        b ^= (uint8_t)(1u << (rnd() % 8));
    rx_push_raw(b);
    if (chance(f_noise_ppm))
        rx_push_raw((uint8_t)rnd());
}

static int32_t dec15(uint16_t raw) { return (raw & 0x8000) ? -(int32_t)(raw & 0x7FFF) : raw; }

/* 실제 서보처럼 목표를 향해 유한 속도로 움직인다 (최대 3000 ticks/s) */
static void servo_update(fake_servo_t *s) {
    uint64_t now = time_us_64();
    double dt = s->last_us ? (double)(now - s->last_us) / 1e6 : 0.0;
    s->last_us = now;
    if (s->reg[40]) {
        double goal = dec15((uint16_t)(s->reg[42] | s->reg[43] << 8));
        double d = goal - s->pos, step = 3000.0 * dt;
        double moved = d > step ? step : d < -step ? -step : d;
        s->pos += moved;
        s->vel = dt > 0 ? moved / dt : 0.0;
    } else {
        s->vel = 0.0;
    }
    int32_t p = (int32_t)(s->pos + 0.5);
    if (s->spike_left > 0)
        p += s->spike_ticks;
    uint16_t pr = p < 0 ? (uint16_t)((-p) | 0x8000) : (uint16_t)p;
    int32_t v = (int32_t)s->vel;
    uint16_t vr = v < 0 ? (uint16_t)((-v) | 0x8000) : (uint16_t)v;
    uint16_t lr = s->load < 0 ? (uint16_t)((-s->load) | 0x400) : (uint16_t)s->load;
    uint8_t *r = s->reg;
    r[56] = (uint8_t)pr, r[57] = (uint8_t)(pr >> 8);
    r[58] = (uint8_t)vr, r[59] = (uint8_t)(vr >> 8);
    r[60] = (uint8_t)lr, r[61] = (uint8_t)(lr >> 8);
    r[62] = s->volt, r[63] = s->temp;
}

static void servo_reply(uint8_t id, const uint8_t *data, uint8_t len) {
    uint8_t sum = (uint8_t)(id + len + 2);
    rx_push(0xFF, f_uart_flip_ppm);
    rx_push(0xFF, f_uart_flip_ppm);
    rx_push(id, f_uart_flip_ppm);
    rx_push((uint8_t)(len + 2), f_uart_flip_ppm);
    rx_push(0, f_uart_flip_ppm); /* 오류 없음 */
    for (uint8_t i = 0; i < len; i++) {
        rx_push(data[i], f_uart_flip_ppm);
        sum = (uint8_t)(sum + data[i]);
    }
    rx_push((uint8_t)~sum, f_uart_flip_ppm);
}

static void servo_write(uint8_t id, uint8_t addr, const uint8_t *d, uint8_t len) {
    fake_servo_t *s = &servo[id];
    if (!s->present || s->dead)
        return;
    if (s->ignore_writes > 0) {
        s->ignore_writes--;
        return;
    }
    servo_update(s);
    for (uint8_t i = 0; i < len && addr + i < 56; i++)
        s->reg[addr + i] = d[i];
}

static bool checksum_ok(const uint8_t *p, size_t n) {
    uint8_t sum = 0;
    for (size_t i = 2; i + 1 < n; i++)
        sum = (uint8_t)(sum + p[i]);
    return (uint8_t)~sum == p[n - 1];
}

static void servo_handle(const uint8_t *p, size_t n) {
    if (n < 6 || p[0] != 0xFF || p[1] != 0xFF || !checksum_ok(p, n))
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
    } else if (ins == 0x02 && id < 254 && servo[id].present && !servo[id].dead) {
        fake_servo_t *s = &servo[id];
        uint8_t addr = p[5], rl = p[6];
        if (addr + rl > 64)
            return;
        servo_update(s);
        servo_reply(id, &s->reg[addr], rl);
        if (addr <= 56 && addr + rl > 56 && s->spike_left > 0)
            s->spike_left--;
    }
}

static const fw_actuator_t hal_acts[FW_N_ACT] = FW_ACTUATORS_INIT;

uint uart_init(uart_inst_t *u, uint baud) {
    (void)u;
    for (int i = 0; i < FW_N_ACT; i++) {
        if (hal_acts[i].kind == FW_ACT_STS) {
            fake_servo_t *s = &servo[hal_acts[i].id];
            memset(s, 0, sizeof *s);
            s->present = true;
            s->pos = 2048;
            s->reg[42] = 2048 & 0xFF, s->reg[43] = 2048 >> 8;
            s->temp = 30, s->volt = 120;
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
    /* 서보가 받는 바이트에도 같은 잡음이 낀다 */
    uint8_t rx[512];
    size_t n = len < sizeof rx ? len : sizeof rx;
    memcpy(rx, src, n);
    for (size_t i = 0; i < n; i++)
        if (chance(f_uart_flip_ppm))
            rx[i] ^= (uint8_t)(1u << (rnd() % 8));
#if FW_STS_ECHO
    for (size_t i = 0; i < n; i++) /* 에코 = 선로에 실제로 실린 값 */
        rx_push(rx[i], f_echo_flip_ppm);
#endif
    servo_handle(rx, n);
}

/*
 * 고장 주입 명령 (한 줄):
 *   uart_flip <ppm>        서보 버스 바이트 비트 뒤집힘 (양방향)
 *   uart_drop <ppm>        서보 응답 바이트 유실
 *   uart_noise <ppm>       응답 사이 잡음 바이트 삽입
 *   echo_flip <ppm>        에코만 깨짐 (송신 확인 실패 -> 재전송)
 *   dead <id> <0|1>        서보 무응답
 *   reset <id>             서보 저전압 리셋: 토크 해제, 목표 = 현재 위치
 *   spike <id> <ticks> <n> 다음 n 개 읽기에 위치 +ticks (체크섬은 맞는 엉뚱한 값)
 *   ignore_writes <id> <n> 다음 n 개 쓰기 무시 (놓친 SYNC WRITE)
 *   temp <id> <C> / volt <id> <dV> / load <id> <permille>
 * 알 수 없으면 false.
 */
bool hal_fault(const char *line) {
    char cmd[32];
    long a = 0, b = 0, c = 0;
    int n = sscanf(line, "%31s %ld %ld %ld", cmd, &a, &b, &c);
    if (n < 2)
        return false;
    fake_servo_t *s = (a >= 0 && a < 254) ? &servo[a] : NULL;
    if (!strcmp(cmd, "uart_flip")) f_uart_flip_ppm = (uint32_t)a;
    else if (!strcmp(cmd, "uart_drop")) f_uart_drop_ppm = (uint32_t)a;
    else if (!strcmp(cmd, "uart_noise")) f_noise_ppm = (uint32_t)a;
    else if (!strcmp(cmd, "echo_flip")) f_echo_flip_ppm = (uint32_t)a;
    else if (!s) return false;
    else if (!strcmp(cmd, "dead") && n >= 3) s->dead = b != 0;
    else if (!strcmp(cmd, "reset")) {
        servo_update(s);
        int32_t p = (int32_t)s->pos;
        s->reg[40] = 0;
        s->reg[42] = (uint8_t)p, s->reg[43] = (uint8_t)(p >> 8);
    } else if (!strcmp(cmd, "spike") && n >= 4) {
        s->spike_ticks = (int32_t)b;
        s->spike_left = (int)c;
    } else if (!strcmp(cmd, "ignore_writes") && n >= 3) s->ignore_writes = (int)b;
    else if (!strcmp(cmd, "temp") && n >= 3) s->temp = (uint8_t)b;
    else if (!strcmp(cmd, "volt") && n >= 3) s->volt = (uint8_t)b;
    else if (!strcmp(cmd, "load") && n >= 3) s->load = (int16_t)b;
    else return false;
    return true;
}
