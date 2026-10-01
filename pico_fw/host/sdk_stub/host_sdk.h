/* 호스트 시뮬레이터용 Pico SDK 최소 대체 헤더 (actuators.c / sts_bus.c 를 리눅스에서 컴파일하기 위함).
 * 구현은 host/hal_stub.c: UART 송신 바이트와 PWM 레벨을 기록하고, 가짜 STS3215 서보가 READ 에 응답한다. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef unsigned int uint;
typedef uint64_t absolute_time_t;
typedef volatile uint32_t spin_lock_t;
typedef struct uart_inst uart_inst_t;

extern uart_inst_t *const uart0_inst, *const uart1_inst;
#define uart0 uart0_inst
#define uart1 uart1_inst

enum { GPIO_FUNC_SPI = 1, GPIO_FUNC_UART = 2, GPIO_FUNC_PWM = 4 };
enum { GPIO_IN = 0, GPIO_OUT = 1 };
enum { UART_PARITY_NONE = 0 };
enum { clk_sys = 5 };

uint64_t time_us_64(void);
static inline absolute_time_t get_absolute_time(void) { return time_us_64(); }
static inline absolute_time_t delayed_by_us(absolute_time_t t, uint64_t us) { return t + us; }
static inline absolute_time_t make_timeout_time_us(uint64_t us) { return time_us_64() + us; }
static inline bool time_reached(absolute_time_t t) { return time_us_64() >= t; }
void sleep_us(uint64_t us);
static inline void sleep_until(absolute_time_t t) { uint64_t n = time_us_64(); if (t > n) sleep_us(t - n); }

void gpio_init(uint pin);
void gpio_set_dir(uint pin, bool out);
void gpio_put(uint pin, bool v);
void gpio_pull_up(uint pin);
void gpio_set_function(uint pin, int fn);

uint32_t clock_get_hz(int clk);
uint pwm_gpio_to_slice_num(uint pin);
void pwm_set_clkdiv(uint slice, float div);
void pwm_set_wrap(uint slice, uint16_t wrap);
void pwm_set_gpio_level(uint pin, uint16_t level);
void pwm_set_enabled(uint slice, bool en);

uint uart_init(uart_inst_t *u, uint baud);
void uart_set_format(uart_inst_t *u, uint bits, uint stop, int parity);
void uart_set_fifo_enabled(uart_inst_t *u, bool en);
bool uart_is_readable(uart_inst_t *u);
char uart_getc(uart_inst_t *u);
void uart_write_blocking(uart_inst_t *u, const uint8_t *src, size_t len);
void uart_tx_wait_blocking(uart_inst_t *u);

static inline int spin_lock_claim_unused(bool required) { (void)required; return 0; }
static inline spin_lock_t *spin_lock_init(int n) { static spin_lock_t l; (void)n; return &l; }
static inline uint32_t spin_lock_blocking(spin_lock_t *l) { (void)l; return 0; }
static inline void spin_unlock(spin_lock_t *l, uint32_t s) { (void)l; (void)s; }
static inline void multicore_launch_core1(void (*f)(void)) { (void)f; }
