/*
 * W5500 드라이버(src/w5500.c) 고장 주입 시험. 실제 드라이버 코드를 SPI 레지스터 에뮬레이터에 붙여 돌린다.
 *   - 설정 쓰기 중 SPI 비트 오류 -> 쓰고 다시 읽어 검출, init 실패 -> 재시도
 *   - W5500 단독 리셋(전압 강하/ESD) -> 주기 건강 검사로 검출 -> 재초기화
 *   - 읽기 비트 오류가 섞인 정상 동작 -> 일시 오류는 흡수 (불필요한 재초기화 없음)
 *   - 16비트 레지스터 값이 계속 흔들림 -> rd16_stable 이 무한 루프에 빠지지 않음
 *   - 연결 중 Sn_SR 이 한 번 잘못 읽힘 -> 연결을 끊지 않음
 *   - 재초기화가 반복되면 SPI 클럭을 낮춤
 * 결과를 사람이 읽을 수 있게 출력하고, 기대와 다르면 종료 코드 1.
 */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "diag.h"
#include "fw_config.h"
#include "host_sdk.h"
#include "w5500.h"

/* ---------------------------------------------------------------- SDK 대체 */

struct spi_inst { int n; };
static struct spi_inst s0 = {0}, s1 = {1};
spi_inst_t *const spi0_inst = &s0, *const spi1_inst = &s1;

uint64_t time_us_64(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}
void sleep_us(uint64_t us) {
    struct timespec ts = {(time_t)(us / 1000000u), (long)(us % 1000000u) * 1000};
    nanosleep(&ts, NULL);
}
void gpio_init(uint pin) { (void)pin; }
void gpio_set_dir(uint pin, bool out) { (void)pin; (void)out; }
void gpio_pull_up(uint pin) { (void)pin; }
void gpio_set_function(uint pin, int fn) { (void)pin; (void)fn; }

/* ---------------------------------------------------------------- W5500 에뮬레이터 */

static uint8_t common[0x40];
static uint8_t sreg[8][0x30];
static uint32_t last_baud;
static uint64_t rng = 0x853C49E6748FEA9Bull;
static uint32_t read_flip_ppm, write_flip_ppm;
static bool rsr_unstable;   /* Sn_RX_RSR 이 읽을 때마다 다른 값 */
static int sr_glitch;       /* 다음 n 번 Sn_SR 읽기는 0x00(CLOSED) */

static uint32_t rnd(void) {
    rng ^= rng << 13;
    rng ^= rng >> 7;
    rng ^= rng << 17;
    return (uint32_t)(rng >> 16);
}
static bool chance(uint32_t ppm) { return ppm && rnd() % 1000000u < ppm; }

static void chip_reset(void) {
    memset(common, 0, sizeof common);
    memset(sreg, 0, sizeof sreg);
    common[0x39] = 0x04; /* VERSIONR */
}

/* 트랜잭션 상태 */
static bool cs_low;
static uint8_t hdr[3];
static int hdr_n;
static uint16_t addr;

static uint8_t *reg_ptr(uint8_t bsb, uint16_t a, uint8_t *scratch) {
    if (bsb == 0)
        return a < sizeof common ? &common[a] : scratch;
    if ((bsb & 3) == 1 && a < 0x30)
        return &sreg[bsb >> 2][a];
    return scratch; /* TX/RX 버퍼: 내용은 이 시험에서 중요하지 않음 */
}

static void sock_command(int s, uint8_t cmd) {
    uint8_t *r = sreg[s];
    switch (cmd) {
    case 0x01: r[3] = 0x13; break;               /* OPEN -> INIT */
    case 0x02: r[3] = 0x14; break;               /* LISTEN */
    case 0x08: case 0x10: r[3] = 0x00; break;    /* DISCON/CLOSE */
    case 0x20: r[2] |= 0x10; break;              /* SEND -> SEND_OK */
    default: break;
    }
    r[1] = 0; /* Sn_CR 은 명령 처리 후 0 */
}

void gpio_put(uint pin, bool v) {
    if (pin != FW_W5500_PIN_CS)
        return;
    if (!v) {
        cs_low = true;
        hdr_n = 0;
    } else {
        cs_low = false;
    }
}

uint spi_init(spi_inst_t *spi, uint baud) {
    (void)spi;
    last_baud = baud;
    return baud;
}
void spi_set_format(spi_inst_t *spi, uint bits, int cpol, int cpha, int order) {
    (void)spi, (void)bits, (void)cpol, (void)cpha, (void)order;
}

int spi_write_blocking(spi_inst_t *spi, const uint8_t *src, size_t len) {
    (void)spi;
    for (size_t i = 0; i < len; i++) {
        if (hdr_n < 3) {
            hdr[hdr_n++] = src[i];
            if (hdr_n == 3)
                addr = (uint16_t)(hdr[0] << 8 | hdr[1]);
            continue;
        }
        uint8_t bsb = hdr[2] >> 3, scratch;
        uint8_t v = src[i];
        if (chance(write_flip_ppm))
            v ^= (uint8_t)(1u << (rnd() % 8));
        if (bsb == 0 && addr == 0x0000 && (v & 0x80)) { /* MR.RST */
            chip_reset();
        } else if ((bsb & 3) == 1 && addr == 0x0001) {
            sock_command(bsb >> 2, v);
        } else if ((bsb & 3) == 1 && addr == 0x0002) {
            sreg[bsb >> 2][2] &= (uint8_t)~v; /* Sn_IR: 1 을 쓰면 지움 */
        } else {
            *reg_ptr(bsb, addr, &scratch) = v;
        }
        addr++;
    }
    return (int)len;
}

int spi_read_blocking(spi_inst_t *spi, uint8_t tx, uint8_t *dst, size_t len) {
    (void)spi, (void)tx;
    uint8_t bsb = hdr[2] >> 3, scratch = 0;
    for (size_t i = 0; i < len; i++, addr++) {
        uint8_t v;
        int s = bsb >> 2;
        if ((bsb & 3) == 1 && addr == 0x0003 && sr_glitch > 0) {
            sr_glitch--;
            v = 0x00;
        } else if ((bsb & 3) == 1 && (addr == 0x0020 || addr == 0x0021)) { /* TX_FSR = 버퍼 전체 */
            uint16_t fsr = (uint16_t)(sreg[s][0x1F] * 1024u);
            v = addr == 0x0020 ? (uint8_t)(fsr >> 8) : (uint8_t)fsr;
        } else if ((bsb & 3) == 1 && (addr == 0x0026 || addr == 0x0027)) { /* RX_RSR */
            uint16_t rsr = rsr_unstable ? (uint16_t)(rnd() & 0xFF) : 0;
            v = addr == 0x0026 ? (uint8_t)(rsr >> 8) : (uint8_t)rsr;
        } else if (bsb == 0 && addr == 0x002E) {
            v = 0x07; /* PHYCFGR: 링크 업 */
        } else {
            v = *reg_ptr(bsb, addr, &scratch);
        }
        if (chance(read_flip_ppm))
            v ^= (uint8_t)(1u << (rnd() % 8));
        dst[i] = v;
    }
    return (int)len;
}

/* ---------------------------------------------------------------- 시험 */

static int failures;
#define EXPECT(cond, ...)                         \
    do {                                          \
        bool ok_ = (cond);                        \
        printf("[%s] ", ok_ ? "ok" : "FAIL");     \
        printf(__VA_ARGS__);                      \
        printf("\n");                             \
        if (!ok_)                                 \
            failures++;                           \
    } while (0)

/* main.c 와 같은 처리: 건강 검사 실패 -> 재초기화 (실패하면 다시) */
static int reinit_until_ok(void) {
    int tries = 0;
    w5500_note_reinit();
    while (!w5500_init() && tries < 20)
        tries++;
    return tries;
}

int main(void) {
    chip_reset();

    /* 1) 정상 */
    EXPECT(w5500_init(), "정상 초기화 (SPI %lu Hz)", (unsigned long)last_baud);
    int bad = 0;
    for (int i = 0; i < 200; i++)
        bad += !w5500_health_check();
    EXPECT(bad == 0 && g_diag[DIAG_SPI_ERRORS] == 0, "정상 상태 건강 검사 200 회: 실패 %d, SPI 오류 %lu", bad,
           (unsigned long)g_diag[DIAG_SPI_ERRORS]);

    /* 2) 설정 쓰기 중 비트 오류 */
    write_flip_ppm = 20000;
    int init_fail = 0, init_ok = 0;
    for (int i = 0; i < 50; i++)
        *(w5500_init() ? &init_ok : &init_fail) += 1;
    write_flip_ppm = 0;
    EXPECT(init_fail > 0, "설정 쓰기 비트 오류 20000 ppm 에서 init 50 회: 검출해서 실패 %d, 성공 %d (성공 시 값 확인됨)",
           init_fail, init_ok);
    EXPECT(w5500_init(), "오류가 사라진 뒤 init 성공");

    /* 3) W5500 단독 리셋 (설정이 0 으로 돌아감) */
    uint32_t reinit0 = g_diag[DIAG_W5500_REINITS];
    chip_reset();
    int checks = 0;
    while (w5500_health_check() && checks < 10)
        checks++;
    checks++;
    EXPECT(checks == FW_R_W5500_FAIL_THRESHOLD, "W5500 리셋 후 건강 검사 %d 번째에 재초기화 요청 (fail_threshold %d)",
           checks, FW_R_W5500_FAIL_THRESHOLD);
    int tries = reinit_until_ok();
    EXPECT(tries == 0 && w5500_health_check() && g_diag[DIAG_W5500_REINITS] == reinit0 + 1,
           "재초기화 후 정상 (재초기화 %lu 회)", (unsigned long)(g_diag[DIAG_W5500_REINITS] - reinit0));

    /* 4) 읽기 비트 오류가 섞인 정상 동작: 일시 오류는 흡수 */
    read_flip_ppm = 1000;
    uint32_t err0 = g_diag[DIAG_SPI_ERRORS];
    int false_reinit = 0;
    for (int i = 0; i < 2000; i++) {
        if (!w5500_health_check()) {
            false_reinit++;
            read_flip_ppm = 0;
            reinit_until_ok();
            read_flip_ppm = 1000;
        }
    }
    read_flip_ppm = 0;
    EXPECT(false_reinit <= 2, "읽기 비트 오류 1000 ppm 에서 건강 검사 2000 회 (=100 ms 주기로 200 s): SPI 오류 %lu, "
           "재초기화 %d 회", (unsigned long)(g_diag[DIAG_SPI_ERRORS] - err0), false_reinit);

    /* 5) Sn_RX_RSR 값이 계속 흔들림: 무한 루프 없음 */
    w5500_init();
    sreg[0][3] = 0x17; /* ESTABLISHED */
    uint8_t buf[256];
    uint16_t n = 0;
    net_event_t ev = w5500_poll(buf, sizeof buf, &n);
    EXPECT(ev == NET_EV_CONNECTED, "소켓 0 연결 이벤트");
    rsr_unstable = true;
    err0 = g_diag[DIAG_SPI_ERRORS];
    uint64_t t0 = time_us_64();
    for (int i = 0; i < 100; i++)
        w5500_poll(buf, sizeof buf, &n);
    uint64_t took = time_us_64() - t0;
    rsr_unstable = false;
    EXPECT(took < 1000000 && g_diag[DIAG_SPI_ERRORS] > err0,
           "RX_RSR 값이 계속 바뀌어도 폴링 100 회가 %llu us 에 끝남 (SPI 오류 %lu 로 기록)", (unsigned long long)took,
           (unsigned long)(g_diag[DIAG_SPI_ERRORS] - err0));

    /* 6) 연결 중 Sn_SR 한 번 잘못 읽힘 */
    sr_glitch = 1;
    ev = w5500_poll(buf, sizeof buf, &n);
    EXPECT(ev != NET_EV_DISCONNECTED && w5500_has_client(), "Sn_SR 이 한 번 CLOSED 로 잘못 읽혀도 연결 유지");
    sreg[0][3] = 0x00;
    ev = w5500_poll(buf, sizeof buf, &n);
    EXPECT(ev == NET_EV_DISCONNECTED, "실제로 닫히면 끊김 이벤트");

    /* 7) 재초기화 반복 -> SPI 클럭 낮춤 */
    for (int i = 0; i < FW_R_W5500_FALLBACK_AFTER; i++)
        reinit_until_ok();
    EXPECT(w5500_spi_baud() == FW_R_W5500_FALLBACK_BAUD_HZ && last_baud == FW_R_W5500_FALLBACK_BAUD_HZ,
           "재초기화 %d 회 이상이면 SPI 클럭 %lu -> %lu Hz", FW_R_W5500_FALLBACK_AFTER,
           (unsigned long)FW_W5500_BAUD_HZ, (unsigned long)last_baud);

    printf("%s\n", failures ? "W5500 FAIL" : "W5500 OK");
    return failures ? 1 : 0;
}
