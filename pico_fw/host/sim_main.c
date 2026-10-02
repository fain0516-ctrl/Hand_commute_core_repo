/*
 * 호스트(리눅스) 시뮬레이터: 펌웨어의 proto.c + session.c 를 그대로 쓰고,
 * W5500 대신 POSIX TCP 소켓, 서보 대신 이상적으로 추종하는 가짜 액추에이터를 붙인다.
 * tests/test_pico_fw.py 가 이 바이너리에 Pi 쪽 comm_core.PicoLink 를 연결해 프로토콜을 검증한다.
 *
 *   ./fw_sim <port>     (0 이면 임의 포트, 실제 포트를 첫 줄에 "port N" 으로 출력)
 *
 * -DFW_SIM_REAL_ACTUATORS 로 빌드하면 가짜 액추에이터 대신 펌웨어의 actuators.c + sts_bus.c 를
 * host/hal_stub.c (SDK 대체) 위에서 그대로 돌린다. 이때 출력:
 *   tcp <ms> <hex>   Pi 로 보낸 TCP 프레임
 *   bus <ms> <hex>   서보 버스(UART)로 보낸 패킷
 *   pwm <ms> <gpio> <us>  PWM 레벨 변화
 *
 * 펌웨어 main.c 와 같은 규칙: 새 연결이 오면 이전 연결을 끊고, link_idle_timeout 동안 수신이 없으면 끊는다.
 *
 * 고장 주입: stdin 으로 한 줄씩 명령을 받는다 ("fault <명령>"). TCP 쪽 명령은 여기서, 서보 버스 쪽은
 * hal_stub.c 의 hal_fault() 가 처리한다 (FW_SIM_REAL_ACTUATORS 일 때만).
 *   fault tcp_rx_flip <ppm>   Pi -> Pico 바이트 비트 뒤집힘 (W5500<->Pico SPI 잡음에 해당)
 *   fault tcp_rx_drop <ppm>   Pi -> Pico 바이트 유실
 *   fault tcp_tx_flip <ppm>   Pico -> Pi 바이트 비트 뒤집힘
 *   fault stall <ms>          core0/core1 이 ms 동안 멈춤 (긴 인터럽트, 플래시 쓰기 등)
 * 처리 결과는 "fault ok <명령>" / "fault unknown <명령>" 으로 출력.
 */
#define _POSIX_C_SOURCE 200809L
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "diag.h"
#include "fw_config.h"
#include "session.h"
#ifdef FW_SIM_REAL_ACTUATORS
#include "actuators.h"
bool hal_fault(const char *line); /* host/hal_stub.c */
#endif

static const fw_actuator_t ACTS[FW_N_ACT] = FW_ACTUATORS_INIT;
static int32_t pos[FW_N_ACT];
static uint32_t enabled;
static int client = -1;

static uint32_t f_rx_flip, f_rx_drop, f_tx_flip;
static uint64_t f_rng = 0x2545F4914F6CDD1Dull;
static bool f_chance(uint32_t ppm) {
    if (!ppm)
        return false;
    f_rng ^= f_rng << 13;
    f_rng ^= f_rng >> 7;
    f_rng ^= f_rng << 17;
    return (uint32_t)(f_rng >> 16) % 1000000u < ppm;
}
static uint16_t corrupt(uint8_t *d, uint16_t n, uint32_t flip_ppm, uint32_t drop_ppm) {
    uint16_t o = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (f_chance(drop_ppm))
            continue;
        uint8_t b = d[i];
        if (f_chance(flip_ppm))
            b ^= (uint8_t)(1u << (f_rng % 8));
        d[o++] = b;
    }
    return o;
}

static uint64_t now_us64(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

static void io_send(void *ctx, const uint8_t *data, uint16_t len) {
    (void)ctx;
#ifdef FW_SIM_REAL_ACTUATORS
    printf("tcp %u ", (unsigned)(now_us64() / 1000));
    for (uint16_t i = 0; i < len; i++)
        printf("%02x", data[i]);
    printf("\n");
#endif
    if (client >= 0) {
        uint8_t tmp[PROTO_MAX_FRAME];
        memcpy(tmp, data, len);
        corrupt(tmp, len, f_tx_flip, 0);
        (void)!send(client, tmp, len, MSG_NOSIGNAL);
    }
}
static void io_apply(void *ctx, const int32_t *t, uint32_t mask) {
    (void)ctx;
#ifdef FW_SIM_REAL_ACTUATORS
    actuators_apply(t, mask);
#endif
    for (int i = 0; i < FW_N_ACT; i++)
        if (mask & (1u << i))
            pos[i] = t[i];
    enabled |= mask;
}
static void io_release(void *ctx, uint32_t mask) {
    (void)ctx;
#ifdef FW_SIM_REAL_ACTUATORS
    actuators_release(mask);
#endif
    enabled &= ~mask;
}
static bool io_read_state(void *ctx, proto_axis_state_t *out, uint8_t n) {
    (void)ctx;
#ifdef FW_SIM_REAL_ACTUATORS
    return actuators_read_state(out, n);
#endif
    for (uint8_t i = 0; i < n; i++) {
        memset(&out[i], 0, sizeof out[i]);
        if (i < FW_N_ACT) {
            out[i].position = pos[i];
            out[i].temperature_c10 = ACTS[i].kind == FW_ACT_STS ? 300 : 0;
            if (enabled & (1u << i))
                out[i].flags = AX_TORQUE_ON;
        }
    }
    return false;
}

int main(int argc, char **argv) {
    int port = argc > 1 ? atoi(argv[1]) : FW_NET_TCP_PORT;
    for (int i = 0; i < FW_N_ACT; i++)
        pos[i] = (ACTS[i].raw_min + ACTS[i].raw_max) / 2;

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a = {.sin_family = AF_INET, .sin_port = htons((uint16_t)port),
                            .sin_addr.s_addr = htonl(INADDR_LOOPBACK)};
    if (bind(srv, (struct sockaddr *)&a, sizeof a) || listen(srv, 4)) {
        perror("bind/listen");
        return 1;
    }
    socklen_t al = sizeof a;
    getsockname(srv, (struct sockaddr *)&a, &al);
    printf("port %d\n", ntohs(a.sin_port));
    fflush(stdout);

    static session_t sess;
    const session_cfg_t cfg = {ACTS, FW_N_ACT, FW_N_STS, FW_N_PWM, FW_VERSION,
                               CAP_STATE_EXT | CAP_DIAG | CAP_CMD_SEQ_ECHO,
                               FW_DEFAULT_CMD_TIMEOUT_MS, FW_MIN_CMD_TIMEOUT_MS, FW_MAX_CMD_TIMEOUT_MS,
                               FW_R_LINK_DIAG_PERIOD_MS, FW_R_LINK_DEGRADE_HOLD_MS, FW_R_LINK_CMD_JITTER_WARN_MS};
    const session_io_t io = {NULL, io_send, io_apply, io_release, io_read_state};
    session_init(&sess, &cfg, &io);
    uint32_t last_rx_ms = 0;
    g_diag[DIAG_RESET_CAUSE] = 0x10000u; /* POR */
    uint8_t buf[2048];
#ifdef FW_SIM_REAL_ACTUATORS
    actuators_setup();
    actuators_bus_start();
    uint64_t next_step = now_us64();
#endif

    bool stdin_open = true;
    char line[256];
    size_t line_len = 0;
    for (;;) {
        struct pollfd fds[3] = {{srv, POLLIN, 0}, {stdin_open ? 0 : -1, POLLIN, 0}, {client, POLLIN, 0}};
        poll(fds, client >= 0 ? 3 : 2, 1);
        if (fds[1].revents & POLLIN) { /* 고장 주입 명령 */
            char c;
            ssize_t r;
            while ((r = read(0, &c, 1)) == 1) {
                if (c != '\n') {
                    if (line_len < sizeof line - 1)
                        line[line_len++] = c;
                    continue;
                }
                line[line_len] = 0;
                line_len = 0;
                const char *cmd = strncmp(line, "fault ", 6) == 0 ? line + 6 : line;
                long v = 0;
                char name[32] = "";
                sscanf(cmd, "%31s %ld", name, &v);
                bool ok = true;
                if (!strcmp(name, "tcp_rx_flip")) f_rx_flip = (uint32_t)v;
                else if (!strcmp(name, "tcp_rx_drop")) f_rx_drop = (uint32_t)v;
                else if (!strcmp(name, "tcp_tx_flip")) f_tx_flip = (uint32_t)v;
                else if (!strcmp(name, "stall")) nanosleep(&(struct timespec){v / 1000, (v % 1000) * 1000000L}, NULL);
#ifdef FW_SIM_REAL_ACTUATORS
                else ok = hal_fault(cmd);
#else
                else ok = false;
#endif
                printf("fault %s %s\n", ok ? "ok" : "unknown", cmd);
                fflush(stdout);
                break; /* 한 번에 한 줄: 나머지는 다음 poll 에서 */
            }
            if (r <= 0)
                stdin_open = false; /* EOF: 더 이상 보지 않는다 */
        }
        fds[1] = fds[2];
        uint64_t t = now_us64();
        uint32_t now_ms = (uint32_t)(t / 1000), now_us = (uint32_t)t;
        g_diag[DIAG_UPTIME_MS] = now_ms;
        if (fds[0].revents & POLLIN) {
            int c = accept(srv, NULL, NULL);
            if (c >= 0) {
                setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
                if (client >= 0) { /* 새 연결 우선 */
                    session_on_disconnect(&sess);
                    close(client);
                }
                client = c;
                session_on_connect(&sess, now_ms);
                last_rx_ms = now_ms;
                fds[1].revents = 0;
            }
        }
        if (client >= 0 && (fds[1].revents & (POLLIN | POLLHUP | POLLERR))) {
            ssize_t n = recv(client, buf, sizeof buf, 0);
            if (n <= 0) {
                session_on_disconnect(&sess);
                close(client);
                client = -1;
            } else {
                last_rx_ms = now_ms;
                uint16_t m = corrupt(buf, (uint16_t)n, f_rx_flip, f_rx_drop);
                session_feed(&sess, buf, m, now_ms, now_us);
            }
        }
        if (client >= 0 && now_ms - last_rx_ms > FW_NET_LINK_IDLE_MS) {
            session_on_disconnect(&sess);
            close(client);
            client = -1;
        }
        session_tick(&sess, now_ms);
#ifdef FW_SIM_REAL_ACTUATORS
        if (t >= next_step) { /* core1 루프 대신 같은 주기로 한 단계씩 */
            actuators_step();
            next_step = t + FW_STS_LOOP_PERIOD_US;
            fflush(stdout);
        }
#endif
        /* 테스트가 내부 상태를 볼 수 있도록 바뀔 때만 출력 */
        static uint32_t last_en = 0xFFFFFFFFu;
        if (enabled != last_en) {
            last_en = enabled;
            printf("enabled %08x status %04x\n", enabled, sess.status);
            fflush(stdout);
        }
    }
}
