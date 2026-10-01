#include "w5500.h"

#include <string.h>

#include "fw_config.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

/* ---------------------------------------------------------------- 레지스터 (W5500 데이터시트) */

#define BSB_COMMON 0x00
#define BSB_SREG(n) ((uint8_t)((n) * 4 + 1))
#define BSB_STX(n) ((uint8_t)((n) * 4 + 2))
#define BSB_SRX(n) ((uint8_t)((n) * 4 + 3))

#define MR 0x0000
#define GAR 0x0001
#define SUBR 0x0005
#define SHAR 0x0009
#define SIPR 0x000F
#define RTR 0x0019
#define RCR 0x001B
#define PHYCFGR 0x002E
#define VERSIONR 0x0039

#define Sn_MR 0x0000
#define Sn_CR 0x0001
#define Sn_IR 0x0002
#define Sn_SR 0x0003
#define Sn_PORT 0x0004
#define Sn_RXBUF_SIZE 0x001E
#define Sn_TXBUF_SIZE 0x001F
#define Sn_TX_FSR 0x0020
#define Sn_TX_WR 0x0024
#define Sn_RX_RSR 0x0026
#define Sn_RX_RD 0x0028
#define Sn_KPALVTR 0x002F

#define MR_TCP 0x01
#define MR_ND 0x20 /* No Delayed ACK: 지연 최소화 */

#define CR_OPEN 0x01
#define CR_LISTEN 0x02
#define CR_DISCON 0x08
#define CR_CLOSE 0x10
#define CR_SEND 0x20
#define CR_RECV 0x40

#define IR_SEND_OK 0x10

#define SR_CLOSED 0x00
#define SR_INIT 0x13
#define SR_LISTEN 0x14
#define SR_ESTABLISHED 0x17
#define SR_CLOSE_WAIT 0x1C

#define W5500_VERSION 0x04

#define SPI_INST (FW_W5500_SPI ? spi1 : spi0)

/* ---------------------------------------------------------------- SPI 접근 */

static inline void cs(bool active) {
    gpio_put(FW_W5500_PIN_CS, active ? 0 : 1);
}

static void xfer_write(uint16_t addr, uint8_t bsb, const uint8_t *data, uint16_t len) {
    uint8_t hdr[3] = {(uint8_t)(addr >> 8), (uint8_t)addr, (uint8_t)((bsb << 3) | 0x04)};
    cs(true);
    spi_write_blocking(SPI_INST, hdr, 3);
    spi_write_blocking(SPI_INST, data, len);
    cs(false);
}

static void xfer_read(uint16_t addr, uint8_t bsb, uint8_t *data, uint16_t len) {
    uint8_t hdr[3] = {(uint8_t)(addr >> 8), (uint8_t)addr, (uint8_t)(bsb << 3)};
    cs(true);
    spi_write_blocking(SPI_INST, hdr, 3);
    spi_read_blocking(SPI_INST, 0, data, len);
    cs(false);
}

static void wr8(uint16_t a, uint8_t bsb, uint8_t v) { xfer_write(a, bsb, &v, 1); }
static uint8_t rd8(uint16_t a, uint8_t bsb) {
    uint8_t v;
    xfer_read(a, bsb, &v, 1);
    return v;
}
static void wr16(uint16_t a, uint8_t bsb, uint16_t v) {
    uint8_t b[2] = {(uint8_t)(v >> 8), (uint8_t)v};
    xfer_write(a, bsb, b, 2);
}
static uint16_t rd16(uint16_t a, uint8_t bsb) {
    uint8_t b[2];
    xfer_read(a, bsb, b, 2);
    return (uint16_t)((b[0] << 8) | b[1]);
}
/* 칩이 갱신 중인 16비트 레지스터는 두 번 같은 값이 나올 때까지 읽는다 (데이터시트 권고) */
static uint16_t rd16_stable(uint16_t a, uint8_t bsb) {
    uint16_t v1, v2 = rd16(a, bsb);
    do {
        v1 = v2;
        v2 = rd16(a, bsb);
    } while (v1 != v2);
    return v1;
}

static void sock_cmd(uint8_t s, uint8_t cmd) {
    wr8(Sn_CR, BSB_SREG(s), cmd);
    absolute_time_t until = make_timeout_time_ms(5);
    while (rd8(Sn_CR, BSB_SREG(s)) && !time_reached(until))
        tight_loop_contents();
}

/* ---------------------------------------------------------------- 소켓 관리 */

static int8_t active = -1;           /* 현재 Pi 와 연결된 소켓 */
static bool was_established[FW_NET_LISTEN_SOCKETS];
static bool send_pending[FW_NET_LISTEN_SOCKETS]; /* 이전 SEND 의 SEND_OK 를 아직 확인하지 않음 */

static void sock_listen(uint8_t s) {
    sock_cmd(s, CR_CLOSE);
    wr8(Sn_MR, BSB_SREG(s), MR_TCP | MR_ND);
    wr16(Sn_PORT, BSB_SREG(s), FW_NET_TCP_PORT);
    wr8(Sn_KPALVTR, BSB_SREG(s), FW_NET_KEEPALIVE_5S);
    sock_cmd(s, CR_OPEN);
    if (rd8(Sn_SR, BSB_SREG(s)) == SR_INIT)
        sock_cmd(s, CR_LISTEN);
    was_established[s] = false;
    send_pending[s] = false;
}

static void sock_close_and_relisten(uint8_t s) {
    sock_cmd(s, CR_DISCON);
    sock_listen(s);
}

bool w5500_init(void) {
    spi_init(SPI_INST, FW_W5500_BAUD_HZ);
    spi_set_format(SPI_INST, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(FW_W5500_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(FW_W5500_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(FW_W5500_PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(FW_W5500_PIN_CS);
    gpio_set_dir(FW_W5500_PIN_CS, GPIO_OUT);
    cs(false);
#if FW_W5500_PIN_INT >= 0
    gpio_init(FW_W5500_PIN_INT);
    gpio_set_dir(FW_W5500_PIN_INT, GPIO_IN);
    gpio_pull_up(FW_W5500_PIN_INT);
#endif
#if FW_W5500_PIN_RST >= 0
    gpio_init(FW_W5500_PIN_RST);
    gpio_set_dir(FW_W5500_PIN_RST, GPIO_OUT);
    gpio_put(FW_W5500_PIN_RST, 0);
    sleep_ms(1); /* RSTn Low >= 500 us */
    gpio_put(FW_W5500_PIN_RST, 1);
    sleep_ms(2); /* PLL 안정화 */
#endif
    wr8(MR, BSB_COMMON, 0x80); /* 소프트 리셋 */
    sleep_ms(1);
    if (rd8(VERSIONR, BSB_COMMON) != W5500_VERSION)
        return false;

    static const uint8_t mac[6] = FW_NET_MAC, ip[4] = FW_NET_IP, mask[4] = FW_NET_NETMASK, gw[4] = FW_NET_GATEWAY;
    xfer_write(SHAR, BSB_COMMON, mac, 6);
    xfer_write(SIPR, BSB_COMMON, ip, 4);
    xfer_write(SUBR, BSB_COMMON, mask, 4);
    xfer_write(GAR, BSB_COMMON, gw, 4);
    wr16(RTR, BSB_COMMON, FW_NET_RETRY_TIME_100US);
    wr8(RCR, BSB_COMMON, FW_NET_RETRY_COUNT);

    for (uint8_t s = 0; s < 8; s++) {
        uint8_t kb = s < FW_NET_LISTEN_SOCKETS ? FW_NET_SOCKET_BUF_KB : 0;
        wr8(Sn_RXBUF_SIZE, BSB_SREG(s), kb);
        wr8(Sn_TXBUF_SIZE, BSB_SREG(s), kb);
    }
    for (uint8_t s = 0; s < FW_NET_LISTEN_SOCKETS; s++)
        sock_listen(s);
    active = -1;
    return true;
}

bool w5500_link_up(void) {
    return rd8(PHYCFGR, BSB_COMMON) & 0x01;
}

bool w5500_has_client(void) {
    return active >= 0;
}

void w5500_drop_active(void) {
    if (active >= 0) {
        sock_close_and_relisten((uint8_t)active);
        active = -1;
    }
}

static uint16_t sock_recv(uint8_t s, uint8_t *buf, uint16_t cap) {
    uint16_t avail = rd16_stable(Sn_RX_RSR, BSB_SREG(s));
    if (!avail)
        return 0;
    uint16_t n = avail < cap ? avail : cap;
    uint16_t rd = rd16(Sn_RX_RD, BSB_SREG(s));
    xfer_read(rd, BSB_SRX(s), buf, n); /* 버퍼 주소는 칩이 자동으로 순환시킨다 */
    wr16(Sn_RX_RD, BSB_SREG(s), (uint16_t)(rd + n));
    sock_cmd(s, CR_RECV);
    return n;
}

net_event_t w5500_poll(uint8_t *buf, uint16_t cap, uint16_t *len) {
    *len = 0;
    net_event_t ev = NET_EV_NONE;
    for (uint8_t s = 0; s < FW_NET_LISTEN_SOCKETS; s++) {
        uint8_t sr = rd8(Sn_SR, BSB_SREG(s));
        if (sr == SR_ESTABLISHED && !was_established[s]) {
            was_established[s] = true;
            if (active >= 0 && active != s) {
                /* 새 연결 우선: 이전 연결(좀비일 수 있음) 정리 */
                sock_close_and_relisten((uint8_t)active);
                ev = NET_EV_DISCONNECTED;
            }
            active = (int8_t)s;
            /* 끊김과 연결이 한 번에 생기면 연결을 알린다 (세션은 연결 시 초기화됨) */
            ev = NET_EV_CONNECTED;
        } else if (sr == SR_CLOSE_WAIT || sr == SR_CLOSED) {
            if (sr == SR_CLOSE_WAIT)
                sock_cmd(s, CR_DISCON);
            sock_listen(s);
            if (active == s) {
                active = -1;
                ev = NET_EV_DISCONNECTED;
            }
        }
    }
    if (active >= 0 && ev != NET_EV_DISCONNECTED)
        *len = sock_recv((uint8_t)active, buf, cap);
    return ev;
}

bool w5500_send(const uint8_t *data, uint16_t len) {
    if (active < 0)
        return false;
    uint8_t s = (uint8_t)active;
    if (send_pending[s]) {
        /* 데이터시트: 다음 SEND 전에 이전 SEND 완료(SEND_OK)를 확인한다. 100 Mbps 에서 수 us 이내 */
        absolute_time_t until = make_timeout_time_us(500);
        while (!(rd8(Sn_IR, BSB_SREG(s)) & IR_SEND_OK)) {
            if (time_reached(until))
                return false; /* 상대가 받지 않음: 이번 프레임은 버린다 */
        }
        wr8(Sn_IR, BSB_SREG(s), IR_SEND_OK);
        send_pending[s] = false;
    }
    if (rd16_stable(Sn_TX_FSR, BSB_SREG(s)) < len)
        return false;
    uint16_t wr = rd16(Sn_TX_WR, BSB_SREG(s));
    xfer_write(wr, BSB_STX(s), data, len);
    wr16(Sn_TX_WR, BSB_SREG(s), (uint16_t)(wr + len));
    sock_cmd(s, CR_SEND);
    send_pending[s] = true;
    return true;
}
