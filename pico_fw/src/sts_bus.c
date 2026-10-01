#include "sts_bus.h"

#include "fw_config.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#define UART_INST (FW_STS_UART ? uart1 : uart0)

#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_SYNC_WRITE 0x83

static void drain_rx(void) {
    while (uart_is_readable(UART_INST))
        (void)uart_getc(UART_INST);
}

static bool getc_timeout(uint8_t *c, absolute_time_t until) {
    while (!uart_is_readable(UART_INST)) {
        if (time_reached(until))
            return false;
    }
    *c = (uint8_t)uart_getc(UART_INST);
    return true;
}

static void set_dir_tx(bool tx) {
#if FW_STS_PIN_DIR >= 0
    gpio_put(FW_STS_PIN_DIR, tx ? FW_STS_DIR_TX_LEVEL : !FW_STS_DIR_TX_LEVEL);
#else
    (void)tx;
#endif
}

/* 패킷 송신. TX/RX 를 묶은 회로면 자기 송신이 RX 로 돌아오므로 그만큼 읽어 버린다. */
static void send_packet(const uint8_t *pkt, uint16_t len) {
    drain_rx();
    set_dir_tx(true);
    uart_write_blocking(UART_INST, pkt, len);
    uart_tx_wait_blocking(UART_INST);
    set_dir_tx(false);
#if FW_STS_ECHO
    absolute_time_t until = make_timeout_time_us(200 + (uint64_t)len * 20000000ull / FW_STS_BAUD);
    uint8_t c;
    for (uint16_t i = 0; i < len && getc_timeout(&c, until); i++) {
    }
#endif
#if FW_STS_RETURN_DELAY_US > 0
    sleep_us(FW_STS_RETURN_DELAY_US);
#endif
}

static uint8_t checksum(const uint8_t *p, uint16_t from, uint16_t to) {
    uint8_t s = 0;
    for (uint16_t i = from; i < to; i++)
        s = (uint8_t)(s + p[i]);
    return (uint8_t)~s;
}

void sts_bus_init(void) {
    uart_init(UART_INST, FW_STS_BAUD);
    uart_set_format(UART_INST, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_INST, true);
    gpio_set_function(FW_STS_PIN_TX, GPIO_FUNC_UART);
    gpio_set_function(FW_STS_PIN_RX, GPIO_FUNC_UART);
    gpio_pull_up(FW_STS_PIN_RX); /* 버스 유휴 = High */
#if FW_STS_PIN_DIR >= 0
    gpio_init(FW_STS_PIN_DIR);
    gpio_set_dir(FW_STS_PIN_DIR, GPIO_OUT);
    set_dir_tx(false);
#endif
#if FW_STS_PIN_OE >= 0
    gpio_init(FW_STS_PIN_OE);
    gpio_set_dir(FW_STS_PIN_OE, GPIO_OUT);
    gpio_put(FW_STS_PIN_OE, FW_STS_OE_ACTIVE_LEVEL);
#endif
}

void sts_write(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t len) {
    uint8_t pkt[8 + 32];
    if (len > 32)
        return;
    uint16_t n = 0;
    pkt[n++] = 0xFF;
    pkt[n++] = 0xFF;
    pkt[n++] = id;
    pkt[n++] = (uint8_t)(len + 3);
    pkt[n++] = INST_WRITE;
    pkt[n++] = addr;
    for (uint8_t i = 0; i < len; i++)
        pkt[n++] = data[i];
    pkt[n] = checksum(pkt, 2, n);
    send_packet(pkt, (uint16_t)(n + 1));
}

void sts_sync_write(uint8_t addr, uint8_t len, const uint8_t *ids, const uint8_t *data, uint8_t count) {
    uint8_t pkt[8 + 32 * 9];
    if (count == 0 || count > 32 || len > 8)
        return;
    uint16_t n = 0;
    pkt[n++] = 0xFF;
    pkt[n++] = 0xFF;
    pkt[n++] = STS_BROADCAST;
    pkt[n++] = (uint8_t)((len + 1) * count + 4);
    pkt[n++] = INST_SYNC_WRITE;
    pkt[n++] = addr;
    pkt[n++] = len;
    for (uint8_t i = 0; i < count; i++) {
        pkt[n++] = ids[i];
        for (uint8_t j = 0; j < len; j++)
            pkt[n++] = data[i * len + j];
    }
    pkt[n] = checksum(pkt, 2, n);
    send_packet(pkt, (uint16_t)(n + 1));
}

sts_result_t sts_read(uint8_t id, uint8_t addr, uint8_t len, uint8_t *out, uint8_t *servo_error) {
    uint8_t pkt[8] = {0xFF, 0xFF, id, 4, INST_READ, addr, len, 0};
    pkt[7] = checksum(pkt, 2, 7);
    send_packet(pkt, 8);

    /* 응답: FF FF ID LEN ERR DATA... CHK */
    absolute_time_t until = make_timeout_time_us(FW_STS_RESPONSE_TIMEOUT_US);
    uint8_t c, prev = 0;
    for (;;) { /* 헤더 동기화 */
        if (!getc_timeout(&c, until))
            return STS_TIMEOUT;
        if (prev == 0xFF && c == 0xFF)
            break;
        prev = c;
    }
    uint8_t hdr[3];
    for (int i = 0; i < 3; i++) {
        if (!getc_timeout(&hdr[i], until))
            return STS_TIMEOUT;
    }
    if (hdr[0] != id || hdr[1] != len + 2)
        return STS_BAD_PACKET;
    uint8_t sum = (uint8_t)(hdr[0] + hdr[1] + hdr[2]);
    for (uint8_t i = 0; i < len; i++) {
        if (!getc_timeout(&out[i], until))
            return STS_TIMEOUT;
        sum = (uint8_t)(sum + out[i]);
    }
    if (!getc_timeout(&c, until))
        return STS_TIMEOUT;
    if (c != (uint8_t)~sum)
        return STS_BAD_PACKET;
    *servo_error = hdr[2];
    return STS_OK;
}
