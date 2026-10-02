#include "proto.h"

#include <string.h>

uint16_t proto_crc16(const uint8_t *data, size_t len, uint16_t crc) {
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t proto_encode(uint8_t *out, uint8_t msg_type, uint8_t flags, uint16_t seq, uint32_t timestamp_us,
                    const uint8_t *payload, uint16_t payload_len) {
    out[0] = 0xAA;
    out[1] = 0x55;
    out[2] = PROTO_VERSION;
    out[3] = msg_type;
    out[4] = flags;
    out[5] = 0;
    put_u16(out + 6, seq);
    put_u32(out + 8, timestamp_us);
    put_u16(out + 12, payload_len);
    if (payload_len && payload != out + PROTO_HEADER_SIZE)
        memmove(out + PROTO_HEADER_SIZE, payload, payload_len);
    size_t body = PROTO_HEADER_SIZE + payload_len;
    put_u16(out + body, proto_crc16(out, body, 0xFFFF));
    return body + PROTO_CRC_SIZE;
}

void proto_decoder_reset(proto_decoder_t *d) {
    d->len = 0;
}

static void drop_front(proto_decoder_t *d, size_t n) {
    memmove(d->buf, d->buf + n, d->len - n);
    d->len -= n;
}

/* 버퍼에 쌓인 바이트에서 가능한 프레임을 모두 꺼낸다. */
static void parse(proto_decoder_t *d, proto_frame_cb cb, void *ctx) {
    for (;;) {
        size_t idx = 0;
        while (idx + 1 < d->len && !(d->buf[idx] == 0xAA && d->buf[idx + 1] == 0x55))
            idx++;
        if (idx + 1 >= d->len) {
            /* magic 없음. 마지막 바이트가 0xAA 면 magic 앞부분일 수 있으니 남긴다 */
            size_t keep = (d->len && d->buf[d->len - 1] == 0xAA) ? 1 : 0;
            d->dropped_bytes += (uint32_t)(d->len - keep);
            drop_front(d, d->len - keep);
            return;
        }
        if (idx) {
            d->dropped_bytes += (uint32_t)idx;
            drop_front(d, idx);
        }
        if (d->len < PROTO_HEADER_SIZE)
            return;
        uint16_t plen = get_u16(d->buf + 12);
        if (d->buf[2] != PROTO_VERSION || plen > PROTO_MAX_PAYLOAD) {
            d->dropped_bytes++;
            drop_front(d, 1);
            continue;
        }
        size_t total = PROTO_HEADER_SIZE + plen + PROTO_CRC_SIZE;
        if (d->len < total)
            return;
        if (get_u16(d->buf + PROTO_HEADER_SIZE + plen) != proto_crc16(d->buf, PROTO_HEADER_SIZE + plen, 0xFFFF)) {
            d->crc_errors++;
            d->dropped_bytes++;
            drop_front(d, 1);
            continue;
        }
        proto_frame_t f = {
            .version = d->buf[2],
            .msg_type = d->buf[3],
            .flags = d->buf[4],
            .seq = get_u16(d->buf + 6),
            .timestamp_us = get_u32(d->buf + 8),
            .payload_len = plen,
            .payload = d->buf + PROTO_HEADER_SIZE,
        };
        cb(ctx, &f);
        drop_front(d, total);
    }
}

void proto_decoder_feed(proto_decoder_t *d, const uint8_t *data, size_t len, proto_frame_cb cb, void *ctx) {
    while (len) {
        size_t room = sizeof(d->buf) - d->len;
        size_t n = len < room ? len : room;
        memcpy(d->buf + d->len, data, n);
        d->len += n;
        data += n;
        len -= n;
        parse(d, cb, ctx);
        /* parse 후에도 가득 찼다면 (최대 프레임 2개 분량인데 완성 프레임이 없음) 앞을 버려 진행을 보장 */
        if (d->len == sizeof(d->buf)) {
            d->dropped_bytes++;
            drop_front(d, 1);
        }
    }
}
