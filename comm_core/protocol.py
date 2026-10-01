"""Pi 5 <-> Pico 2 (W5500) TCP 바이너리 프레임 프로토콜.

요구사항이 아직 확정되지 않았으므로 여유를 두고 설계했다.
- 버전 필드: 프레임 형식이 바뀌어도 구버전 펌웨어를 구분할 수 있다.
- 가변 길이 payload (최대 MAX_PAYLOAD): 축 수가 늘어나도 프레임 형식은 그대로다.
- 모르는 msg_type 은 수신 측에서 무시한다 (나중에 메시지를 추가해도 호환).
- TCP 스트림 위에서 magic + CRC16 으로 프레임 경계를 다시 맞춘다.

프레임 구조 (Little-endian):

    off size 내용
    0   2    magic       0xAA 0x55
    2   1    version     PROTOCOL_VERSION
    3   1    msg_type    MsgType
    4   1    flags       메시지별 플래그
    5   1    reserved    0
    6   2    seq         0~65535 순환
    8   4    timestamp   송신 측 단조 시계 (us, 32bit 순환)
    12  2    payload_len 0~MAX_PAYLOAD
    14  N    payload
    14+N 2   crc16       CRC-16/CCITT-FALSE (header + payload)
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional, Sequence

MAGIC = b"\xAA\x55"
PROTOCOL_VERSION = 1
HEADER_FMT = "<2sBBBBHIH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 14
CRC_SIZE = 2
MAX_PAYLOAD = 1024
MAX_ACTUATORS = 32


class MsgType(IntEnum):
    HELLO = 0x01          # Pi -> Pico: 연결 직후 설정 전달
    HELLO_ACK = 0x02      # Pico -> Pi: 펌웨어 정보/축 수
    HEARTBEAT = 0x03      # 양방향: 링크 생존 확인
    ACTUATOR_CMD = 0x10   # Pi -> Pico: 액추에이터 지령
    ESTOP = 0x11          # Pi -> Pico: 전 축 토크 해제
    STATE = 0x20          # Pico -> Pi: 액추에이터 상태 피드백
    ERROR = 0x7F          # 양방향: 오류 보고


class CmdMode(IntEnum):
    POSITION = 0  # STS3215: 0~4095 tick, PWM: 펄스 폭 us
    TORQUE = 1    # mNm (0 이면 해당 축 토크 해제로 해석)
    VELOCITY = 2  # 예약: 현재 Pico 펌웨어 미지원


class CmdFlag(IntEnum):
    WATCHDOG_TRIPPED = 0x01  # 상위 지령 끊김으로 생성된 안전 지령


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF). Pico 쪽도 같은 함수를 쓴다."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass
class Frame:
    msg_type: int
    payload: bytes = b""
    seq: int = 0
    timestamp_us: int = 0
    flags: int = 0
    version: int = PROTOCOL_VERSION


def encode_frame(frame: Frame) -> bytes:
    if len(frame.payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too large: {len(frame.payload)} > {MAX_PAYLOAD}")
    header = struct.pack(
        HEADER_FMT,
        MAGIC,
        frame.version,
        int(frame.msg_type),
        frame.flags & 0xFF,
        0,
        frame.seq & 0xFFFF,
        frame.timestamp_us & 0xFFFFFFFF,
        len(frame.payload),
    )
    body = header + frame.payload
    return body + struct.pack("<H", crc16_ccitt(body))


class FrameDecoder:
    """TCP 바이트 스트림을 프레임으로 자른다. 깨진 데이터는 버리고 다음 magic 에서 재동기화."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.crc_errors = 0
        self.dropped_bytes = 0

    def feed(self, data: bytes) -> List[Frame]:
        self._buf += data
        frames: List[Frame] = []
        while True:
            idx = self._buf.find(MAGIC)
            if idx < 0:
                # 마지막 바이트가 magic 의 앞부분일 수 있으니 남겨둔다
                keep = 1 if self._buf[-1:] == MAGIC[:1] else 0
                self.dropped_bytes += len(self._buf) - keep
                del self._buf[: len(self._buf) - keep]
                break
            if idx > 0:
                self.dropped_bytes += idx
                del self._buf[:idx]
            if len(self._buf) < HEADER_SIZE:
                break
            _, version, msg_type, flags, _, seq, ts, plen = struct.unpack_from(HEADER_FMT, self._buf)
            if version != PROTOCOL_VERSION or plen > MAX_PAYLOAD:
                self._skip_one()
                continue
            total = HEADER_SIZE + plen + CRC_SIZE
            if len(self._buf) < total:
                break
            (crc,) = struct.unpack_from("<H", self._buf, HEADER_SIZE + plen)
            if crc != crc16_ccitt(bytes(self._buf[: HEADER_SIZE + plen])):
                self.crc_errors += 1
                self._skip_one()
                continue
            payload = bytes(self._buf[HEADER_SIZE : HEADER_SIZE + plen])
            del self._buf[:total]
            frames.append(Frame(msg_type, payload, seq, ts, flags, version))
        return frames

    def _skip_one(self) -> None:
        self.dropped_bytes += 1
        del self._buf[:1]


# ---------------------------------------------------------------- payloads


@dataclass
class Hello:
    """Pi -> Pico. Pico 는 cmd_timeout_ms 동안 ACTUATOR_CMD 가 없으면 스스로 토크를 해제해야 한다."""

    cmd_period_ms: int = 10
    cmd_timeout_ms: int = 100
    STRUCT = struct.Struct("<HH4x")

    def pack(self) -> bytes:
        return self.STRUCT.pack(self.cmd_period_ms, self.cmd_timeout_ms)

    @classmethod
    def unpack(cls, data: bytes) -> "Hello":
        return cls(*cls.STRUCT.unpack_from(data))


@dataclass
class HelloAck:
    fw_version: int = 0
    n_sts: int = 0
    n_pwm: int = 0
    capabilities: int = 0
    STRUCT = struct.Struct("<HBBI")

    def pack(self) -> bytes:
        return self.STRUCT.pack(self.fw_version, self.n_sts, self.n_pwm, self.capabilities)

    @classmethod
    def unpack(cls, data: bytes) -> "HelloAck":
        return cls(*cls.STRUCT.unpack_from(data))


@dataclass
class ActuatorCmd:
    """payload: mode u8, count u8, reserved u16, values i32 x count.

    값 순서는 액추에이터 인덱스 (0~5 = STS3215 ID 순, 6~9 = PWM 채널 순).
    """

    mode: int
    values: Sequence[int]
    HEAD = struct.Struct("<BBH")

    def pack(self) -> bytes:
        n = len(self.values)
        if n > MAX_ACTUATORS:
            raise ValueError(f"too many actuators: {n}")
        return self.HEAD.pack(int(self.mode), n, 0) + struct.pack(f"<{n}i", *(int(v) for v in self.values))

    @classmethod
    def unpack(cls, data: bytes) -> "ActuatorCmd":
        mode, n, _ = cls.HEAD.unpack_from(data)
        values = struct.unpack_from(f"<{n}i", data, cls.HEAD.size)
        return cls(mode, list(values))


@dataclass
class ActuatorState:
    position: int = 0      # 원시 단위 (STS tick / PWM us)
    velocity: int = 0      # 원시 단위/s
    effort: int = 0        # STS load (0.1% 단위) 등 원시 값
    temperature_c10: int = 0
    flags: int = 0         # 축별 오류 비트
    STRUCT = struct.Struct("<iiihH")


@dataclass
class State:
    """payload: status u16, error u16, count u8, reserved 3, ActuatorState x count (16 B 씩)."""

    status: int = 0
    error: int = 0
    actuators: List[ActuatorState] = field(default_factory=list)
    HEAD = struct.Struct("<HHB3x")

    def pack(self) -> bytes:
        out = self.HEAD.pack(self.status, self.error, len(self.actuators))
        for a in self.actuators:
            out += ActuatorState.STRUCT.pack(a.position, a.velocity, a.effort, a.temperature_c10, a.flags)
        return out

    @classmethod
    def unpack(cls, data: bytes) -> "State":
        status, error, n = cls.HEAD.unpack_from(data)
        acts = []
        off = cls.HEAD.size
        for _ in range(n):
            acts.append(ActuatorState(*ActuatorState.STRUCT.unpack_from(data, off)))
            off += ActuatorState.STRUCT.size
        return cls(status, error, acts)


@dataclass
class ErrorMsg:
    code: int
    message: str = ""

    def pack(self) -> bytes:
        return struct.pack("<H", self.code) + self.message.encode("utf-8")[: MAX_PAYLOAD - 2]

    @classmethod
    def unpack(cls, data: bytes) -> "ErrorMsg":
        (code,) = struct.unpack_from("<H", data)
        return cls(code, data[2:].decode("utf-8", "replace"))


def try_unpack_state(frame: Frame) -> Optional[State]:
    if frame.msg_type != MsgType.STATE:
        return None
    try:
        return State.unpack(frame.payload)
    except struct.error:
        return None
