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
    DIAG = 0x21           # Pico -> Pi: 진단 카운터 (주기적)
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


# HELLO_ACK.capabilities 비트
CAP_STATE_EXT = 0x01   # STATE 축 레코드 24 B (age/voltage/fault level 포함)
CAP_DIAG = 0x02        # DIAG 메시지 송신
CAP_CMD_SEQ_ECHO = 0x04  # STATE 에 응답한 ACTUATOR_CMD 의 seq 를 담음

# STATE.status 비트 (펌웨어 session.h 와 같음)
ST_TORQUE_ENABLED = 0x0001
ST_WATCHDOG = 0x0002
ST_ESTOP = 0x0004
ST_BUS_FAULT = 0x0008
ST_HELLO_DONE = 0x0010
ST_SEQ_GAP = 0x0020
ST_LEVEL_SHIFT = 8      # bit8-9: 고장 단계 (FaultLevel)
ST_LEVEL_MASK = 0x0300


class FaultLevel(IntEnum):
    OK = 0         # 정상
    DEGRADED = 1   # 경고: 재시도/누락/추정값 사용 중이지만 제어는 계속
    HOLD = 2       # 일부 축 관측 불가: 해당 축 목표를 고정
    SAFE_OFF = 3   # 토크 해제 (워치독/ESTOP/보호)


# 축 flags 비트 (펌웨어 session.h AX_* 와 같음)
AX_NO_RESPONSE = 0x0001   # 피드백 끊김 (lost_ms 초과)
AX_BAD_PACKET = 0x0002    # 마지막 읽기가 깨진 패킷
AX_SERVO_ERROR = 0x0004   # 서보 상태 바이트 오류
AX_TORQUE_ON = 0x0008
AX_CLAMPED = 0x0010
AX_STALE = 0x0020         # 피드백이 stale_ms 보다 오래됨
AX_ESTIMATED = 0x0040     # 이번 값은 측정이 아니라 관측기 추정값
AX_IMPLAUSIBLE = 0x0080   # 마지막 샘플이 타당성 검사에서 버려짐
AX_OVERTEMP = 0x0100
AX_VOLTAGE = 0x0200
AX_OVERLOAD = 0x0400
AX_CMD_MISMATCH = 0x0800  # 서보 레지스터 재확인 결과가 지령과 다름 (재전송함)
AX_SLEW_LIMITED = 0x1000  # 지령 변화율 제한이 걸림
AX_HOLD = 0x2000          # 관측 불가로 목표를 고정 중
AX_PROTECT_OFF = 0x4000   # 과열/과부하 보호로 토크 해제

AGE_UNKNOWN = 0xFFFF

# DIAG 필드 순서 (펌웨어 proto.h 의 DIAG_FIELDS 와 같음, tests 가 일치 여부를 검사)
DIAG_FIELDS = [
    "uptime_ms", "reset_cause", "fault_level", "rx_frames", "crc_errors", "dropped_bytes", "seq_gaps",
    "cmd_interval_max_us", "cmd_interval_mean_us", "bus_timeouts", "bus_bad_packets", "bus_echo_errors",
    "bus_retries", "implausible_samples", "verify_mismatches", "slew_limited", "spi_errors", "w5500_reinits",
    "link_drops", "loop_overruns", "loop_max_us", "watchdog_trips", "estops", "protect_trips",
]


@dataclass
class ActuatorState:
    position: int = 0      # 원시 단위 (STS tick / PWM us)
    velocity: int = 0      # 원시 단위/s
    effort: int = 0        # STS load (0.1% 단위) 등 원시 값
    temperature_c10: int = 0
    flags: int = 0         # 축별 오류 비트 (AX_*)
    age_ms: int = 0        # 마지막 유효 측정 이후 시간 (AGE_UNKNOWN = 측정 없음). 구형 펌웨어는 0
    voltage_dv: int = 0    # 0.1 V
    level: int = 0         # 축 고장 단계 (FaultLevel)
    STRUCT = struct.Struct("<iiihH")          # 구형 16 B
    EXT = struct.Struct("<iiihHHBB4x")        # 확장 24 B (뒤 4 B 예약)


@dataclass
class State:
    """payload: status u16, error u16, count u8, axis_size u8, cmd_seq u16, 축 레코드 x count.

    axis_size 0 은 구형 16 B 레코드 (예전 reserved 3 바이트가 0 이던 형식과 호환).
    """

    status: int = 0
    error: int = 0
    actuators: List[ActuatorState] = field(default_factory=list)
    cmd_seq: int = 0
    axis_size: int = 0
    HEAD = struct.Struct("<HHBBH")

    @property
    def level(self) -> int:
        return (self.status & ST_LEVEL_MASK) >> ST_LEVEL_SHIFT

    def pack(self) -> bytes:
        ext = self.axis_size == ActuatorState.EXT.size
        out = self.HEAD.pack(self.status, self.error, len(self.actuators), self.axis_size, self.cmd_seq)
        for a in self.actuators:
            if ext:
                out += ActuatorState.EXT.pack(a.position, a.velocity, a.effort, a.temperature_c10, a.flags,
                                              a.age_ms, a.voltage_dv, a.level)
            else:
                out += ActuatorState.STRUCT.pack(a.position, a.velocity, a.effort, a.temperature_c10, a.flags)
        return out

    @classmethod
    def unpack(cls, data: bytes) -> "State":
        status, error, n, axis_size, cmd_seq = cls.HEAD.unpack_from(data)
        size = axis_size or ActuatorState.STRUCT.size
        if size < ActuatorState.STRUCT.size:
            raise struct.error(f"axis record too small: {size}")
        acts = []
        off = cls.HEAD.size
        for _ in range(n):
            if size >= ActuatorState.EXT.size:
                acts.append(ActuatorState(*ActuatorState.EXT.unpack_from(data, off)))
            else:
                acts.append(ActuatorState(*ActuatorState.STRUCT.unpack_from(data, off)))
            off += size  # 더 큰 레코드는 앞부분만 읽는다 (향후 확장 대비)
        return cls(status, error, acts, cmd_seq, axis_size)


def unpack_diag(data: bytes) -> dict:
    """payload: count u8, reserved 3, u32 x count. 모르는 뒷 필드는 field_N 으로."""
    (n,) = struct.unpack_from("<B3x", data)
    vals = struct.unpack_from(f"<{n}I", data, 4)
    return {(DIAG_FIELDS[i] if i < len(DIAG_FIELDS) else f"field_{i}"): v for i, v in enumerate(vals)}


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
