"""Pi 5 쪽 TCP 클라이언트: Pico 2 + W5500 (서버) 와 연결을 유지한다.

- 백그라운드 스레드 1개가 연결/재연결/수신/하트비트를 담당한다.
- send_command() 는 제어 루프에서 바로 호출하며 블로킹 시간을 짧게 제한한다.
- 링크가 끊기면 지수 백오프로 재연결하고, 끊긴 동안 지령은 버린다 (오래된 지령 재전송 금지).
"""

from __future__ import annotations

import logging
import select
import socket
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Sequence

from .protocol import (
    ActuatorCmd,
    ErrorMsg,
    Frame,
    FrameDecoder,
    Hello,
    HelloAck,
    MsgType,
    State,
    encode_frame,
)

log = logging.getLogger(__name__)


@dataclass
class PicoLinkConfig:
    """값은 모두 설정 파일(config/comm_core.yaml 의 pico 섹션)에서 온다."""

    host: str
    port: int
    connect_timeout_s: float
    send_timeout_s: float           # 100Hz 루프를 막지 않도록 짧게
    link_timeout_s: float           # 이 시간 동안 아무 수신이 없으면 끊긴 것으로 판단
    heartbeat_interval_s: float
    reconnect_min_s: float
    reconnect_max_s: float
    cmd_period_ms: int
    pico_cmd_timeout_ms: int        # HELLO 로 Pico 에 전달하는 자체 워치독 시간


class PicoLink:
    def __init__(
        self,
        config: PicoLinkConfig,
        on_state: Optional[Callable[[State], None]] = None,
        on_link_change: Optional[Callable[[bool], None]] = None,
    ) -> None:
        self.cfg = config
        self._on_state = on_state
        self._on_link_change = on_link_change
        self._sock: Optional[socket.socket] = None
        self._send_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._seq = 0
        self._t0 = time.monotonic()
        self._last_tx = 0.0
        self._connected = False

        self.latest_state: Optional[State] = None
        self.latest_state_time = 0.0
        self.hello_ack: Optional[HelloAck] = None
        self.stats = {"tx_frames": 0, "rx_frames": 0, "reconnects": 0, "send_failures": 0, "errors": 0}

    # ------------------------------------------------------------ public

    @property
    def connected(self) -> bool:
        return self._connected

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="pico-link", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._connected:
            self.send_estop()
        self._close()
        if self._thread:
            self._thread.join(timeout=2.0)

    def send_command(self, mode: int, values: Sequence[int], flags: int = 0) -> bool:
        return self._send(MsgType.ACTUATOR_CMD, ActuatorCmd(mode, values).pack(), flags)

    def send_estop(self) -> bool:
        return self._send(MsgType.ESTOP, b"")

    # ------------------------------------------------------------ internals

    def _now_us(self) -> int:
        return int((time.monotonic() - self._t0) * 1e6)

    def _send(self, msg_type: int, payload: bytes, flags: int = 0) -> bool:
        sock = self._sock
        if sock is None or not self._connected:
            return False
        with self._send_lock:
            self._seq = (self._seq + 1) & 0xFFFF
            data = encode_frame(Frame(msg_type, payload, self._seq, self._now_us(), flags))
            try:
                sock.settimeout(self.cfg.send_timeout_s)
                sock.sendall(data)
            except OSError as e:
                # 송신 버퍼가 가득 차 타임아웃 났거나 연결이 끊김: 재연결로 정리
                self.stats["send_failures"] += 1
                log.warning("pico send failed: %s", e)
                self._mark_down()
                return False
            self._last_tx = time.monotonic()
            self.stats["tx_frames"] += 1
            return True

    def _set_connected(self, value: bool) -> None:
        if value != self._connected:
            self._connected = value
            if self._on_link_change:
                try:
                    self._on_link_change(value)
                except Exception:  # 콜백 오류가 통신 스레드를 죽이면 안 된다
                    log.exception("on_link_change callback failed")

    def _mark_down(self) -> None:
        self._set_connected(False)
        self._close()

    def _close(self) -> None:
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def _connect(self) -> bool:
        try:
            sock = socket.create_connection((self.cfg.host, self.cfg.port), timeout=self.cfg.connect_timeout_s)
        except OSError as e:
            log.debug("pico connect failed: %s", e)
            return False
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        self._sock = sock
        self._set_connected(True)
        self._send(MsgType.HELLO, Hello(self.cfg.cmd_period_ms, self.cfg.pico_cmd_timeout_ms).pack())
        log.info("pico link up %s:%d", self.cfg.host, self.cfg.port)
        return True

    def _run(self) -> None:
        backoff = self.cfg.reconnect_min_s
        first = True
        while not self._stop.is_set():
            if not first:
                self.stats["reconnects"] += 1
            first = False
            if not self._connect():
                self._stop.wait(backoff)
                backoff = min(backoff * 2, self.cfg.reconnect_max_s)
                continue
            backoff = self.cfg.reconnect_min_s
            self._serve()
            self._mark_down()
            if not self._stop.is_set():
                log.warning("pico link down, reconnecting")

    def _serve(self) -> None:
        decoder = FrameDecoder()
        last_rx = time.monotonic()
        while not self._stop.is_set():
            sock = self._sock
            if sock is None:
                return
            try:
                readable, _, _ = select.select([sock], [], [], 0.02)
            except (OSError, ValueError):
                return
            now = time.monotonic()
            if readable:
                try:
                    data = sock.recv(4096)
                except (BlockingIOError, socket.timeout):
                    data = None
                except OSError:
                    return
                if data == b"":
                    return  # 원격 종료
                if data:
                    last_rx = now
                    for frame in decoder.feed(data):
                        self._handle(frame)
            if now - last_rx > self.cfg.link_timeout_s:
                log.warning("pico link timeout (%.0f ms no rx)", (now - last_rx) * 1e3)
                return
            if now - self._last_tx > self.cfg.heartbeat_interval_s:
                self._send(MsgType.HEARTBEAT, b"")

    def _handle(self, frame: Frame) -> None:
        self.stats["rx_frames"] += 1
        try:
            if frame.msg_type == MsgType.STATE:
                state = State.unpack(frame.payload)
                self.latest_state = state
                self.latest_state_time = time.monotonic()
                if self._on_state:
                    self._on_state(state)
            elif frame.msg_type == MsgType.HELLO_ACK:
                self.hello_ack = HelloAck.unpack(frame.payload)
                log.info("pico hello: %s", self.hello_ack)
            elif frame.msg_type == MsgType.ERROR:
                self.stats["errors"] += 1
                err = ErrorMsg.unpack(frame.payload)
                log.error("pico error %d: %s", err.code, err.message)
            # HEARTBEAT 및 미정의 타입은 수신 시각 갱신만 하고 무시 (향후 확장 대비)
        except Exception:
            log.exception("failed to handle pico frame type=0x%02x", frame.msg_type)
