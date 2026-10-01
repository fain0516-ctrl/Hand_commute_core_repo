"""Pico 2 + W5500 펌웨어 대용 TCP 서버 (하드웨어 없이 개발/테스트용).

실행: python3 -m comm_core.fake_pico --port 5000
펌웨어 구현 시 동작 기준이 되도록 실제 Pico 에 요구되는 규칙을 그대로 따른다:
  - HELLO 수신 시 HELLO_ACK 응답
  - ACTUATOR_CMD 마다 STATE 1개 응답
  - HEARTBEAT 에 HEARTBEAT 응답
  - cmd_timeout_ms 동안 지령이 없거나 ESTOP 수신 시 전 축 토크 해제 (torque_enabled=False)
"""

from __future__ import annotations

import argparse
import logging
import socket
import threading
import time
from typing import List, Optional

from .protocol import (
    ActuatorCmd,
    ActuatorState,
    CmdMode,
    Frame,
    FrameDecoder,
    Hello,
    HelloAck,
    MsgType,
    State,
    encode_frame,
)

log = logging.getLogger(__name__)


class FakePico:
    def __init__(self, host: str = "127.0.0.1", port: int = 0, n_sts: int = 6, n_pwm: int = 4) -> None:
        self.n_sts, self.n_pwm = n_sts, n_pwm
        n = n_sts + n_pwm
        self.positions: List[int] = [2048] * n_sts + [1500] * n_pwm
        self.last_cmd: Optional[ActuatorCmd] = None
        self.last_cmd_flags = 0
        self.cmd_count = 0
        self.torque_enabled = False
        self.cmd_timeout_ms = 100
        self.estops = 0
        self.connections = 0
        self.mute = False  # True 면 응답하지 않음 (링크 타임아웃 시험용)
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((host, port))
        self._srv.listen(1)
        self._stop = threading.Event()
        self._conn: Optional[socket.socket] = None
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._seq = 0

    @property
    def port(self) -> int:
        return self._srv.getsockname()[1]

    def start(self) -> "FakePico":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self.drop_connection()
        self._srv.close()
        self._thread.join(timeout=2.0)

    def drop_connection(self) -> None:
        conn, self._conn = self._conn, None
        if conn:
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            conn.close()

    def _send(self, conn: socket.socket, msg_type: int, payload: bytes = b"") -> None:
        if self.mute:
            return
        self._seq = (self._seq + 1) & 0xFFFF
        try:
            conn.sendall(encode_frame(Frame(msg_type, payload, self._seq, int(time.monotonic() * 1e6))))
        except OSError:
            pass

    def _serve(self) -> None:
        self._srv.settimeout(0.05)
        while not self._stop.is_set():
            try:
                conn, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.settimeout(0.01)
            self._conn = conn
            self.connections += 1
            self._handle_conn(conn)
            if self._conn is conn:
                self._conn = None
            conn.close()
            self.torque_enabled = False

    def _handle_conn(self, conn: socket.socket) -> None:
        dec = FrameDecoder()
        last_cmd_t = time.monotonic()
        while not self._stop.is_set() and self._conn is conn:
            try:
                data = conn.recv(4096)
                if not data:
                    return
            except socket.timeout:
                data = b""
            except OSError:
                return
            for f in dec.feed(data):
                if f.msg_type == MsgType.HELLO:
                    self.cmd_timeout_ms = Hello.unpack(f.payload).cmd_timeout_ms
                    self._send(conn, MsgType.HELLO_ACK, HelloAck(1, self.n_sts, self.n_pwm, 0).pack())
                elif f.msg_type == MsgType.HEARTBEAT:
                    self._send(conn, MsgType.HEARTBEAT)
                elif f.msg_type == MsgType.ESTOP:
                    self.estops += 1
                    self.torque_enabled = False
                elif f.msg_type == MsgType.ACTUATOR_CMD:
                    last_cmd_t = time.monotonic()
                    self._apply(ActuatorCmd.unpack(f.payload), f.flags)
                    self._send(conn, MsgType.STATE, self._state().pack())
            if (time.monotonic() - last_cmd_t) * 1000 > self.cmd_timeout_ms:
                self.torque_enabled = False

    def _apply(self, cmd: ActuatorCmd, flags: int) -> None:
        self.last_cmd, self.last_cmd_flags = cmd, flags
        self.cmd_count += 1
        if cmd.mode == CmdMode.POSITION:
            self.torque_enabled = True
            for i, v in enumerate(cmd.values[: len(self.positions)]):
                self.positions[i] = v  # 이상적 추종
        elif cmd.mode == CmdMode.TORQUE:
            self.torque_enabled = any(cmd.values)

    def _state(self) -> State:
        return State(
            status=1 if self.torque_enabled else 0,
            actuators=[ActuatorState(position=p, temperature_c10=300) for p in self.positions],
        )


def main() -> None:
    p = argparse.ArgumentParser(description="Fake Pico 2 + W5500 TCP server")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=5000)
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO)
    pico = FakePico(args.host, args.port).start()
    log.info("fake pico listening on %s:%d", args.host, pico.port)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pico.stop()


if __name__ == "__main__":
    main()
