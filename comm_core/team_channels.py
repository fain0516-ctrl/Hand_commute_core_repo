"""타 팀용 UDP JSON 채널 (COMM_SOCKET_SPEC 기준).

CH1 :5555 TX  3-A -> 3-B  관절 텔레메트리 100Hz
CH2 :5556 RX  3-B -> 3-A  모터 지령 (100ms 워치독, 토크 클램핑)
CH3 :5557 RX  4팀 -> 3-A  슬립 반사 신호 (수신 즉시 3-B 로 중계 가능)
CH4 :5559 RX/TX 3팀 <-> 1팀  관절 프로프리오셉션 TX / VLA 액션 RX

모든 소켓은 논블로킹이며 제어 루프가 매 틱 poll 한다 (스레드 없음).
"""

from __future__ import annotations

import json
import logging
import socket
from typing import Any, Dict, List, Optional, Tuple

log = logging.getLogger(__name__)

Addr = Tuple[str, int]
MAX_DATAGRAM = 65507


class UdpEndpoint:
    """하나의 UDP 포트. bind 주소가 없으면 송신 전용."""

    def __init__(self, bind: Optional[Addr] = None) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if bind is not None:
            self.sock.bind(bind)
        self.sock.setblocking(False)
        self.rx_ok = 0
        self.rx_bad = 0
        self.tx_ok = 0
        self.tx_fail = 0

    @property
    def address(self) -> Addr:
        return self.sock.getsockname()

    def recv_all(self) -> List[Tuple[Dict[str, Any], Addr]]:
        """버퍼에 쌓인 JSON 데이터그램을 모두 읽는다. 깨진 패킷은 세고 버린다."""
        out = []
        while True:
            try:
                data, addr = self.sock.recvfrom(MAX_DATAGRAM)
            except (BlockingIOError, InterruptedError):
                break
            except OSError as e:  # 예: ICMP port unreachable 로 인한 ECONNREFUSED
                log.debug("udp recv error on %s: %s", self.address, e)
                continue
            try:
                msg = json.loads(data)
                if not isinstance(msg, dict):
                    raise ValueError("not an object")
            except ValueError:
                self.rx_bad += 1
                continue
            self.rx_ok += 1
            out.append((msg, addr))
        return out

    def recv_latest(self) -> Optional[Tuple[Dict[str, Any], Addr]]:
        """최신 1 프레임만 사용 (IF-03 '논블로킹 최신 1프레임 취득' 규칙)."""
        msgs = self.recv_all()
        return msgs[-1] if msgs else None

    def send_json(self, msg: Dict[str, Any], addr: Addr) -> bool:
        return self.send_raw(json.dumps(msg, separators=(",", ":")).encode(), addr)

    def send_raw(self, data: bytes, addr: Addr) -> bool:
        try:
            self.sock.sendto(data, addr)
        except OSError as e:
            self.tx_fail += 1
            log.debug("udp send to %s failed: %s", addr, e)
            return False
        self.tx_ok += 1
        return True

    def close(self) -> None:
        self.sock.close()


def parse_float_list(value: Any, length: int) -> Optional[List[float]]:
    """길이와 타입이 맞는 숫자 리스트만 통과. NaN/inf 는 거부."""
    if not isinstance(value, list) or len(value) != length:
        return None
    out = []
    for v in value:
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            return None
        f = float(v)
        if f != f or f in (float("inf"), float("-inf")):
            return None
        out.append(f)
    return out
