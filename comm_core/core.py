"""3-A 통신 코어: 100Hz 루프에서 팀 UDP 채널과 Pico TCP 링크를 잇는다.

매 틱:
  1. CH2 (5556) 최신 지령 수신 -> 검증 -> 클램핑 -> 워치독 갱신
  2. 워치독 만료 시 전 축 토크 0 지령으로 대체
  3. Pico 로 ACTUATOR_CMD 송신 (Pico 자체 워치독도 함께 유지)
  4. CH3 (5557) 슬립 신호 / CH4 (5559) VLA 액션 수신 -> 설정 시 3-B 로 즉시 중계
  5. Pico 최신 상태 -> CH1 (5555) 텔레메트리, CH4 프로프리오셉션 송신
"""

from __future__ import annotations

import json
import logging
import time
from typing import Any, Dict, List, Optional, Tuple

from .config import ActuatorConfig, CoreConfig
from .kinematics import HandKinematics
from .pico_link import PicoLink
from .protocol import CmdFlag, CmdMode, State
from .team_channels import UdpEndpoint, parse_float_list

log = logging.getLogger(__name__)

STATUS_NORMAL = "NORMAL"
STATUS_WATCHDOG = "WATCHDOG"
STATUS_LINK_DOWN = "PICO_DISCONNECTED"


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class CommandTranslator:
    """CH2 JSON 지령 -> (CmdMode, 원시 정수 값 리스트). 잘못된 지령은 None."""

    def __init__(self, actuators: List[ActuatorConfig]) -> None:
        self.acts = actuators

    def translate(self, msg: Dict[str, Any]) -> Optional[Tuple[int, List[int]]]:
        mode = msg.get("mode", "torque")
        n = len(self.acts)
        if mode == "torque":
            torques = parse_float_list(msg.get("torques"), n)
            if torques is None:
                return None
            return CmdMode.TORQUE, [self.torque_to_raw(i, t) for i, t in enumerate(torques)]
        if mode == "position":
            positions = parse_float_list(msg.get("positions"), n)
            if positions is None:
                return None
            return CmdMode.POSITION, [self.rad_to_raw(i, q) for i, q in enumerate(positions)]
        if mode == "raw_position":
            raw = parse_float_list(msg.get("raw"), n)
            if raw is None:
                return None
            return CmdMode.POSITION, [self.clamp_raw(i, r) for i, r in enumerate(raw)]
        return None

    def torque_to_raw(self, i: int, nm: float) -> int:
        lim = abs(self.acts[i].torque_limit_nm)
        return int(round(clamp(nm, -lim, lim) * 1000.0))  # mNm

    def rad_to_raw(self, i: int, q: float) -> int:
        a = self.acts[i]
        return self.clamp_raw(i, a.raw_zero + q / a.rad_per_raw)

    def clamp_raw(self, i: int, raw: float) -> int:
        a = self.acts[i]
        return int(round(clamp(raw, a.raw_min, a.raw_max)))

    def zero_torque(self) -> Tuple[int, List[int]]:
        return CmdMode.TORQUE, [0] * len(self.acts)


class TelemetryBuilder:
    """Pico 원시 상태 -> SI 단위 관절/액추에이터 값. 매핑은 hand_model.yaml 의 joints[].source."""

    def __init__(self, cfg: CoreConfig) -> None:
        self.cfg = cfg
        hand = cfg.hand
        # 이름 -> 인덱스는 시작 시 한 번만 풀어 둔다 (루프에서 문자열 검색 없음)
        self._plan = []
        for j in hand.joints:
            terms = []
            for t in j.source or []:
                if t.actuator is not None:
                    terms.append((True, hand.actuator_index(t.actuator), t.scale))
                else:
                    terms.append((False, hand.joint_index(t.joint), t.scale))
            self._plan.append((j.source is not None, j.offset, terms))
        self.kinematics = HandKinematics(hand) if cfg.channels.include_fingertips else None

    def actuators(self, state: Optional[State]) -> Tuple[List[float], List[float], List[float]]:
        n = len(self.cfg.actuators)
        pos, vel, tq = [0.0] * n, [0.0] * n, [0.0] * n
        if state is None:
            return pos, vel, tq
        for i, (a, s) in enumerate(zip(self.cfg.actuators, state.actuators)):
            pos[i] = (s.position - a.raw_zero) * a.rad_per_raw
            vel[i] = s.velocity * a.rad_per_raw
            tq[i] = s.effort * a.nm_per_effort
        return pos, vel, tq

    def joints(self, act_pos: List[float], act_vel: List[float]) -> Tuple[List[float], List[float]]:
        q = [0.0] * self.cfg.n_joints
        dq = [0.0] * self.cfg.n_joints
        for j, (measured, offset, terms) in enumerate(self._plan):
            if not measured:
                continue
            qj, dqj = offset, 0.0
            for is_act, idx, scale in terms:
                if is_act:
                    qj += scale * act_pos[idx]
                    dqj += scale * act_vel[idx]
                else:
                    qj += scale * q[idx]
                    dqj += scale * dq[idx]
            q[j], dq[j] = qj, dqj
        return q, dq


class CommCore:
    def __init__(self, cfg: CoreConfig, link: Optional[PicoLink] = None) -> None:
        cfg.validate()
        self.cfg = cfg
        ch = cfg.channels
        self.link = link or PicoLink(cfg.pico)
        self.translator = CommandTranslator(cfg.actuators)
        self.telemetry = TelemetryBuilder(cfg)

        self.tx = UdpEndpoint((ch.bind_host, ch.telemetry_port))
        self.cmd_rx = UdpEndpoint((ch.bind_host, ch.command_port))
        self.slip_rx = UdpEndpoint((ch.bind_host, ch.slip_port))
        self.vla = UdpEndpoint((ch.bind_host, ch.vla_port))

        self.seq = 0
        self.last_cmd: Tuple[int, List[int]] = self.translator.zero_torque()
        self.last_cmd_time: Optional[float] = None
        self.watchdog_tripped = True  # 첫 지령 전까지는 토크 0
        self.latest_slip: Optional[Dict[str, Any]] = None
        self.latest_vla_action: Optional[Dict[str, Any]] = None
        self._next_proprio = 0.0
        self.stats = {"cmd_rejected": 0, "watchdog_trips": 0, "overruns": 0}

    # ------------------------------------------------------------ lifecycle

    def start(self) -> None:
        self.link.start()

    def close(self) -> None:
        self.link.stop()
        for ep in (self.tx, self.cmd_rx, self.slip_rx, self.vla):
            ep.close()

    def run(self, duration_s: Optional[float] = None) -> None:
        period = 1.0 / self.cfg.loop_rate_hz
        t_end = None if duration_s is None else time.monotonic() + duration_s
        next_t = time.monotonic()
        while t_end is None or time.monotonic() < t_end:
            self.step(time.monotonic())
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                self.stats["overruns"] += 1
                if -delay > period:  # 크게 밀렸으면 따라잡지 말고 재정렬
                    next_t = time.monotonic()

    # ------------------------------------------------------------ one tick

    def step(self, now: float) -> None:
        self._poll_command(now)
        mode, values = self._current_command(now)
        flags = CmdFlag.WATCHDOG_TRIPPED if self.watchdog_tripped else 0
        self.link.send_command(mode, values, flags)
        self._poll_slip()
        self._poll_vla()
        self._publish(now)

    def _poll_command(self, now: float) -> None:
        latest = self.cmd_rx.recv_latest()
        if latest is None:
            return
        msg, _ = latest
        cmd = self.translator.translate(msg)
        if cmd is None:
            self.stats["cmd_rejected"] += 1
            return
        self.last_cmd = cmd
        self.last_cmd_time = now
        if self.watchdog_tripped:
            log.info("command stream (re)started")
        self.watchdog_tripped = False

    def _current_command(self, now: float) -> Tuple[int, List[int]]:
        expired = self.last_cmd_time is None or now - self.last_cmd_time > self.cfg.command_watchdog_s
        if expired:
            if not self.watchdog_tripped:
                self.stats["watchdog_trips"] += 1
                log.warning("command watchdog tripped: zero torque")
            self.watchdog_tripped = True
            return self.translator.zero_torque()
        return self.last_cmd

    def _poll_slip(self) -> None:
        dest = self.cfg.channels.slip_relay_dest
        for msg, _ in self.slip_rx.recv_all():
            self.latest_slip = msg
            if dest is not None:
                self.tx.send_json(msg, dest)

    def _poll_vla(self) -> None:
        dest = self.cfg.channels.vla_action_relay_dest
        for msg, _ in self.vla.recv_all():
            self.latest_vla_action = msg
            if dest is not None:
                self.tx.send_json(msg, dest)

    def status(self) -> str:
        if not self.link.connected:
            return STATUS_LINK_DOWN
        if self.watchdog_tripped:
            return STATUS_WATCHDOG
        return STATUS_NORMAL

    def _publish(self, now: float) -> None:
        ch = self.cfg.channels
        state = self.link.latest_state if self.link.connected else None
        pos, vel, tq = self.telemetry.actuators(state)
        q, dq = self.telemetry.joints(pos, vel)
        self.seq += 1
        ts = time.time()
        msg = {
            "seq": self.seq,
            "timestamp": ts,
            "status": self.status(),
            "q": q,
            "dq": dq,
            "actuator_pos": pos,
            "actuator_vel": vel,
            "actuator_torque": tq,
        }
        if self.telemetry.kinematics is not None:
            msg["fingertips"] = self.telemetry.kinematics.fingertips(q)
        self.tx.send_json(msg, ch.telemetry_dest)
        if ch.proprio_dest is not None and now >= self._next_proprio:
            self._next_proprio = now + 1.0 / ch.proprio_rate_hz
            self.vla.send_json({"seq": self.seq, "timestamp": ts, "q": q, "dq": dq}, ch.proprio_dest)

    def snapshot(self) -> str:
        return json.dumps(
            {
                "status": self.status(),
                "link": self.link.stats,
                "core": self.stats,
                "cmd_rx": {"ok": self.cmd_rx.rx_ok, "bad": self.cmd_rx.rx_bad},
            }
        )
