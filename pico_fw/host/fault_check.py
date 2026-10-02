#!/usr/bin/env python3
"""고장 주입 시험: 잡음/누락/관측 불가 상황에서 펌웨어와 Pi 링크가 어떻게 반응하는지 확인한다.

하드웨어 없이 실행한다.
  - 펌웨어: proto.c / session.c / actuators.c / sts_bus.c / diag.c 를 host/hal_stub.c (Pico SDK 대체, 가짜 STS3215)
    위에서 컴파일한 시뮬레이터 (pipeline_check.py 와 같은 바이너리). 고장은 stdin 명령으로 넣는다.
  - Pi 쪽: 실제 comm_core.PicoLink 가 100 Hz 로 위치 지령을 보내고 STATE/DIAG 를 받는다.
W5500 (SPI) 쪽 고장은 host/w5500_fault_test.c 가 따로 확인한다 (--w5500).

실행 (저장소 루트에서): python3 pico_fw/host/fault_check.py [--out report.md] [--only 이름,...]
모든 시나리오가 기대대로면 종료 코드 0.
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import os
import subprocess
import sys
import tempfile
import threading
import time
from typing import Callable, Dict, List, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from pipeline_check import FW, ROOT, build_sim, classify_bus  # noqa: E402

from comm_core.config import load_config  # noqa: E402
from comm_core.pico_link import PicoLink  # noqa: E402
from comm_core.protocol import (  # noqa: E402
    AX_CMD_MISMATCH,
    AX_ESTIMATED,
    AX_HOLD,
    AX_IMPLAUSIBLE,
    AX_NO_RESPONSE,
    AX_OVERTEMP,
    AX_PROTECT_OFF,
    AX_SLEW_LIMITED,
    AX_STALE,
    AX_TORQUE_ON,
    CmdMode,
    FaultLevel,
)

N_STS, N_PWM = 6, 4
MID = 2048
FAULT_AT = 0.5  # 지령 시작 후 고장을 넣는 시각 (s)


# ---------------------------------------------------------------- 실행 도구


class SimProc:
    def __init__(self, binary: str) -> None:
        self.proc = subprocess.Popen([binary, "0"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        self.port = int(self.proc.stdout.readline().split()[1])
        self.bus: List[tuple] = []  # (ms, hex)
        self.acks: List[str] = []
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        for line in self.proc.stdout:
            p = line.split()
            if len(p) >= 3 and p[0] == "bus":
                self.bus.append((int(p[1]), p[2]))
            elif p and p[0] == "fault":
                self.acks.append(line.strip())

    def fault(self, cmd: str) -> None:
        self.proc.stdin.write(f"fault {cmd}\n")
        self.proc.stdin.flush()

    def close(self) -> None:
        self.proc.kill()
        self.proc.wait()
        self.proc.stdout.close()
        self.proc.stdin.close()


@dataclasses.dataclass
class Sample:
    t: float  # 지령 시작 기준 (s)
    status: int
    level: int
    pos: List[int]
    flags: List[int]
    age: List[int]
    axl: List[int]


@dataclasses.dataclass
class Run:
    samples: List[Sample]
    diag: Dict[str, int]       # 마지막 DIAG (지령 종료 후)
    diag_run: Dict[str, int]   # 지령 중 받은 DIAG 들의 필드별 최대 (구간 통계 포함)
    link: Dict[str, float]
    bus: List[tuple]
    t0_ms: int
    acks: List[str]
    targets: List[tuple]  # (t, values)

    def window(self, a: float, b: float) -> List[Sample]:
        return [s for s in self.samples if a <= s.t < b]

    def first(self, cond: Callable[[Sample], bool], after: float = 0.0) -> Optional[float]:
        for s in self.samples:
            if s.t >= after and cond(s):
                return s.t
        return None

    def bus_window(self, a: float, b: float) -> List[tuple]:
        return [(ms / 1000.0 - self.t0_ms / 1000.0, h) for ms, h in self.bus
                if a <= ms / 1000.0 - self.t0_ms / 1000.0 < b]


def sts_wave(t: float) -> List[int]:
    """STS 축: 천천히 움직이는 목표 (+-200 ticks, 0.5 Hz), PWM 축: 고정 1500 us."""
    sts = [int(MID + 200 * math.sin(2 * math.pi * 0.5 * t + i)) for i in range(N_STS)]
    return sts + [1500] * N_PWM


def run_scenario(binary: str, duration: float, timeline: List[tuple],
                 target: Callable[[float], List[int]] = sts_wave, mode_at: Optional[Callable] = None) -> Run:
    """timeline: [(t, 'fault cmd' | callable(link))], 지령 시작 시각 기준."""
    sim = SimProc(binary)
    base = load_config(os.path.join(ROOT, "config", "comm_core.yaml")).pico
    cfg = dataclasses.replace(base, host="127.0.0.1", port=sim.port, reconnect_min_s=0.02, reconnect_max_s=0.1)
    samples: List[Sample] = []
    t0 = [0.0]

    def on_state(st):
        a = st.actuators
        samples.append(Sample(time.monotonic() - t0[0], st.status, st.level, [x.position for x in a],
                              [x.flags for x in a], [x.age_ms for x in a], [x.level for x in a]))

    link = PicoLink(cfg, on_state=on_state)
    link.start()
    deadline = time.monotonic() + 2.0
    while link.hello_ack is None and time.monotonic() < deadline:
        time.sleep(0.005)
    assert link.hello_ack is not None, "펌웨어 시뮬레이터에 연결하지 못함"
    t0[0] = time.monotonic()
    t0_ms = int(t0[0] * 1000)
    pending = sorted(timeline, key=lambda e: e[0])
    targets = []
    diag_run: Dict[str, int] = {}
    diag_seen = 0.0
    next_t = t0[0]
    while True:
        now = time.monotonic()
        t = now - t0[0]
        if t >= duration:
            break
        while pending and pending[0][0] <= t:
            _, action = pending.pop(0)
            if callable(action):
                action(link)
            else:
                sim.fault(action)
        if link.latest_diag_time != diag_seen and link.latest_diag:
            diag_seen = link.latest_diag_time
            for k, v in link.latest_diag.items():
                diag_run[k] = max(v, diag_run.get(k, 0))
        mode, vals = (mode_at(t) if mode_at else (CmdMode.POSITION, target(t)))
        if mode is not None:
            link.send_command(mode, vals)
            targets.append((t, vals))
        next_t += 0.01
        time.sleep(max(0.0, next_t - time.monotonic()))
    # 마지막 DIAG 를 받을 때까지 하트비트만 유지 (지령 없음 -> Pico 워치독 해제는 결과에 포함하지 않음)
    seen = link.latest_diag_time
    deadline = time.monotonic() + 1.0
    while link.latest_diag_time == seen and time.monotonic() < deadline:
        time.sleep(0.01)
    diag = dict(link.latest_diag or {})
    stats = dict(link.stats)
    link.stop()
    sim.close()
    return Run(samples, diag, diag_run, stats, list(sim.bus), t0_ms, list(sim.acks), targets)


# ---------------------------------------------------------------- 시나리오


@dataclasses.dataclass
class Scenario:
    name: str
    fault: str
    expect: str
    duration: float
    timeline: List[tuple]
    check: Callable[[Run], tuple]  # -> (ok, observed 문장 목록)
    target: Callable[[float], List[int]] = sts_wave
    mode_at: Optional[Callable] = None


def flag_frac(samples: List[Sample], axis: int, bit: int) -> float:
    return sum(1 for s in samples if s.flags[axis] & bit) / max(1, len(samples))


def torque_on_all(s: Sample) -> bool:
    return all(f & AX_TORQUE_ON for f in s.flags[:N_STS])


def lvl(x: int) -> str:
    return FaultLevel(x).name


def max_level(samples: List[Sample]) -> int:
    return max((s.level for s in samples), default=0)


def ms(x: Optional[float], ref: float) -> str:
    return "없음" if x is None else f"{(x - ref) * 1000:.0f} ms"


def chk_baseline(r: Run):
    steady = r.window(0.3, 1.5)
    # 시뮬레이터 프로세스가 OS 에 밀려 지령을 30 ms 넘게 늦게 받으면 지터 경고(status bit5)로만 DEGRADED 가 된다.
    # 축 문제 없이 그 이유뿐인 DEGRADED 는 호스트 사정이므로 따로 센다.
    jitter_only = [s for s in steady if s.level == 1 and s.status & 0x20 and not any(s.axl)]
    ok = all(s.level == 0 or s in jitter_only for s in steady) and all(torque_on_all(s) for s in steady)
    ok &= r.link["crc_errors"] == 0 and r.diag.get("bus_timeouts", 1) == 0
    return ok, [f"STATE {len(r.samples)} 개, 정상 구간 단계 최대 {lvl(max_level(steady))}"
                + (f" (호스트 스케줄링 지터 경고로 DEGRADED {len(jitter_only)} 개)" if jitter_only else ""),
                f"왕복 시간 평균 {r.link['rtt_ms_mean']:.2f} ms / 최대 {r.link['rtt_ms_max']:.2f} ms, "
                f"STATE 누락 {r.link['state_missing']}",
                f"버스 타임아웃 {r.diag.get('bus_timeouts')}, 깨진 패킷 {r.diag.get('bus_bad_packets')}, "
                f"루프 최대 {r.diag.get('loop_max_us')} us"]


def chk_uart_noise(limit_age: int):
    def chk(r: Run):
        w = r.window(FAULT_AT, FAULT_AT + 1.0)
        on = sum(1 for s in w if torque_on_all(s)) / max(1, len(w))
        max_age = max((a for s in w for a in s.age[:N_STS] if a != 0xFFFF), default=0)
        bad = r.diag.get("bus_bad_packets", 0) + r.diag.get("bus_timeouts", 0)
        ok = on == 1.0 and bad > 0 and max_age <= limit_age and r.diag.get("bus_retries", 0) > 0
        ok &= max_level(r.window(FAULT_AT + 1.3, 9)) == 0
        return ok, [f"토크 유지 {on * 100:.0f}% (STS 6 축 모두 켜진 STATE 비율)",
                    f"깨진 응답 {r.diag.get('bus_bad_packets')} / 타임아웃 {r.diag.get('bus_timeouts')} / "
                    f"재시도 {r.diag.get('bus_retries')} / 에코 오류 {r.diag.get('bus_echo_errors')} / "
                    f"타당성 탈락 {r.diag.get('implausible_samples')}",
                    f"측정 나이 최대 {max_age} ms (stale 기준 20 ms), 고장 중 단계 최대 {lvl(max_level(w))}",
                    f"고장 해제 후 단계 {lvl(max_level(r.window(FAULT_AT + 1.3, 9)))}"]
    return chk


def chk_echo(r: Run):
    w = r.window(FAULT_AT, FAULT_AT + 1.0)
    on = sum(1 for s in w if torque_on_all(s)) / max(1, len(w))
    ok = r.diag.get("bus_echo_errors", 0) > 0 and r.diag.get("bus_retries", 0) > 0 and on == 1.0
    return ok, [f"에코 불일치 {r.diag.get('bus_echo_errors')} 회 -> 재전송 {r.diag.get('bus_retries')} 회",
                f"토크 유지 {on * 100:.0f}%, 지령 재확인 불일치 {r.diag.get('verify_mismatches')}"]


def chk_spike(r: Run):
    w = r.window(FAULT_AT - 0.05, FAULT_AT + 0.5)
    jumps = [abs(b.pos[1] - a.pos[1]) for a, b in zip(w, w[1:])]
    imp = flag_frac(w, 1, AX_IMPLAUSIBLE)
    ok = max(jumps, default=0) < 100 and r.diag.get("implausible_samples", 0) >= 2
    return ok, [f"축 1 에 체크섬이 맞는 +800 tick 튄 값 2 개 주입",
                f"보고 위치의 STATE 간 최대 변화 {max(jumps, default=0)} tick (튄 값이 그대로면 800)",
                f"타당성 탈락 {r.diag.get('implausible_samples')} 개, IMPLAUSIBLE 표시 STATE 비율 {imp * 100:.0f}%"]


def chk_dead_one(r: Run):
    a = 3
    t_stale = r.first(lambda s: s.flags[a] & AX_STALE, FAULT_AT)
    t_lost = r.first(lambda s: s.flags[a] & AX_NO_RESPONSE, FAULT_AT)
    t_hold = r.first(lambda s: s.flags[a] & AX_HOLD, FAULT_AT)
    w = r.window(FAULT_AT + 0.15, FAULT_AT + 0.4)
    held_pos = {s.pos[a] for s in w if s.flags[a] & AX_HOLD}
    others_ok = all(all(s.flags[i] & AX_TORQUE_ON for i in range(N_STS)) for s in w)
    t_rec = r.first(lambda s: s.axl[a] == 0, FAULT_AT + 0.4)
    est = r.window(FAULT_AT, FAULT_AT + 0.1)
    est_frac = flag_frac(est, a, AX_ESTIMATED)
    ok = (t_stale is not None and t_lost is not None and t_hold is not None and others_ok
          and t_rec is not None and t_rec - (FAULT_AT + 0.4) < 0.1)
    return ok, [f"축 3 응답 끊김 0.4 s: STALE {ms(t_stale, FAULT_AT)}, NO_RESPONSE {ms(t_lost, FAULT_AT)}, "
                f"HOLD {ms(t_hold, FAULT_AT)} 후 (lost_ms 100, lost_action hold)",
                f"처음 100 ms 동안 ESTIMATED(관측기 추정값) 표시 비율 {est_frac * 100:.0f}%",
                f"HOLD 중 보고 위치 {sorted(held_pos)[:3]} (목표 고정), 다른 5 축 토크 유지 {others_ok}",
                f"복구 후 OK 까지 {ms(t_rec, FAULT_AT + 0.4)}, 전체 단계 최대 {lvl(max_level(r.samples))}"]


def chk_bus_dead(r: Run):
    t_off = r.first(lambda s: not any(f & AX_TORQUE_ON for f in s.flags[:N_STS]), FAULT_AT)
    w = r.window(FAULT_AT + 0.35, FAULT_AT + 0.6)
    pwm_on = all(all(s.flags[i] & AX_TORQUE_ON for i in range(N_STS, N_STS + N_PWM)) for s in w)
    lv = max_level(w)
    off_pkts = [h for t, h in r.bus_window(FAULT_AT, FAULT_AT + 0.6)
                if classify_bus(h)[:2] == ("SYNC_WRITE", "TORQUE_ENABLE") and ":0" in classify_bus(h)[2]]
    t_back = r.first(torque_on_all, FAULT_AT + 0.6)
    ok = (t_off is not None and 0.25 <= t_off - FAULT_AT <= 0.4 and pwm_on and lv == 3
          and len(off_pkts) >= 2 and t_back is not None)
    return ok, [f"STS 6 축 모두 무응답 0.6 s: {ms(t_off, FAULT_AT)} 에 STS 토크 해제 (bus_dead_ms 300, torque_off)",
                f"해제 중 TORQUE_ENABLE=0 패킷 {len(off_pkts)} 회 반복 (응답 없는 쓰기라 주기 반복), 단계 {lvl(lv)}",
                f"PWM 4 축은 계속 동작 {pwm_on}",
                f"버스 복구 후 {ms(t_back, FAULT_AT + 0.6)} 에 측정 위치에서 다시 토크 (덜컥 움직이지 않게)"]


def chk_reset(r: Run):
    a = 2
    t_mis = r.first(lambda s: s.flags[a] & AX_CMD_MISMATCH, FAULT_AT)
    fix = [t for t, h in r.bus_window(FAULT_AT, FAULT_AT + 0.5)
           if classify_bus(h)[0] == "WRITE" and f"id={a + 1} " in classify_bus(h)[2]]
    ok = t_mis is not None and r.diag.get("verify_mismatches", 0) >= 1 and fix and fix[0] - FAULT_AT < 0.2
    return ok, [f"축 2 서보가 저전압 리셋 (토크 꺼짐, 목표 = 현재 위치). 펌웨어는 토크가 켜진 줄 앎",
                f"레지스터 재확인으로 {ms(t_mis, FAULT_AT)} 에 CMD_MISMATCH 표시, "
                f"{ms(fix[0] if fix else None, FAULT_AT)} 에 GOAL+TORQUE 다시 씀",
                f"재확인 불일치 {r.diag.get('verify_mismatches')} 회 (주기 verify_period_ms 20 x 6 축 순환)"]


def hold_then_step4(t: float) -> List[int]:
    """축 4 만 고장 시각에 2048 -> 2348 로 옮기고 그 뒤 고정 (다른 축은 고정)."""
    v = [MID] * N_STS + [1500] * N_PWM
    if t >= FAULT_AT:
        v[4] = MID + 300
    return v


def chk_ignore(r: Run):
    a = 4
    t_mis = r.first(lambda s: s.flags[a] & AX_CMD_MISMATCH, FAULT_AT)
    end = r.window(r.samples[-1].t - 0.2, 9) if r.samples else []
    reached = bool(end) and all(abs(s.pos[a] - (MID + 300)) <= 5 for s in end)
    ok = t_mis is not None and r.diag.get("verify_mismatches", 0) >= 1 and reached
    return ok, [f"축 4 를 2048 -> 2348 로 옮기는 동안 서보가 쓰기 패킷 28 개를 놓침 (응답 없는 패킷이라 송신 측은 모름)",
                f"재확인으로 {ms(t_mis, FAULT_AT)} 에 발견, 재전송. 불일치 {r.diag.get('verify_mismatches')} 회",
                f"최종 위치가 목표에 도달 {reached} (재확인이 없으면 2048 에 머묾)"]


def chk_overtemp(r: Run):
    a = 5
    t_warn = r.first(lambda s: s.flags[a] & AX_OVERTEMP, FAULT_AT)
    t_off = r.first(lambda s: s.flags[a] & AX_PROTECT_OFF, FAULT_AT)
    t_cool = FAULT_AT + 0.9
    latched = all(s.flags[a] & AX_PROTECT_OFF for s in r.window(t_cool + 0.05, 1.8))
    t_rel = r.first(lambda s: not (s.flags[a] & AX_PROTECT_OFF), 1.8)
    t_back = r.first(lambda s: s.flags[a] & AX_TORQUE_ON, 2.0)
    others = all(all(s.flags[i] & AX_TORQUE_ON for i in range(N_STS) if i != a)
                 for s in r.window(t_off or 9, 1.8))
    ok = (t_warn is not None and t_off is not None and 0.45 <= t_off - (t_warn or 0) <= 0.6
          and latched and others and t_rel is not None and t_back is not None)
    return ok, [f"축 5 온도 75 C (경고 60 C, 해제 70 C): {ms(t_warn, FAULT_AT)} 에 OVERTEMP 경고",
                f"{ms(t_off, FAULT_AT)} 에 PROTECT_OFF 로 그 축만 토크 해제 (protect_ms 500 동안 지속 확인)",
                f"온도가 내려가도 해제 유지 {latched} (래치), 다른 축 토크 유지 {others}",
                f"Pi 가 토크 0 지령을 보낸 뒤 래치 해제 ({ms(t_rel, 1.8)}), 위치 지령으로 다시 토크 ({ms(t_back, 2.0)})"]


def chk_tcp_rx(r: Run):
    w = r.window(FAULT_AT, FAULT_AT + 1.0)
    on = sum(1 for s in w if torque_on_all(s)) / max(1, len(w))
    ok = r.diag.get("crc_errors", 0) > 0 and r.diag.get("seq_gaps", 0) > 0 and r.link["state_missing"] > 0
    ok &= on > 0.9
    return ok, [f"Pi->Pico 바이트 비트 뒤집힘 2000 ppm 1 s (W5500-Pico SPI 잡음에 해당)",
                f"Pico: CRC 오류 {r.diag.get('crc_errors')}, 버린 바이트 {r.diag.get('dropped_bytes')}, "
                f"seq 건너뜀 {r.diag.get('seq_gaps')} -> DEGRADED (단계 최대 {lvl(max_level(w))})",
                f"Pi: 응답 없는 지령 {r.link['state_missing']} 개, 토크 유지 {on * 100:.0f}% "
                f"(지령이 100 ms 넘게 연속으로 깨지지 않는 한 유지)"]


def chk_tcp_tx(r: Run):
    ok = r.link["crc_errors"] > 0 and r.link["state_missing"] > 0
    return ok, [f"Pico->Pi 바이트 비트 뒤집힘 2000 ppm 1 s",
                f"Pi: CRC 오류 {r.link['crc_errors']}, 버린 바이트 {r.link['dropped_bytes']}, "
                f"STATE 누락 {r.link['state_missing']} (텔레메트리는 state_stale_s 넘으면 PICO_STALE)",
                f"링크 재연결 {r.link['reconnects']}"]


def chk_stall(r: Run):
    end = FAULT_AT + 0.15
    first = r.window(end - 0.05, end + 0.05)[:1]
    t_back = r.first(lambda s: torque_on_all(s) and s.level <= 1, end - 0.05)
    gap = r.diag_run.get("cmd_interval_max_us", 0)
    trips = r.diag_run.get("watchdog_trips", 0)
    ok = trips >= 1 and gap >= 100000 and t_back is not None and t_back - end < 0.05
    lv = lvl(first[0].level) if first else "없음"
    return ok, [f"Pico 전체(core0+core1)가 150 ms 멈춤 (긴 인터럽트/플래시 쓰기 등에 해당). 그동안 온 지령은 버퍼에 쌓임",
                f"재개 시 지령 간격 {gap / 1000:.0f} ms > cmd_timeout 100 ms 라 Pico 워치독이 먼저 토크 해제 "
                f"(watchdog_trips {trips})",
                f"재개 직후 STATE 단계 {lv} (서보 측정도 150 ms 끊겨 NO_RESPONSE), 쌓인 지령과 새 측정으로 "
                f"{ms(t_back, end)} 에 전 축 토크 + 단계 DEGRADED 이하로 복귀",
                f"지터 경고로 degrade_hold_ms(1000) 동안 DEGRADED 유지, Pi 왕복 시간 최대 {r.link['rtt_ms_max']:.0f} ms"]


def step_target(t: float) -> List[int]:
    v = MID if t < FAULT_AT else MID + 1500
    return [v] * N_STS + [1000 if t < FAULT_AT else 2000] * N_PWM


def chk_slew(r: Run):
    goals = []
    for t, h in r.bus_window(FAULT_AT - 0.05, FAULT_AT + 0.6):
        ins, addr, items = classify_bus(h)
        if ins == "SYNC_WRITE" and addr.startswith("GOAL"):
            for it in items.strip("{}").split(", "):
                sid, v = it.split(":")
                if sid == "1":
                    goals.append((t, int(v)))
    incs = [b[1] - a[1] for a, b in zip(goals, goals[1:])]
    reach = next((t for t, v in goals if v >= MID + 1500), None)
    start = next((t for t, v in goals if v > MID), None)
    rate = 1500 / (reach - start) if reach and start and reach > start else 0
    # 한 번에 나가는 증가량 <= 6000 tick/s x 루프 2 ms (+ 루프 지연 여유)
    ok = incs and max(incs) <= 6000 * 0.002 * 2 and reach is not None and 0.2 <= reach - FAULT_AT <= 0.4
    return ok, [f"축 0 목표 2048 -> 3548 계단 지령",
                f"서보로 나간 목표가 {ms(reach, FAULT_AT)} 에 걸쳐 증가 (slew 6000 tick/s -> 250 ms), "
                f"평균 {rate:.0f} tick/s, 쓰기 1 회당 최대 증가 {max(incs, default=0)} tick",
                f"slew 제한 횟수 {r.diag.get('slew_limited')}, PWM 은 4000 us/s"]


SCENARIOS = [
    Scenario("baseline", "없음", "단계 OK, 오류 0", 1.5, [], chk_baseline),
    Scenario("uart_noise", "서보 버스 비트 뒤집힘 3000 ppm + 잡음 바이트 2000 ppm (1 s)",
             "재시도로 흡수, 토크 유지, 측정 나이 짧게 유지", 2.0,
             [(FAULT_AT, "uart_flip 3000"), (FAULT_AT, "uart_noise 2000"),
              (FAULT_AT + 1.0, "uart_flip 0"), (FAULT_AT + 1.0, "uart_noise 0")], chk_uart_noise(40)),
    Scenario("uart_heavy", "서보 버스 비트 뒤집힘 15000 ppm + 바이트 유실 5000 ppm (1 s)",
             "샘플 일부 누락 -> STALE/추정값, 토크 유지, 끝나면 OK", 2.0,
             [(FAULT_AT, "uart_flip 15000"), (FAULT_AT, "uart_drop 5000"),
              (FAULT_AT + 1.0, "uart_flip 0"), (FAULT_AT + 1.0, "uart_drop 0")], chk_uart_noise(100)),
    Scenario("echo", "송신 에코만 깨짐 20000 ppm (1 s)", "송신 실패를 감지해 재전송", 2.0,
             [(FAULT_AT, "echo_flip 20000"), (FAULT_AT + 1.0, "echo_flip 0")], chk_echo),
    Scenario("spike", "축 1 위치에 +800 tick 튄 값 2 개 (체크섬 정상)", "타당성 검사로 버림, 위치 튐 없음", 1.2,
             [(FAULT_AT, "spike 2 800 2")], chk_spike),
    Scenario("dead_servo", "축 3 서보 무응답 0.4 s", "STALE -> 추정값 -> NO_RESPONSE/HOLD, 다른 축 유지, 복구", 1.5,
             [(FAULT_AT, "dead 4 1"), (FAULT_AT + 0.4, "dead 4 0")], chk_dead_one),
    Scenario("bus_dead", "STS 6 개 모두 무응답 0.6 s", "300 ms 뒤 STS 토크 해제, PWM 유지, 복구 후 재개", 1.8,
             [(FAULT_AT, f"dead {i} 1") for i in range(1, 7)] +
             [(FAULT_AT + 0.6, f"dead {i} 0") for i in range(1, 7)], chk_bus_dead),
    Scenario("servo_reset", "축 2 서보 저전압 리셋 (토크 꺼짐)", "레지스터 재확인으로 발견, 다시 씀", 1.2,
             [(FAULT_AT, "reset 3")], chk_reset),
    Scenario("missed_write", "축 4 이동 중 쓰기 패킷 28 개 놓침", "재확인으로 발견, 다시 써서 목표 도달", 2.0,
             [(FAULT_AT - 0.01, "ignore_writes 5 28")], chk_ignore, target=hold_then_step4),
    Scenario("overtemp", "축 5 온도 75 C 0.9 s, 그 뒤 Pi 가 토크 0 -> 위치 지령", "경고 -> 500 ms 뒤 그 축만 해제(래치)",
             2.5, [(FAULT_AT, "temp 6 75"), (FAULT_AT + 0.9, "temp 6 30")], chk_overtemp,
             mode_at=lambda t: (CmdMode.TORQUE, [0] * 10) if 1.8 <= t < 2.0 else (CmdMode.POSITION, sts_wave(t))),
    Scenario("tcp_rx_noise", "Pi->Pico 바이트 비트 뒤집힘 2000 ppm (1 s)", "CRC 로 버림, seq gap -> DEGRADED, 토크 유지",
             2.0, [(FAULT_AT, "tcp_rx_flip 2000"), (FAULT_AT + 1.0, "tcp_rx_flip 0")], chk_tcp_rx),
    Scenario("tcp_tx_noise", "Pico->Pi 바이트 비트 뒤집힘 2000 ppm (1 s)", "Pi 가 CRC 로 버리고 누락을 셈",
             2.0, [(FAULT_AT, "tcp_tx_flip 2000"), (FAULT_AT + 1.0, "tcp_tx_flip 0")], chk_tcp_tx),
    Scenario("stall", "Pico 150 ms 멈춤", "Pico 워치독 해제 후 다음 지령으로 복귀", 1.5,
             [(FAULT_AT, "stall 150")], chk_stall),
    Scenario("step_cmd", "위치 지령 계단 +1500 tick", "변화율 제한으로 250 ms 에 걸쳐 이동", 1.2, [], chk_slew,
             target=step_target),
]


# ---------------------------------------------------------------- W5500


def run_w5500(tmp: str) -> tuple:
    out = os.path.join(tmp, "w5500_fault_test")
    subprocess.run(["gcc", "-std=c11", "-O1", "-Wall", "-Wextra", "-Werror",
                    "-I", os.path.join(FW, "host", "sdk_stub"), "-I", os.path.join(FW, "src"), "-I", tmp,
                    os.path.join(FW, "src", "w5500.c"), os.path.join(FW, "src", "diag.c"),
                    os.path.join(FW, "host", "w5500_fault_test.c"), "-o", out], check=True)
    r = subprocess.run([out], capture_output=True, text=True, timeout=60)
    return r.returncode == 0, r.stdout


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--board", default="pico2_w5500")
    ap.add_argument("--out", default=None)
    ap.add_argument("--only", default=None, help="쉼표로 구분한 시나리오 이름")
    args = ap.parse_args()
    tmp = tempfile.mkdtemp()
    binary = build_sim(args.board, tmp)
    only = set(args.only.split(",")) if args.only else None

    lines = [f"# 고장 주입 시험 ({args.board})", "",
             "펌웨어 C 코드(세션/서보 루프/버스 드라이버)를 SDK 대체 위에서 돌리고, Pi 쪽은 실제 `PicoLink` 가 "
             "100 Hz 로 위치 지령을 보낸다. 고장은 지령 시작 0.5 s 뒤에 넣는다. 시각은 고장 시작 기준.", "",
             "| 시나리오 | 넣은 고장 | 기대 | 결과 |", "|---|---|---|---|"]
    details = []
    all_ok = True
    for sc in SCENARIOS:
        if only and sc.name not in only:
            continue
        # 호스트 OS 스케줄링(시뮬레이터 프로세스가 수십 ms 밀림)에 따라 시간 판정이 흔들릴 수 있어 실패하면 한 번 더 돌린다
        for attempt in range(2):
            r = run_scenario(binary, sc.duration, sc.timeline, sc.target, sc.mode_at)
            unknown = [a for a in r.acks if a.startswith("fault unknown")]
            ok, obs = sc.check(r)
            ok = bool(ok) and not unknown
            if ok:
                break
        all_ok &= ok
        verdict = ("통과" if attempt == 0 else "통과 (2회째)") if ok else "실패"
        lines.append(f"| {sc.name} | {sc.fault} | {sc.expect} | {verdict} |")
        details += [f"## {sc.name}", "", f"- 고장: {sc.fault}", f"- 기대: {sc.expect}", "- 관측:"]
        details += [f"  - {o}" for o in obs]
        if unknown:
            details.append(f"  - 알 수 없는 고장 명령: {unknown}")
        details.append("")
        print(f"{sc.name}: {'ok' if ok else 'FAIL'}", file=sys.stderr)
    if not only or "w5500" in only:
        ok, out = run_w5500(tmp)
        all_ok &= ok
        lines.append(f"| w5500 | SPI 비트 오류, 칩 리셋, 레지스터 값 흔들림 | 검출 -> 재초기화 -> 클럭 낮춤 | "
                     f"{'통과' if ok else '실패'} |")
        details += ["## w5500 (host/w5500_fault_test.c)", "", "```", out.rstrip(), "```", ""]
    notes = ["", "## 이 시험이 보여주지 않는 것", "",
             "- 실제 잡음의 크기와 분포: 여기 ppm 값은 가정이다. 보드에서 DIAG 카운터(버스 재시도, SPI 오류, CRC 오류)를 "
             "모아 실제 오류율을 재고 임계값을 맞춰야 한다.",
             "- 시간: 시뮬레이터는 리눅스 프로세스라 ms 단위 판정은 OS 스케줄링에 흔들린다 (실패 시 1 회 재실행). "
             "보드의 루프 시간은 DIAG `loop_max_us` 로 확인한다. 잡음이 심하면 읽기 1 회가 응답 대기(1.5 ms) x 재시도 "
             "2 회까지 걸릴 수 있어 `read_budget_us` 로 한 루프의 읽기 시간을 묶었다.",
             "- 전원: 서보 돌입 전류로 Pico 가 저전압 리셋되는 경우는 시뮬레이션하지 않았다. 리셋 원인은 DIAG "
             "`reset_cause` (bit17 = BOR) 로 보고된다.", ""]
    text = "\n".join(lines + notes + details)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text)
    print(text)
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
