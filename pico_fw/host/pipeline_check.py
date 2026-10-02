#!/usr/bin/env python3
"""가상 입력 -> Pi 5 comm_core -> (TCP) -> 펌웨어 출력 확인.

하드웨어 없이 전체 경로를 돌린다.
  - Pi 쪽: 실제 comm_core.CommCore (100 Hz 루프, 팀 UDP 포트)
  - 펌웨어: proto.c / session.c / actuators.c / sts_bus.c 를 host/hal_stub.c (Pico SDK 대체) 위에서 컴파일한
    시뮬레이터. 서보 버스로 나가는 UART 바이트, PWM 레벨, Pi 로 보내는 TCP 프레임을 모두 기록한다.
  - 가상 입력: 3-B 지령(CH2 :5556, torque/position/raw_position/잘못된 JSON), 4팀 슬립(CH3), 1팀 VLA(CH4),
    지령 중단, Pi 루프 정지, Pi 종료

실행 (저장소 루트에서): python3 pico_fw/host/pipeline_check.py [--board pico2_w5500] [--out report.md]
"""

from __future__ import annotations

import argparse
import collections
import dataclasses
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from comm_core.config import load_config  # noqa: E402
from comm_core.core import CommCore  # noqa: E402
from comm_core.protocol import FrameDecoder, MsgType, State  # noqa: E402

FW = os.path.join(ROOT, "pico_fw")
MSG_NAMES = {int(m): m.name for m in MsgType}
STS_INST = {0x02: "READ", 0x03: "WRITE", 0x83: "SYNC_WRITE"}
STS_ADDR = {40: "TORQUE_ENABLE", 41: "ACC", 42: "GOAL_POSITION+TIME+SPEED", 56: "PRESENT_POS..TEMP"}


def build_sim(board: str, tmp: str) -> str:
    subprocess.run([sys.executable, os.path.join(FW, "tools", "gen_config.py"),
                    "--config", os.path.join(FW, "config", "controller.yaml"),
                    "--board", os.path.join(FW, "config", "boards", board + ".yaml"), "--out", tmp], check=True)
    out = os.path.join(tmp, "fw_sim_real")
    src = ["proto.c", "session.c", "actuators.c", "sts_bus.c", "diag.c"]
    subprocess.run(["gcc", "-std=c11", "-O1", "-DFW_SIM_REAL_ACTUATORS",
                    "-I", os.path.join(FW, "host", "sdk_stub"), "-I", os.path.join(FW, "src"), "-I", tmp,
                    *[os.path.join(FW, "src", s) for s in src],
                    os.path.join(FW, "host", "hal_stub.c"), os.path.join(FW, "host", "sim_main.c"), "-o", out],
                   check=True)
    return out


# ---------------------------------------------------------------- 출력 분류


def classify_bus(hexstr: str) -> tuple:
    p = bytes.fromhex(hexstr)
    if len(p) < 6 or p[:2] != b"\xff\xff":
        return ("BAD", "", "")
    ident, ins = p[2], p[4]
    name = STS_INST.get(ins, f"0x{ins:02x}")
    if ins == 0x83:
        addr, dl = p[5], p[6]
        items = []
        for o in range(7, len(p) - 1, dl + 1):
            sid, d = p[o], p[o + 1:o + 1 + dl]
            if addr == 42:
                items.append(f"{sid}:{int.from_bytes(d[:2], 'little')}")
            else:
                items.append(f"{sid}:{d[0]}")
        return (name, STS_ADDR.get(addr, str(addr)), "{" + ", ".join(items) + "}")
    if ins == 0x02:
        return (name, STS_ADDR.get(p[5], str(p[5])), f"id={ident} len={p[6]}")
    return (name, STS_ADDR.get(p[5], str(p[5])), f"id={ident} data={p[6:-1].hex()}")


class SimLog:
    def __init__(self, proc: subprocess.Popen) -> None:
        self.events = []  # (ms, kind, payload)
        self.proc = proc
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        for line in self.proc.stdout:
            parts = line.split()
            if len(parts) >= 3 and parts[0] in ("tcp", "bus"):
                self.events.append((int(parts[1]), parts[0], parts[2]))
            elif len(parts) == 4 and parts[0] == "pwm":
                self.events.append((int(parts[1]), "pwm", (int(parts[2]), int(parts[3]))))

    def window(self, t0: float, t1: float):
        return [e for e in self.events if int(t0 * 1000) <= e[0] < int(t1 * 1000)]


def summarize(events) -> dict:
    tcp = collections.Counter()
    tcp_samples = {}
    bus = collections.Counter()
    bus_samples = {}
    pwm = []
    dec = FrameDecoder()
    for ms, kind, payload in events:
        if kind == "tcp":
            for f in dec.feed(bytes.fromhex(payload)):
                name = MSG_NAMES.get(f.msg_type, hex(f.msg_type))
                tcp[name] += 1
                if f.msg_type == MsgType.STATE:
                    st = State.unpack(f.payload)
                    tcp_samples[name] = (f"status=0x{st.status:04x} error={st.error} "
                                         f"pos={[a.position for a in st.actuators]} "
                                         f"flags={[hex(a.flags) for a in st.actuators]}")
                elif f.msg_type == MsgType.ERROR:
                    code = int.from_bytes(f.payload[:2], "little")
                    tcp_samples.setdefault(name, set()).add(f"{code}: {f.payload[2:].decode()}")
                elif f.msg_type == MsgType.HELLO_ACK:
                    tcp_samples[name] = f.payload.hex()
        elif kind == "bus":
            key = classify_bus(payload)[:2]
            bus[key] += 1
            bus_samples[key] = classify_bus(payload)[2]
        else:
            pwm.append((ms, *payload))
    return {"tcp": tcp, "tcp_samples": tcp_samples, "bus": bus, "bus_samples": bus_samples, "pwm": pwm}


def release_latency(events):
    """창 안에서 마지막 지령 응답(STATE) -> 그 뒤 첫 토크 해제 패킷까지 걸린 시간 (ms).
    (변화율 제한 때문에 마지막 GOAL 쓰기는 마지막 지령보다 이를 수 있어 지령 응답을 기준으로 잰다)"""
    last_cmd = None
    dec = FrameDecoder()
    for ms, kind, payload in events:
        if kind == "tcp":
            if any(f.msg_type == MsgType.STATE for f in dec.feed(bytes.fromhex(payload))):
                last_cmd = ms
        elif kind == "bus" and last_cmd is not None:
            ins, addr, items = classify_bus(payload)
            if ins == "SYNC_WRITE" and addr == "TORQUE_ENABLE" and ":0" in items:
                return ms - last_cmd
    return None


# ---------------------------------------------------------------- 시나리오


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--board", default="pico2_w5500")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    tmp = tempfile.mkdtemp()
    sim_bin = build_sim(args.board, tmp)
    proc = subprocess.Popen([sim_bin, "0"], stdout=subprocess.PIPE, text=True)
    port = int(proc.stdout.readline().split()[1])
    log = SimLog(proc)
    t_boot = time.monotonic()

    sinks = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(4)]
    for s in sinks:
        s.bind(("127.0.0.1", 0))
        s.setblocking(False)
    base = load_config(os.path.join(ROOT, "config", "comm_core.yaml"))
    cfg = dataclasses.replace(
        base,
        pico=dataclasses.replace(base.pico, host="127.0.0.1", port=port),
        channels=dataclasses.replace(
            base.channels, bind_host="127.0.0.1", telemetry_port=0, command_port=0, slip_port=0, vla_port=0,
            telemetry_dest=sinks[0].getsockname(), proprio_dest=sinks[1].getsockname(),
            slip_relay_dest=sinks[2].getsockname(), vla_action_relay_dest=sinks[3].getsockname()),
    )
    core = CommCore(cfg)
    core.start()
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(ep, msg):
        tx.sendto(msg if isinstance(msg, bytes) else json.dumps(msg).encode(), ep.address)

    def run(duration, every=None, step=True):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if every:
                every()
            if step:
                core.step(time.monotonic())
            time.sleep(0.01)

    phases = []

    def phase(name, desc, fn):
        t0 = time.monotonic()
        fn()
        phases.append((name, desc, t0, time.monotonic()))

    phase("0 부팅", "펌웨어 시작 (Pi 연결 전)", lambda: time.sleep(0.15))
    phase("1 연결, 지령 없음", "Pi 연결 직후 CH2 지령 없음 -> Pi 워치독이 토크 0 지령 송신", lambda: run(0.3))
    q = [0.5, 0.4, 0.3, 0.2, 0.1, -0.1, 0.0, 0.3, 0.6, -0.3]
    phase("2 position", f"CH2 {{mode: position, positions: {q}}} 100 Hz",
          lambda: run(0.3, lambda: send(core.cmd_rx, {"mode": "position", "positions": q})))
    raw = [100, 4095, 2048, 3000, 1000, 2500, 500, 2500, 1500, 9999]
    phase("3 raw_position", f"CH2 {{mode: raw_position, raw: {raw}}} (9999 는 범위 밖)",
          lambda: run(0.3, lambda: send(core.cmd_rx, {"mode": "raw_position", "raw": raw})))
    phase("4 torque != 0", "CH2 {mode: torque, torques: [1.0]*10}",
          lambda: run(0.3, lambda: send(core.cmd_rx, {"mode": "torque", "torques": [1.0] * 10})))
    phase("5 position 복귀", "CH2 position [0]*10",
          lambda: run(0.2, lambda: send(core.cmd_rx, {"mode": "position", "positions": [0.0] * 10})))
    phase("6 torque = 0", "CH2 {mode: torque, torques: [0]*10}",
          lambda: run(0.2, lambda: send(core.cmd_rx, {"mode": "torque", "torques": [0.0] * 10})))
    phase("7 잘못된 입력 + 슬립/VLA", "CH2 깨진 JSON/길이 오류 + CH3 슬립 + CH4 VLA 액션 (지령 없음)",
          lambda: run(0.3, lambda: (send(core.cmd_rx, b"not json"),
                                    send(core.cmd_rx, {"mode": "torque", "torques": [1.0] * 3}),
                                    send(core.slip_rx, {"timestamp": 1.0, "slip_detected": [True] * 5}),
                                    send(core.vla, {"task": "pick", "synergy_mode": "pinch"}))))
    phase("8 position 후 CH2 중단", "position 0.3 s 보낸 뒤 CH2 끊김 (Pi 는 계속 동작)",
          lambda: (run(0.2, lambda: send(core.cmd_rx, {"mode": "position", "positions": [0.2] * 10})), run(0.3)))
    phase("9 position 후 Pi 루프 정지", "position 뒤 Pi 100 Hz 루프가 멈춤 (하트비트만 유지) -> Pico 자체 워치독",
          lambda: (run(0.2, lambda: send(core.cmd_rx, {"mode": "position", "positions": [0.2] * 10})),
                   run(0.3, step=False)))
    phase("10 position 중 Pi 종료", "position 지령 중 Pi comm_core 종료 (ESTOP 송신 후 연결 끊김)",
          lambda: (run(0.2, lambda: send(core.cmd_rx, {"mode": "position", "positions": [0.2] * 10})),
                   core.close(), time.sleep(0.3)))

    time.sleep(0.1)
    proc.kill()
    proc.wait()

    lines = [f"# 펌웨어 출력 확인 ({args.board})", "",
             "방법: 하드웨어 없이 실행. Pi 쪽은 실제 `comm_core.CommCore`, 펌웨어는 `proto.c` `session.c` `actuators.c` "
             "`sts_bus.c` 를 `host/hal_stub.c` (Pico SDK 대체, 가짜 STS3215 6개) 위에서 컴파일한 시뮬레이터. "
             "core1 루프는 같은 주기(2 ms)로 한 단계씩 실행.", ""]
    total = summarize(log.events)
    lines += ["## 출력 종류 (전체)", "", "| 출력 경로 | 종류 | 횟수 |", "|---|---|---|"]
    for k, v in sorted(total["tcp"].items()):
        lines.append(f"| TCP -> Pi | {k} | {v} |")
    for (ins, addr), v in sorted(total["bus"].items()):
        lines.append(f"| 서보 버스 (UART) | {ins} {addr} | {v} |")
    pins = sorted({p for _, p, _ in total["pwm"]})
    lines.append(f"| PWM (GPIO {', '.join(map(str, pins))}) | 펄스 폭 변경 | {len(total['pwm'])} |")
    lines.append("")
    for name, desc, t0, t1 in phases:
        s = summarize(log.window(t0, t1))
        lines += [f"## {name}", "", f"입력: {desc}", ""]
        if s["tcp"]:
            lines.append("- TCP -> Pi: " + ", ".join(f"{k} x{v}" for k, v in sorted(s["tcp"].items())))
            for k, v in sorted(s["tcp_samples"].items()):
                lines.append(f"  - 마지막 {k}: `{sorted(v) if isinstance(v, set) else v}`")
        else:
            lines.append("- TCP -> Pi: 없음")
        if s["bus"]:
            lines.append("- 서보 버스: " + ", ".join(f"{i} {a} x{v}" for (i, a), v in sorted(s["bus"].items())))
            for (i, a), v in sorted(s["bus_samples"].items()):
                if i == "SYNC_WRITE":
                    lines.append(f"  - 마지막 {i} {a}: `{v}`")
        else:
            lines.append("- 서보 버스: 없음")
        lat = release_latency(log.window(t0, t1))
        if lat is not None:
            lines.append(f"- 토크 해제: 마지막 지령 응답(STATE) 후 {lat} ms 에 TORQUE_ENABLE=0 + PWM 0 us")
        if s["pwm"]:
            lines.append("- PWM: " + ", ".join(f"GPIO{p}={us}us" for _, p, us in s["pwm"]))
        else:
            lines.append("- PWM: 변화 없음")
        lines.append("")
    text = "\n".join(lines)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text)
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
