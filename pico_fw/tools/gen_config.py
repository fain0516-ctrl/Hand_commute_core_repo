#!/usr/bin/env python3
"""펌웨어 설정 생성기: YAML -> fw_config.h (+ board.cmake).

하드코딩을 없애기 위해 펌웨어 C 코드에는 핀 번호나 파라미터 기본값을 두지 않는다.
  - pico_fw/config/controller.yaml     : 네트워크, 서보 버스 타이밍, 안전 파라미터
  - pico_fw/config/boards/<보드>.yaml   : 핀 배치 (보드마다 1개)
  - config/comm_core.yaml (Pi 쪽)       : Pico IP/포트, Pico 워치독 시간 (같은 값을 두 번 쓰지 않도록 그대로 읽는다)
  - config/hand_model.yaml (Pi 쪽)      : 액추에이터 순서, 종류, 버스 ID, 원시값 범위

누락된 키나 모르는 키는 오류다. 핀은 RP2350 의 기능 표(SPI/UART)와 맞는지, 중복이 없는지 검사한다.
보드 YAML 의 `inferred` 목록에 있는 항목은 PCB 에서 읽은 값이 아니라 추정값이므로 빌드 때 경고로 다시 알려준다.

사용: python3 gen_config.py --config controller.yaml --board boards/pico2_w5500.yaml --out build/generated
"""

from __future__ import annotations

import argparse
import ipaddress
import os
import sys
from typing import Any, Dict, List, Optional

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.exit("gen_config: PyYAML 이 필요합니다 (sudo apt install python3-yaml 또는 pip install pyyaml)")


class ConfigError(Exception):
    pass


# ---------------------------------------------------------------- 스키마 도우미


def take(d: Any, path: str, keys: Dict[str, Any]) -> Dict[str, Any]:
    """keys = {이름: 타입 또는 (타입들)}. 누락/모르는 키는 오류."""
    if not isinstance(d, dict):
        raise ConfigError(f"{path}: 매핑이어야 합니다")
    missing = [k for k in keys if k not in d]
    unknown = [k for k in d if k not in keys]
    if missing:
        raise ConfigError(f"{path}: 키 누락 {missing}")
    if unknown:
        raise ConfigError(f"{path}: 모르는 키 {unknown}")
    for k, t in keys.items():
        if t is None:
            continue
        v = d[k]
        if t is int and isinstance(v, bool):
            raise ConfigError(f"{path}.{k}: 정수여야 합니다")
        if not isinstance(v, t):
            raise ConfigError(f"{path}.{k}: 타입 {t} 이어야 합니다 (현재 {v!r})")
    return d


def rng(path: str, v: int, lo: int, hi: int) -> int:
    if not lo <= v <= hi:
        raise ConfigError(f"{path}: {v} 는 {lo}~{hi} 범위여야 합니다")
    return v


PIN = (int, type(None))


# ---------------------------------------------------------------- RP2350 핀 기능 표


CHIP_GPIO = {"rp2350a": 30, "rp2350b": 48}


def spi_of(pin: int) -> tuple:
    """(SPI 번호, 역할). 역할 0=RX(MISO) 1=CSn 2=SCK 3=TX(MOSI). RP2350 데이터시트 GPIO 기능 표."""
    return (pin >> 3) & 1, pin & 3


def uart_of(pin: int) -> tuple:
    """(UART 번호, 역할). 역할 0=TX 1=RX 2=CTS 3=RTS (기본 UART 기능, F2)."""
    return ((pin >> 2) ^ (pin >> 3)) & 1, pin & 3


class PinBook:
    def __init__(self, chip: str) -> None:
        self.n = CHIP_GPIO[chip]
        self.used: Dict[int, str] = {}

    def use(self, path: str, pin: Optional[int], required: bool) -> int:
        if pin is None:
            if required:
                raise ConfigError(f"{path}: 필수 핀이 비어 있습니다 (null)")
            return -1
        rng(path, pin, 0, self.n - 1)
        if pin in self.used:
            raise ConfigError(f"{path}: GPIO{pin} 이 이미 {self.used[pin]} 에 쓰였습니다")
        self.used[pin] = path
        return pin


# ---------------------------------------------------------------- 로드


def load_yaml(path: str) -> Any:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def resolve(base_file: str, rel: str) -> str:
    return os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(base_file)), rel))


def parse_ip(path: str, s: str) -> List[int]:
    try:
        return list(ipaddress.IPv4Address(s).packed)
    except ValueError as e:
        raise ConfigError(f"{path}: {e}") from None


def build(config_path: str, board_path: str) -> Dict[str, Any]:
    cfg = take(load_yaml(config_path), "controller", {
        "fw_version": int, "shared": dict, "network": dict, "servo_bus": dict, "pwm": dict, "safety": dict,
    })
    shared = take(cfg["shared"], "controller.shared", {"comm_core": str, "hand_model": str})
    comm_path = resolve(config_path, shared["comm_core"])
    hand_path = resolve(config_path, shared["hand_model"])

    # Pi 쪽 설정에서 필요한 값만 읽는다 (Pi 로더가 전체 검증을 담당)
    comm = load_yaml(comm_path)
    pico = comm.get("pico") if isinstance(comm, dict) else None
    if not isinstance(pico, dict) or not all(k in pico for k in ("host", "port", "pico_cmd_timeout_ms")):
        raise ConfigError(f"{comm_path}: pico.host / pico.port / pico.pico_cmd_timeout_ms 가 필요합니다")
    hand = load_yaml(hand_path)
    acts = hand.get("actuators") if isinstance(hand, dict) else None
    if not isinstance(acts, list) or not acts:
        raise ConfigError(f"{hand_path}: actuators 목록이 필요합니다")

    net = take(cfg["network"], "network", {
        "mac": str, "netmask": str, "gateway": str, "listen_sockets": int, "socket_buf_kb": int,
        "link_idle_timeout_ms": int, "retry_time_100us": int, "retry_count": int, "keepalive_5s": int,
    })
    bus = take(cfg["servo_bus"], "servo_bus", {
        "baud": int, "response_timeout_us": int, "return_delay_us": int, "loop_period_us": int,
        "reads_per_loop": int, "goal_speed": int, "goal_acc": int, "torque_off_repeat_ms": int,
        "max_missed_reads": int,
    })
    pwm = take(cfg["pwm"], "pwm", {"frequency_hz": int, "release_mode": str})
    safety = take(cfg["safety"], "safety", {"hw_watchdog_ms": int, "min_cmd_timeout_ms": int, "max_cmd_timeout_ms": int})

    board = take(load_yaml(board_path), "board", {
        "name": str, "chip": str, "sdk_board": str, "description": str, "source": str,
        "w5500": dict, "servo_bus": dict, "pwm_pins": list, "status_led": dict, "aux_inputs": list, "inferred": list,
    })
    if board["chip"] not in CHIP_GPIO:
        raise ConfigError(f"board.chip: {list(CHIP_GPIO)} 중 하나여야 합니다")
    pins = PinBook(board["chip"])

    # --- W5500 (SPI)
    w = take(board["w5500"], "board.w5500", {
        "spi": int, "baud_hz": int, "sck": PIN, "mosi": PIN, "miso": PIN, "cs": PIN, "rst": PIN, "int": PIN,
    })
    rng("board.w5500.spi", w["spi"], 0, 1)
    rng("board.w5500.baud_hz", w["baud_hz"], 100_000, 80_000_000)
    for role, key in ((2, "sck"), (3, "mosi"), (0, "miso")):
        p = pins.use(f"board.w5500.{key}", w[key], True)
        if spi_of(p) != (w["spi"], role):
            raise ConfigError(f"board.w5500.{key}: GPIO{p} 는 SPI{w['spi']} {key.upper()} 핀이 아닙니다 "
                              f"(RP2350 기능 표: SPI{spi_of(p)[0]} 역할 {spi_of(p)[1]})")
    w["cs"] = pins.use("board.w5500.cs", w["cs"], True)   # CS 는 GPIO 로 직접 제어 (아무 핀 가능)
    w["rst"] = pins.use("board.w5500.rst", w["rst"], False)
    w["int"] = pins.use("board.w5500.int", w["int"], False)

    # --- 서보 버스 (STS3215 반이중 UART)
    sb = take(board["servo_bus"], "board.servo_bus", {
        "uart": int, "tx": PIN, "rx": PIN, "echo": bool,
        "dir": PIN, "dir_tx_level": int, "oe": PIN, "oe_active_level": int,
    })
    rng("board.servo_bus.uart", sb["uart"], 0, 1)
    for role, key in ((0, "tx"), (1, "rx")):
        p = pins.use(f"board.servo_bus.{key}", sb[key], True)
        if uart_of(p) != (sb["uart"], role):
            raise ConfigError(f"board.servo_bus.{key}: GPIO{p} 는 UART{sb['uart']} {key.upper()} 핀이 아닙니다")
    sb["dir"] = pins.use("board.servo_bus.dir", sb["dir"], False)
    sb["oe"] = pins.use("board.servo_bus.oe", sb["oe"], False)
    rng("board.servo_bus.dir_tx_level", sb["dir_tx_level"], 0, 1)
    rng("board.servo_bus.oe_active_level", sb["oe_active_level"], 0, 1)

    # --- PWM 채널 핀
    pwm_pins = [pins.use(f"board.pwm_pins[{i}]", p, True) for i, p in enumerate(board["pwm_pins"])]

    led = take(board["status_led"], "board.status_led", {"pin": PIN, "active_level": int})
    led_pin = pins.use("board.status_led.pin", led["pin"], False)
    for i, a in enumerate(board["aux_inputs"]):
        take(a, f"board.aux_inputs[{i}]", {"name": str, "pin": int, "note": str})
        pins.use(f"board.aux_inputs[{i}]", a["pin"], True)

    # --- 액추에이터 (hand_model.yaml 순서 그대로)
    out_acts = []
    sts_ids = set()
    for i, a in enumerate(acts):
        path = f"{hand_path}: actuators[{i}]"
        for k in ("name", "kind", "bus_id", "raw_min", "raw_max"):
            if k not in a:
                raise ConfigError(f"{path}: {k} 누락")
        if a["raw_min"] > a["raw_max"]:
            raise ConfigError(f"{path}: raw_min > raw_max")
        if a["kind"] == "sts":
            rng(f"{path}.bus_id", a["bus_id"], 0, 253)
            if a["bus_id"] in sts_ids:
                raise ConfigError(f"{path}: STS ID {a['bus_id']} 중복")
            sts_ids.add(a["bus_id"])
            rng(f"{path}.raw", a["raw_min"], 0, 4095)
            rng(f"{path}.raw", a["raw_max"], 0, 4095)
            out_acts.append(("FW_ACT_STS", a["bus_id"], a["raw_min"], a["raw_max"], -1, a["name"]))
        elif a["kind"] == "pwm":
            ch = a["bus_id"]
            if not 0 <= ch < len(pwm_pins):
                raise ConfigError(f"{path}: PWM 채널 {ch} 에 해당하는 board.pwm_pins 항목이 없습니다 "
                                  f"(보드에 {len(pwm_pins)} 개)")
            period_us = 1_000_000 // pwm["frequency_hz"]
            rng(f"{path}.raw_max", a["raw_max"], 0, period_us)
            out_acts.append(("FW_ACT_PWM", ch, a["raw_min"], a["raw_max"], pwm_pins[ch], a["name"]))
        else:
            raise ConfigError(f"{path}: kind 는 sts 또는 pwm 이어야 합니다")
    if len(out_acts) > 32:
        raise ConfigError("액추에이터는 최대 32 개 (프로토콜 MAX_ACTUATORS)")

    if pwm["release_mode"] not in ("low", "hold"):
        raise ConfigError("pwm.release_mode: low (펄스 끊기) 또는 hold (마지막 펄스 유지)")
    mac = [int(x, 16) for x in net["mac"].split(":")]
    if len(mac) != 6 or any(not 0 <= x <= 255 for x in mac):
        raise ConfigError("network.mac: aa:bb:cc:dd:ee:ff 형식이어야 합니다")
    rng("network.listen_sockets", net["listen_sockets"], 1, 8)
    if net["socket_buf_kb"] not in (1, 2, 4, 8, 16) or net["socket_buf_kb"] * net["listen_sockets"] > 16:
        raise ConfigError("network.socket_buf_kb: 1/2/4/8/16 중 하나이고 listen_sockets x socket_buf_kb <= 16 이어야 합니다")
    rng("safety.hw_watchdog_ms", safety["hw_watchdog_ms"], 10, 8000)
    rng("pico.pico_cmd_timeout_ms", pico["pico_cmd_timeout_ms"], safety["min_cmd_timeout_ms"], safety["max_cmd_timeout_ms"])
    rng("servo_bus.reads_per_loop", bus["reads_per_loop"], 0, 32)

    return {
        "cfg": cfg, "net": net, "bus": bus, "pwm": pwm, "safety": safety, "board": board, "w": w, "sb": sb,
        "led_pin": led_pin, "led_level": led["active_level"], "acts": out_acts, "mac": mac,
        "ip": parse_ip("comm_core.pico.host", pico["host"]), "port": rng("comm_core.pico.port", pico["port"], 1, 65535),
        "netmask": parse_ip("network.netmask", net["netmask"]), "gateway": parse_ip("network.gateway", net["gateway"]),
        "cmd_timeout_ms": pico["pico_cmd_timeout_ms"], "sources": [config_path, board_path, comm_path, hand_path],
    }


# ---------------------------------------------------------------- 출력


def arr(v: List[int]) -> str:
    return "{" + ", ".join(str(x) for x in v) + "}"


def render_header(c: Dict[str, Any]) -> str:
    b, w, sb, net, bus, pwm, safety = c["board"], c["w"], c["sb"], c["net"], c["bus"], c["pwm"], c["safety"]
    n_sts = sum(1 for a in c["acts"] if a[0] == "FW_ACT_STS")
    L = [
        "/* 자동 생성 파일: pico_fw/tools/gen_config.py. 직접 고치지 말고 YAML 을 고치세요.",
        *[f" *   {os.path.relpath(s)}" for s in c["sources"]],
        " */",
        "#pragma once",
        '#include "fw_types.h"',
        "",
        f"#define FW_VERSION              {c['cfg']['fw_version']}",
        f'#define FW_BOARD_NAME           "{b["name"]}"',
        f"#define FW_GPIO_COUNT           {CHIP_GPIO[b['chip']]}",
        "",
        "/* 네트워크 (W5500) */",
        f"#define FW_NET_MAC              {arr(c['mac'])}",
        f"#define FW_NET_IP               {arr(c['ip'])}",
        f"#define FW_NET_NETMASK          {arr(c['netmask'])}",
        f"#define FW_NET_GATEWAY          {arr(c['gateway'])}",
        f"#define FW_NET_TCP_PORT         {c['port']}",
        f"#define FW_NET_LISTEN_SOCKETS   {net['listen_sockets']}",
        f"#define FW_NET_SOCKET_BUF_KB    {net['socket_buf_kb']}",
        f"#define FW_NET_LINK_IDLE_MS     {net['link_idle_timeout_ms']}",
        f"#define FW_NET_RETRY_TIME_100US {net['retry_time_100us']}",
        f"#define FW_NET_RETRY_COUNT      {net['retry_count']}",
        f"#define FW_NET_KEEPALIVE_5S     {net['keepalive_5s']}",
        "",
        f"#define FW_W5500_SPI            {w['spi']}",
        f"#define FW_W5500_BAUD_HZ        {w['baud_hz']}",
        f"#define FW_W5500_PIN_SCK        {w['sck']}",
        f"#define FW_W5500_PIN_MOSI       {w['mosi']}",
        f"#define FW_W5500_PIN_MISO       {w['miso']}",
        f"#define FW_W5500_PIN_CS         {w['cs']}",
        f"#define FW_W5500_PIN_RST        {w['rst']}",
        f"#define FW_W5500_PIN_INT        {w['int']}",
        "",
        "/* STS3215 서보 버스 (반이중 UART) */",
        f"#define FW_STS_UART             {sb['uart']}",
        f"#define FW_STS_BAUD             {bus['baud']}",
        f"#define FW_STS_PIN_TX           {sb['tx']}",
        f"#define FW_STS_PIN_RX           {sb['rx']}",
        f"#define FW_STS_ECHO             {1 if sb['echo'] else 0}",
        f"#define FW_STS_PIN_DIR          {sb['dir']}",
        f"#define FW_STS_DIR_TX_LEVEL     {sb['dir_tx_level']}",
        f"#define FW_STS_PIN_OE           {sb['oe']}",
        f"#define FW_STS_OE_ACTIVE_LEVEL  {sb['oe_active_level']}",
        f"#define FW_STS_RESPONSE_TIMEOUT_US {bus['response_timeout_us']}",
        f"#define FW_STS_RETURN_DELAY_US  {bus['return_delay_us']}",
        f"#define FW_STS_LOOP_PERIOD_US   {bus['loop_period_us']}",
        f"#define FW_STS_READS_PER_LOOP   {bus['reads_per_loop']}",
        f"#define FW_STS_GOAL_SPEED       {bus['goal_speed']}",
        f"#define FW_STS_GOAL_ACC         {bus['goal_acc']}",
        f"#define FW_STS_TORQUE_OFF_REPEAT_MS {bus['torque_off_repeat_ms']}",
        f"#define FW_STS_MAX_MISSED_READS {bus['max_missed_reads']}",
        "",
        "/* PWM 서보 */",
        f"#define FW_PWM_FREQUENCY_HZ     {pwm['frequency_hz']}",
        f"#define FW_PWM_RELEASE_HOLD     {1 if pwm['release_mode'] == 'hold' else 0}",
        "",
        "/* 상태 LED (-1 = 없음) */",
        f"#define FW_LED_PIN              {c['led_pin']}",
        f"#define FW_LED_ACTIVE_LEVEL     {c['led_level']}",
        "",
        "/* 안전 */",
        f"#define FW_DEFAULT_CMD_TIMEOUT_MS {c['cmd_timeout_ms']}  /* HELLO 받기 전까지 쓰는 값 (comm_core.yaml pico_cmd_timeout_ms) */",
        f"#define FW_MIN_CMD_TIMEOUT_MS   {safety['min_cmd_timeout_ms']}",
        f"#define FW_MAX_CMD_TIMEOUT_MS   {safety['max_cmd_timeout_ms']}",
        f"#define FW_HW_WATCHDOG_MS       {safety['hw_watchdog_ms']}",
        "",
        "/* 액추에이터: hand_model.yaml 순서 = ACTUATOR_CMD/STATE 값 순서 */",
        f"#define FW_N_ACT                {len(c['acts'])}",
        f"#define FW_N_STS                {n_sts}",
        f"#define FW_N_PWM                {len(c['acts']) - n_sts}",
        "#define FW_ACTUATORS_INIT { \\",
    ]
    for kind, bid, lo, hi, pin, name in c["acts"]:
        L.append(f"    {{{kind}, {bid}, {lo}, {hi}, {pin}}}, /* {name} */ \\")
    L.append("}")
    L.append("")
    return "\n".join(L)


def render_cmake(c: Dict[str, Any]) -> str:
    b = c["board"]
    return "\n".join([
        "# 자동 생성 파일: pico_fw/tools/gen_config.py",
        f'set(PICO_BOARD "{b["sdk_board"]}")',
        'set(PICO_PLATFORM "rp2350-arm-s")',
        f'set(FW_CONFIG_SOURCES "{";".join(os.path.abspath(s) for s in c["sources"])}")',
        "",
    ])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--board", required=True)
    ap.add_argument("--out", required=True, help="fw_config.h / board.cmake 를 쓸 디렉터리")
    args = ap.parse_args()
    try:
        c = build(args.config, args.board)
    except (ConfigError, OSError, yaml.YAMLError) as e:
        print(f"gen_config: 오류: {e}", file=sys.stderr)
        return 1
    os.makedirs(args.out, exist_ok=True)
    for name, text in (("fw_config.h", render_header(c)), ("board.cmake", render_cmake(c))):
        path = os.path.join(args.out, name)
        old = open(path, encoding="utf-8").read() if os.path.exists(path) else None
        if old != text:  # 내용이 같으면 다시 쓰지 않아 불필요한 재빌드를 막는다
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)
    for item in c["board"]["inferred"]:
        print(f"gen_config: 경고: [{c['board']['name']}] 추정값 사용 - {item}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
