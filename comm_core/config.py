"""통신 코어 설정. 모든 값은 JSON 파일로 덮어쓸 수 있다 (config/comm_core.example.json 참고)."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, fields, is_dataclass
from typing import Any, Dict, List, Optional, Tuple

from .pico_link import PicoLinkConfig


@dataclass
class ActuatorConfig:
    name: str
    kind: str                     # "sts" (STS3215) | "pwm" (40KG PWM 서보)
    torque_limit_nm: float = 2.5  # CH2 토크 클램핑 한계 (절대값)
    raw_min: int = 0              # 위치 지령 원시값 하한 (STS tick / PWM us)
    raw_max: int = 4095
    raw_zero: int = 2048          # 0 rad 에 해당하는 원시값
    rad_per_raw: float = 2 * math.pi / 4096
    nm_per_effort: float = 0.0    # 피드백 effort 원시값 -> Nm (0 이면 토크 미보고)
    default_raw: Optional[int] = None  # 시작 시 지령 (없으면 raw_zero)


def _default_actuators() -> List[ActuatorConfig]:
    sts_names = ["thumb_base", "thumb_tip", "index_mcp", "middle_mcp", "ring_mcp", "little_mcp"]
    pwm_names = ["index_tendon", "middle_tendon", "ring_tendon", "little_tendon"]
    acts = [
        ActuatorConfig(n, "sts", torque_limit_nm=1.8 if i < 2 else 2.5, nm_per_effort=0.0019)
        for i, n in enumerate(sts_names)
    ]
    acts += [
        # 40KG PWM: 500~2500us, 1500us = 0 rad, 2000us/270deg 기준
        ActuatorConfig(n, "pwm", raw_min=500, raw_max=2500, raw_zero=1500, rad_per_raw=math.radians(270) / 2000)
        for n in pwm_names
    ]
    return acts


@dataclass
class JointSource:
    """관절 i 의 값을 어디서 얻는지. actuator 또는 joint (다른 관절의 커플링) 중 하나."""

    actuator: Optional[int] = None
    joint: Optional[int] = None
    scale: float = 1.0


def _default_joint_map() -> List[Optional[JointSource]]:
    # 임시 매핑: 관절 0~9 = 액추에이터 0~9, 10~13 은 추정 불가로 0.
    # 실제 14 관절 인덱스 정의가 확정되면 설정 파일에서 교체한다.
    return [JointSource(actuator=i) for i in range(10)] + [None] * 4


@dataclass
class TeamChannelsConfig:
    bind_host: str = "0.0.0.0"
    telemetry_port: int = 5555            # CH1 송신 시 출발 포트로도 사용
    command_port: int = 5556              # CH2
    slip_port: int = 5557                 # CH3
    vla_port: int = 5559                  # CH4
    telemetry_dest: Tuple[str, int] = ("127.0.0.1", 15555)   # 3-B 수신 주소
    proprio_dest: Optional[Tuple[str, int]] = None           # 1팀 VLA 수신 주소 (None 이면 미송신)
    proprio_rate_hz: float = 30.0
    slip_relay_dest: Optional[Tuple[str, int]] = None        # 슬립 신호를 3-B 로 바로 중계할 주소
    vla_action_relay_dest: Optional[Tuple[str, int]] = None  # VLA 액션을 3-B 로 중계할 주소


@dataclass
class CoreConfig:
    loop_rate_hz: float = 100.0
    command_watchdog_s: float = 0.1
    n_joints: int = 14
    pico: PicoLinkConfig = field(default_factory=PicoLinkConfig)
    channels: TeamChannelsConfig = field(default_factory=TeamChannelsConfig)
    actuators: List[ActuatorConfig] = field(default_factory=_default_actuators)
    joint_map: List[Optional[JointSource]] = field(default_factory=_default_joint_map)

    def validate(self) -> None:
        if not 0 < len(self.actuators) <= 32:
            raise ValueError("actuators must have 1..32 entries")
        if len(self.joint_map) != self.n_joints:
            raise ValueError(f"joint_map must have {self.n_joints} entries")
        for i, src in enumerate(self.joint_map):
            if src is None:
                continue
            if (src.actuator is None) == (src.joint is None):
                raise ValueError(f"joint_map[{i}]: set exactly one of actuator/joint")
            if src.actuator is not None and not 0 <= src.actuator < len(self.actuators):
                raise ValueError(f"joint_map[{i}]: actuator index out of range")
            if src.joint is not None and not 0 <= src.joint < i:
                raise ValueError(f"joint_map[{i}]: joint must refer to an earlier joint")
        for a in self.actuators:
            if a.kind not in ("sts", "pwm"):
                raise ValueError(f"actuator {a.name}: unknown kind {a.kind!r}")
            if a.raw_min > a.raw_max:
                raise ValueError(f"actuator {a.name}: raw_min > raw_max")


def _build(cls, data: Any):
    if data is None:
        return None
    if not is_dataclass(cls):
        return data
    kwargs: Dict[str, Any] = {}
    known = {f.name: f for f in fields(cls)}
    for key, value in data.items():
        if key not in known:
            raise ValueError(f"unknown config key {cls.__name__}.{key}")
        kwargs[key] = value
    return cls(**kwargs)


def _addr(v: Any) -> Optional[Tuple[str, int]]:
    return None if v is None else (str(v[0]), int(v[1]))


def config_from_dict(data: Dict[str, Any]) -> CoreConfig:
    data = dict(data)
    pico = _build(PicoLinkConfig, data.pop("pico", {}))
    ch = data.pop("channels", {})
    ch = _build(TeamChannelsConfig, ch)
    for name in ("telemetry_dest", "proprio_dest", "slip_relay_dest", "vla_action_relay_dest"):
        setattr(ch, name, _addr(getattr(ch, name)))
    kwargs: Dict[str, Any] = {"pico": pico, "channels": ch}
    if "actuators" in data:
        kwargs["actuators"] = [_build(ActuatorConfig, a) for a in data.pop("actuators")]
    if "joint_map" in data:
        kwargs["joint_map"] = [_build(JointSource, j) for j in data.pop("joint_map")]
    cfg = _build(CoreConfig, data)
    for k, v in kwargs.items():
        setattr(cfg, k, v)
    cfg.validate()
    return cfg


def load_config(path: Optional[str]) -> CoreConfig:
    if path is None:
        cfg = CoreConfig()
        cfg.validate()
        return cfg
    with open(path, "r", encoding="utf-8") as f:
        return config_from_dict(json.load(f))
