"""통신 코어 설정 로더.

하드코딩을 없애기 위해 코드에는 기본값을 두지 않는다. 모든 값은 YAML 파일에서 온다.
  - config/comm_core.yaml : 포트, 주기, 워치독, Pico 링크
  - config/hand_model.yaml: 액추에이터, 14 관절 매핑/커플링, DH 파라미터
누락된 키나 모르는 키가 있으면 시작 시 바로 오류를 낸다 (오타가 조용히 무시되지 않게).
"""

from __future__ import annotations

import json
import os
import typing
from dataclasses import MISSING, dataclass, fields, is_dataclass
from typing import Any, Dict, List, Optional, Tuple

from .pico_link import PicoLinkConfig

Addr = Tuple[str, int]


# ---------------------------------------------------------------- hand model


@dataclass
class ActuatorConfig:
    name: str
    kind: str                 # "sts" (STS3215) | "pwm" (40KG PWM 서보)
    bus_id: int               # STS 서보 ID 또는 PWM 채널 번호 (Pico 펌웨어 참고용)
    torque_limit_nm: float    # CH2 토크 지령 클램핑 한계 (절대값)
    raw_min: int              # 위치 지령 원시값 하한 (STS tick / PWM us)
    raw_max: int
    raw_zero: int             # 0 rad 에 해당하는 원시값
    rad_per_raw: float
    nm_per_effort: float      # 피드백 effort(STS Present Load) -> Nm 추정 계수. 0 이면 0 보고


@dataclass
class SourceTerm:
    """관절 값 = sum(term.scale * 값) + offset. 값은 액추에이터(rad) 또는 앞선 관절."""

    scale: float
    actuator: Optional[str] = None
    joint: Optional[str] = None


@dataclass
class DHParam:
    """표준 DH: Rot_z(theta) Trans_z(d) Trans_x(a) Rot_x(alpha). theta = q + theta_offset."""

    a: float
    alpha: float
    d: float
    theta_offset: float


@dataclass
class JointConfig:
    name: str
    finger: str
    limits: List[float]               # [min, max] rad
    offset: float
    source: Optional[List[SourceTerm]]  # null 이면 측정 불가 -> 0 보고
    dh: DHParam


@dataclass
class FingerConfig:
    name: str
    base_xyz: List[float]             # 손바닥 좌표계 기준 손가락 기저 위치 (m)
    base_rpy: List[float]             # 기저 자세 (rad, roll-pitch-yaw)
    joints: List[str]                 # 기저 -> 끝 순서의 관절 이름


@dataclass
class HandModel:
    actuators: List[ActuatorConfig]
    joints: List[JointConfig]
    fingers: List[FingerConfig]

    def actuator_index(self, name: str) -> int:
        for i, a in enumerate(self.actuators):
            if a.name == name:
                return i
        raise KeyError(name)

    def joint_index(self, name: str) -> int:
        for i, j in enumerate(self.joints):
            if j.name == name:
                return i
        raise KeyError(name)

    def validate(self) -> None:
        if not 0 < len(self.actuators) <= 32:
            raise ValueError("actuators must have 1..32 entries")
        _unique([a.name for a in self.actuators], "actuator")
        _unique([j.name for j in self.joints], "joint")
        for a in self.actuators:
            if a.kind not in ("sts", "pwm"):
                raise ValueError(f"actuator {a.name}: unknown kind {a.kind!r}")
            if a.raw_min > a.raw_max:
                raise ValueError(f"actuator {a.name}: raw_min > raw_max")
            if a.rad_per_raw == 0:
                raise ValueError(f"actuator {a.name}: rad_per_raw must be non-zero")
        names = {a.name for a in self.actuators}
        for i, j in enumerate(self.joints):
            if len(j.limits) != 2 or j.limits[0] > j.limits[1]:
                raise ValueError(f"joint {j.name}: limits must be [min, max]")
            earlier = {x.name for x in self.joints[:i]}
            for t in j.source or []:
                if (t.actuator is None) == (t.joint is None):
                    raise ValueError(f"joint {j.name}: each source term needs exactly one of actuator/joint")
                if t.actuator is not None and t.actuator not in names:
                    raise ValueError(f"joint {j.name}: unknown actuator {t.actuator!r}")
                if t.joint is not None and t.joint not in earlier:
                    raise ValueError(f"joint {j.name}: joint {t.joint!r} must be defined earlier")
        joint_names = {j.name for j in self.joints}
        for f in self.fingers:
            if len(f.base_xyz) != 3 or len(f.base_rpy) != 3:
                raise ValueError(f"finger {f.name}: base_xyz/base_rpy need 3 values")
            for jn in f.joints:
                if jn not in joint_names:
                    raise ValueError(f"finger {f.name}: unknown joint {jn!r}")


# ---------------------------------------------------------------- runtime


@dataclass
class TeamChannelsConfig:
    bind_host: str
    telemetry_port: int                    # CH1 송신 시 출발 포트
    command_port: int                      # CH2
    slip_port: int                         # CH3
    vla_port: int                          # CH4
    telemetry_dest: Addr                   # 3-B 텔레메트리 수신 주소
    proprio_dest: Optional[Addr]           # 1팀 VLA 수신 주소 (null 이면 미송신)
    proprio_rate_hz: float
    slip_relay_dest: Optional[Addr]        # 슬립 신호를 3-B 로 바로 중계할 주소
    vla_action_relay_dest: Optional[Addr]  # VLA 액션을 3-B 로 중계할 주소
    include_fingertips: bool               # 텔레메트리에 DH 기반 손끝 위치 추가 (규격 외 확장 키)
    include_diagnostics: bool              # 텔레메트리에 축별 신선도/고장 단계/링크 품질 추가 (규격 외 확장 키)


@dataclass
class CoreConfig:
    loop_rate_hz: float
    command_watchdog_s: float
    pico: PicoLinkConfig
    channels: TeamChannelsConfig
    hand: HandModel

    @property
    def actuators(self) -> List[ActuatorConfig]:
        return self.hand.actuators

    @property
    def n_joints(self) -> int:
        return len(self.hand.joints)

    def validate(self) -> None:
        if self.loop_rate_hz <= 0:
            raise ValueError("loop_rate_hz must be > 0")
        self.hand.validate()


# ---------------------------------------------------------------- loading


def _unique(names: List[str], what: str) -> None:
    seen = set()
    for n in names:
        if n in seen:
            raise ValueError(f"duplicate {what} name {n!r}")
        seen.add(n)


def _convert(tp: Any, value: Any, path: str) -> Any:
    origin = typing.get_origin(tp)
    args = typing.get_args(tp)
    if origin is typing.Union:  # Optional[X]
        if value is None:
            return None
        inner = [a for a in args if a is not type(None)][0]
        return _convert(inner, value, path)
    if value is None:
        raise ValueError(f"{path}: must not be null")
    if is_dataclass(tp):
        return build(tp, value, path)
    if origin in (list, List):
        if not isinstance(value, list):
            raise ValueError(f"{path}: expected a list")
        return [_convert(args[0], v, f"{path}[{i}]") for i, v in enumerate(value)]
    if origin in (tuple, Tuple):
        if not isinstance(value, (list, tuple)) or len(value) != len(args):
            raise ValueError(f"{path}: expected {len(args)} values")
        return tuple(_convert(a, v, f"{path}[{i}]") for i, (a, v) in enumerate(zip(args, value)))
    if tp is float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{path}: expected a number")
        return float(value)
    if tp is int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{path}: expected an integer")
        return value
    if tp is bool:
        if not isinstance(value, bool):
            raise ValueError(f"{path}: expected true/false")
        return value
    if tp is str:
        if not isinstance(value, str):
            raise ValueError(f"{path}: expected a string")
        return value
    return value


def build(cls: Any, data: Any, path: str = "") -> Any:
    """dict -> dataclass. 기본값이 없는 필드는 필수, 모르는 키는 오류."""
    if not isinstance(data, dict):
        raise ValueError(f"{path or cls.__name__}: expected a mapping")
    hints = typing.get_type_hints(cls)
    known = {f.name: f for f in fields(cls)}
    unknown = set(data) - set(known)
    if unknown:
        raise ValueError(f"{path or cls.__name__}: unknown keys {sorted(unknown)}")
    kwargs: Dict[str, Any] = {}
    for name, f in known.items():
        sub = f"{path}.{name}" if path else name
        if name not in data:
            if f.default is not MISSING:  # 선택 키 (SourceTerm 의 actuator/joint 등)
                continue
            raise ValueError(f"{sub}: missing")
        kwargs[name] = _convert(hints[name], data[name], sub)
    return cls(**kwargs)


def read_file(path: str) -> Any:
    with open(path, "r", encoding="utf-8") as f:
        if path.endswith((".yaml", ".yml")):
            import yaml  # Pi OS: sudo apt install python3-yaml

            return yaml.safe_load(f)
        return json.load(f)


def load_hand_model(path: str) -> HandModel:
    hand = build(HandModel, read_file(path), "hand")
    hand.validate()
    return hand


def config_from_dict(data: Dict[str, Any], base_dir: str = ".") -> CoreConfig:
    """`hand_model` 키는 파일 경로(설정 파일 기준 상대경로) 또는 인라인 mapping."""
    data = dict(data)
    hand_src = data.pop("hand_model", None)
    if hand_src is None:
        raise ValueError("hand_model: missing")
    if isinstance(hand_src, str):
        hand_data = read_file(os.path.join(base_dir, hand_src))
    else:
        hand_data = hand_src
    data["hand"] = hand_data
    cfg = build(CoreConfig, data)
    cfg.validate()
    return cfg


def load_config(path: str) -> CoreConfig:
    return config_from_dict(read_file(path), os.path.dirname(os.path.abspath(path)))
