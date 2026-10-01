# comm_core: Pi 5 통신 코어 (3-A)

Raspberry Pi 5 (4GB) 에서 도는 통신 코어입니다. 한쪽은 타 팀 UDP 채널(`COMM_SOCKET_SPEC` 기준), 다른 한쪽은 랜선으로 연결된 **Pico 2 + W5500** 컨트롤러 보드(TCP)입니다.

```
3-B 제어 ──UDP 5556 지령──▶ ┌──────────────┐ ──TCP (랜선)──▶ Pico 2 + W5500 ──▶ STS3215 x6 / PWM x4
3-B 제어 ◀─UDP 5555 텔레메트리─ │  comm_core   │ ◀──STATE─────
4팀 촉각 ──UDP 5557 슬립──────▶ │  (Pi 5)      │ ──중계──▶ 3-B
1팀 VLA  ◀─UDP 5559 q,dq──────  │  100 Hz 루프 │
1팀 VLA  ──UDP 5559 액션──────▶ └──────────────┘ ──중계──▶ 3-B
```

- 표준 라이브러리만 사용합니다 (Python 3.11, Pi OS Bookworm 기본). 추가 설치가 필요 없습니다.
- 스레드는 Pico 링크용 1개뿐이고, UDP 채널은 100 Hz 루프에서 논블로킹으로 poll 합니다.

## 실행

```bash
# 하드웨어 없이 시험: 가짜 Pico 서버
python3 -m comm_core.fake_pico --port 5000

# 통신 코어 (설정 파일 생략 시 기본값, Pico 기본 주소 192.168.10.20:5000)
python3 -m comm_core --config config/comm_core.example.json

# 테스트
python3 -m unittest discover -s tests -t .
```

## 안전 규칙 (구현됨)

| 규칙 | 동작 |
|---|---|
| 100 ms 워치독 | CH2 지령이 100 ms 넘게 없으면 전 축 토크 0 지령(`WATCHDOG_TRIPPED` 플래그)으로 대체. 시작 직후에도 첫 지령 전까지 토크 0 |
| 토크 클램핑 | 엄지(액추에이터 0, 1) ±1.8 Nm, 나머지 ±2.5 Nm. 설정 `actuators[].torque_limit_nm` |
| 위치 클램핑 | 액추에이터별 `raw_min`~`raw_max` (STS 0~4095 tick, PWM 500~2500 us) |
| 잘못된 지령 | 길이 불일치, NaN/inf, 모르는 mode, 깨진 JSON 은 버리고 카운트만 증가 |
| Pico 자체 워치독 | 연결 시 HELLO 로 `cmd_timeout_ms` 전달. **Pico 펌웨어는 이 시간 동안 지령이 없으면 스스로 토크를 해제해야 함** (Pi 가 죽거나 랜선이 빠진 경우 대비) |
| 링크 감시 | 300 ms 동안 Pico 수신이 없으면 끊김으로 판단하고 재연결(100 ms~2 s 백오프). 텔레메트리 status = `PICO_DISCONNECTED` |
| 종료 | 종료 시 Pico 에 `ESTOP` 송신 |

## CH2 지령 형식 (3-B → 3-A, :5556)

규격서의 torque 모드에 더해, 실제 하드웨어가 위치 서보이므로 위치 모드도 받습니다.

```json
{"mode": "torque",       "torques":   [10개, Nm]}
{"mode": "position",     "positions": [10개, rad]}
{"mode": "raw_position", "raw":       [10개, STS tick / PWM us]}
```

액추에이터 순서: 0~5 = STS3215 (엄지 base, 엄지 tip, 검지/중지/약지/새끼 MCP), 6~9 = PWM 텐던 (검지~새끼).

## CH1 텔레메트리 (3-A → 3-B, :5555, 100 Hz)

규격서 그대로 `seq, timestamp, status, q[14], dq[14], actuator_pos[10], actuator_vel[10], actuator_torque[10]`.
`status` 는 `NORMAL` / `WATCHDOG` / `PICO_DISCONNECTED`.

14 관절 값은 설정 `joint_map` 으로 만듭니다. 관절마다 `{"actuator": i, "scale": s}` 또는 다른 관절의 커플링 `{"joint": j, "scale": 0.88}` (예: DIP = 0.88·PIP), 모르면 `null`. 기본값은 관절 0~9 = 액추에이터 0~9, 10~13 = 0 인 임시 매핑이므로 관절 인덱스가 확정되면 교체해야 합니다.

## Pi ↔ Pico TCP 프로토콜 (펌웨어 구현용)

Pico 2 가 TCP 서버(기본 포트 5000), Pi 5 가 클라이언트입니다. 모든 값은 Little-endian 입니다.

### 프레임

| off | size | 내용 |
|---|---|---|
| 0 | 2 | magic `0xAA 0x55` |
| 2 | 1 | version = 1 |
| 3 | 1 | msg_type |
| 4 | 1 | flags |
| 5 | 1 | reserved (0) |
| 6 | 2 | seq (순환) |
| 8 | 4 | timestamp (송신 측 us, 순환) |
| 12 | 2 | payload_len (≤ 1024) |
| 14 | N | payload |
| 14+N | 2 | CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), header+payload 대상. `"123456789"` → `0x29B1` |

수신 측은 magic 을 찾고, version/길이/CRC 가 틀리면 1바이트 버리고 다시 찾습니다. 모르는 msg_type 은 무시합니다.

### 메시지

| type | 이름 | 방향 | payload |
|---|---|---|---|
| 0x01 | HELLO | Pi→Pico | `u16 cmd_period_ms, u16 cmd_timeout_ms, 4B reserved` |
| 0x02 | HELLO_ACK | Pico→Pi | `u16 fw_version, u8 n_sts, u8 n_pwm, u32 capabilities` |
| 0x03 | HEARTBEAT | 양방향 | 없음. Pico 는 받으면 HEARTBEAT 로 응답 |
| 0x10 | ACTUATOR_CMD | Pi→Pico | `u8 mode, u8 count, u16 reserved, i32 values[count]`. mode 0 = 위치(tick/us), 1 = 토크(mNm, 0 = 토크 해제), 2 = 예약. flags bit0 = 워치독 지령 |
| 0x11 | ESTOP | Pi→Pico | 없음. 전 축 토크 해제 |
| 0x20 | STATE | Pico→Pi | `u16 status, u16 error, u8 count, 3B reserved` + 축마다 `i32 position, i32 velocity, i32 effort, i16 temperature(0.1°C), u16 flags` (16 B) |
| 0x7F | ERROR | 양방향 | `u16 code` + UTF-8 메시지 |

Pico 는 ACTUATOR_CMD 를 받을 때마다 STATE 를 1개 돌려보냅니다 (100 Hz 피드백). 10축 기준 프레임 크기는 지령 60 B, 상태 184 B 로 W5500 100 Mbps 대비 여유가 큽니다. `count` 는 최대 32 축까지 허용해 축이 늘어나도 프레임 형식은 바뀌지 않습니다.

`comm_core/fake_pico.py` 가 이 규칙을 그대로 구현한 참조 서버이므로 펌웨어 동작 기준으로 쓸 수 있습니다.

> 참고: `Pico_middle_controller.md` 는 UDP 26바이트 고정 패킷을 제안하지만, 프로젝트 결정(TCP, 랜선 연결)에 따라 TCP + 가변 길이 프레임으로 구현했습니다. 지연을 줄이기 위해 양쪽 모두 `TCP_NODELAY` 를 켜고, Pi 는 끊긴 동안의 지령을 쌓아두지 않고 버립니다.
