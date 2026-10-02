# pico_fw: Pico 2 컨트롤러 펌웨어

Pi 5 의 `comm_core` (TCP 클라이언트) 와 랜선으로 연결되는 Pico 2 (RP2350) + W5500 펌웨어입니다.
STS3215 6축 (반이중 UART) 과 PWM 서보 4축을 구동합니다. 프로토콜은 `comm_core/README.md` 의 "Pi ↔ Pico TCP 프로토콜" 그대로입니다.

- 언어/SDK: **C + Raspberry Pi Pico SDK 2.1** (RP2350 공식 SDK, 듀얼 코어와 하드웨어 워치독을 직접 씀). W5500 드라이버는 외부 라이브러리 없이 `src/w5500.c` 에 직접 구현했습니다.
- core0: W5500 폴링 → 프레임 처리 → 워치독 / core1: 서보 버스와 PWM (버스 통신이 네트워크 응답을 막지 않음)
- 코드에 핀 번호나 파라미터가 없습니다. 모두 YAML 에서 읽어 빌드 때 `fw_config.h` 를 생성합니다.

## 빌드

```bash
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi python3-yaml
git clone -b 2.1.1 https://github.com/raspberrypi/pico-sdk && (cd pico-sdk && git submodule update --init lib/tinyusb)

cd pico_fw
cmake -B build -DPICO_SDK_PATH=../pico-sdk -DFW_BOARD=pico2_w5500     # 보드 = config/boards/<이름>.yaml
cmake --build build -j
# build/pico_controller.uf2 를 BOOTSEL 모드의 Pico 2 에 복사
```

디버그 로그는 USB 시리얼(CDC)로 나옵니다 (`stat rx=.. tx=.. cmd=.. wd=..` 5초마다).

## 설정 파일

| 파일 | 내용 |
|---|---|
| `config/controller.yaml` | MAC/넷마스크/게이트웨이, W5500 소켓 수·버퍼, 서보 버스 보레이트·타이밍, PWM 주파수, 안전 파라미터, **강건성 임계값 (`robustness:`)** |
| `config/boards/pico2_w5500.yaml` | Pico 2 + W5500 모듈 (점퍼 배선). `Pico_middle_controller.md` 3장 핀 배분 |
| `config/boards/middleware_pcb_rev0.yaml` | 업로드된 컨트롤러 PCB (RP2350B 칩 내장형) 에서 읽은 핀 |
| `../config/comm_core.yaml` (Pi) | **Pico IP = `pico.host`, 포트 = `pico.port`, 자체 워치독 = `pico.pico_cmd_timeout_ms`** 를 그대로 읽음 |
| `../config/hand_model.yaml` (Pi) | 액추에이터 순서, 종류(sts/pwm), STS ID / PWM 채널, `raw_min`~`raw_max` 를 그대로 읽음 |

Pi 와 같은 값을 두 번 적지 않으므로, IP 나 서보 ID 를 바꾸면 Pi 설정만 고치고 펌웨어를 다시 빌드하면 됩니다.
생성기는 핀이 RP2350 기능 표(SPI/UART)와 맞는지, 핀이 겹치지 않는지, 키 누락·오타가 없는지 검사하고 틀리면 cmake 단계에서 멈춥니다.

## 동작 규칙

| 상황 | 동작 |
|---|---|
| 전원 투입 / 리셋 | 전 축 토크 해제 상태로 시작 (STS Torque Enable = 0, PWM 펄스 없음) |
| HELLO | `cmd_timeout_ms` 저장 (`safety.min/max_cmd_timeout_ms` 범위로 제한), HELLO_ACK 응답 (`fw_version`, STS 6, PWM 4) |
| ACTUATOR_CMD 위치(mode 0) | `raw_min`~`raw_max` 로 잘라서 적용, 토크 켬. 잘렸으면 ERROR 4 (연결당 1회) + 축 flags `0x10` |
| ACTUATOR_CMD 토크(mode 1) | 값이 0 인 축은 토크 해제. 0 이 아닌 값은 STS3215 가 위치 서보라 지원하지 않음 → ERROR 3, 해당 축 변화 없음 |
| 모든 ACTUATOR_CMD | STATE 1개 응답 |
| ESTOP | 전 축 토크 해제 (`ST_ESTOP`), 다음 위치 지령까지 유지 |
| `cmd_timeout_ms` 동안 지령 없음 | **Pico 가 스스로 전 축 토크 해제** (`ST_WATCHDOG`). Pi 가 죽거나 랜선이 빠진 경우 |
| 연결 끊김 / 랜선 링크 다운 / `link_idle_timeout_ms` 수신 없음 | 연결 정리 후 전 축 토크 해제, 다시 대기 |
| 새 연결 | 이전 연결을 끊고 새 연결을 받음 (`listen_sockets: 2`, 좀비 연결 때문에 재연결이 막히지 않게) |
| 메인 루프 정지 | RP2350 하드웨어 워치독 리셋 (`hw_watchdog_ms`) → 토크 해제 상태로 재시작 |

STS 토크 해제는 응답 없는 패킷이므로 해제 상태에서 `torque_off_repeat_ms` 마다 다시 보냅니다. 토크를 켤 때는 목표 위치를 먼저 쓰고 켭니다 (예전 목표로 튀지 않게).

## 강건성 (잡음, 누락, 관측 불가)

"모든 데이터가 지연·누락 없이 관측된다" 는 가정을 빼고, 각 경로에서 깨지거나 빠질 수 있다고 보고 처리합니다.
임계값은 모두 `config/controller.yaml` 의 `robustness:` 에 있고, 생성기가 키 누락·오타·범위를 검사합니다.

| 경로 | 깨질 수 있는 것 | 처리 |
|---|---|---|
| Pi ↔ Pico (TCP, W5500-Pico SPI) | 바이트 비트 오류, 프레임 누락, 지연/지터 | CRC-16 으로 버리고 재동기화. 수신 seq 가 건너뛰면 `seq_gaps` + DEGRADED. 지령 간격 최대/평균 측정, `cmd_jitter_warn_ms` 초과면 DEGRADED. STATE 에 응답한 지령 seq 를 담아 Pi 가 왕복 시간과 누락을 잼 |
| W5500 (SPI) | 레지스터 읽기 오류, 칩 단독 리셋, 값이 계속 흔들림 | 설정은 쓰고 다시 읽어 확인. `health_period_ms` 마다 VERSIONR/IP/MAC/소켓 상태 확인, `fail_threshold` 연속 실패면 연결을 버리고(=토크 해제) 재초기화, `fallback_after` 회 이상이면 SPI 클럭을 `fallback_baud_hz` 로 낮춤. 16비트 레지스터 안정 읽기는 횟수 제한. 소켓이 닫힌 것으로 읽히면 한 번 더 읽어 확인 |
| 서보 버스 (반이중 UART) | 비트 오류, 바이트 유실, 잡음 바이트, 충돌 | 송신 에코를 보낸 값과 비교해 틀리면 재전송(`write_retries`). 응답은 헤더 재동기화 + ID/길이/체크섬 검사, `read_retries` 재시도 |
| 서보 측정값 | 8비트 체크섬을 우연히 통과한 엉뚱한 값 | 타당성 검사: 위치 범위, 직전 값에서 `max_speed_ticks_per_s × dt + jump_margin_ticks` 넘는 점프, 온도/전압 범위. `max_reject_streak` 번 연속이면 실제 변화로 인정 |
| 서보 측정 누락 | 응답 없음, 버스 전체 무응답 | 축별 `age_ms` 보고. `stale_ms` 초과 → STALE + 관측기 추정값(목표를 향한 1차 지연, `observer_tau_ms`, `estimate_max_ms` 까지). `lost_ms` 초과 → NO_RESPONSE + `lost_action` (hold = 목표 고정). 모든 STS 가 `bus_dead_ms` 넘게 무응답 → `bus_dead_action` (torque_off). 측정이 돌아오면 측정 위치에서 다시 켬 |
| 서보 지령 (응답 없는 SYNC WRITE) | 서보가 패킷을 놓침, 서보 저전압 리셋으로 토크 꺼짐 | `verify_period_ms` 마다 한 축씩 TORQUE_ENABLE/GOAL 을 읽어 비교, 다르면 CMD_MISMATCH + 다시 씀 |
| 지령 자체 | 큰 계단, 튄 지령 | 변화율 제한 (`sts_ticks_per_s`, `pwm_us_per_s`). 토크를 켤 때는 측정 위치에서 출발 |
| 서보 상태 | 과열, 전압 이상, 과부하 | 경고 플래그 + DEGRADED. `temp_off_c`/`load_off_permille` 가 `protect_ms` 동안 계속되면 그 축만 토크 해제(PROTECT_OFF), Pi 가 그 축 토크를 해제할 때까지 유지 |
| Pico 자체 | 루프 지연, 리셋 | core1 루프 최대 시간/초과 횟수 측정. 리셋 원인(POR/BOR 저전압/RUN 핀/글리치/워치독) 을 DIAG 로 보고 |

고장 단계 (축별 `level`, 전체는 `status` bit8-9): `OK` → `DEGRADED` (재시도/누락/추정값 사용 중, 제어 계속) → `HOLD` (관측 불가 축 목표 고정) → `SAFE_OFF` (토크 해제: 워치독/ESTOP/보호/버스 무응답).

DIAG 메시지(`diag_period_ms` 마다, 연결 직후 1회): 가동 시간, 리셋 원인, 전체 단계, 수신 프레임/CRC 오류/버린 바이트/seq 건너뜀, 지령 간격 최대/평균, 버스 타임아웃/깨진 패킷/에코 오류/재시도, 타당성 탈락, 재확인 불일치, 변화율 제한, SPI 오류, W5500 재초기화, 연결 끊김, 루프 초과/최대, 워치독, ESTOP, 보호 해제. 순서는 `src/proto.h` 의 `DIAG_FIELDS` 와 `comm_core/protocol.py` 의 `DIAG_FIELDS` 가 같아야 하며 시험이 검사합니다.

### STATE 필드

- `status`: bit0 Pi 가 토크를 요청한 축 있음, bit1 워치독 해제, bit2 ESTOP, bit3 응답 없는 서보 있음, bit4 HELLO 받음, bit5 최근 seq 건너뜀/지터, bit8-9 고장 단계
- `error`: 마지막 ERROR 코드 (1 payload 오류, 2 축 수 불일치, 3 지원하지 않는 모드, 4 값 잘림)
- `cmd_seq`: 이 STATE 가 응답한 ACTUATOR_CMD 의 seq (Pi 의 왕복 시간/누락 측정용)
- 축마다 24 B: `position` (STS tick / PWM us), `velocity` (STS step/s), `effort` (STS Present Load, 0.1 % 단위, 부호 있음), `temperature` (0.1 °C), `flags`, `age_ms` (마지막 유효 측정 이후, 0xFFFF = 측정 없음), `voltage` (0.1 V), `level`
- `flags`: bit0 응답 없음, bit1 패킷 오류, bit2 서보 오류 바이트, bit3 실제로 토크 켜짐, bit4 값 잘림, bit5 STALE, bit6 추정값, bit7 타당성 탈락, bit8 과열, bit9 전압, bit10 과부하, bit11 지령 불일치(재전송함), bit12 변화율 제한 중, bit13 HOLD, bit14 보호 해제
- STS 상태는 core1 이 라운드로빈으로 읽습니다 (`reads_per_loop: 2`, `loop_period_us: 2000` → 6 축 전체 약 6 ms 마다 갱신)

## 시험 (하드웨어 없이)

```bash
python3 -m unittest tests.test_pico_fw -v     # 저장소 루트에서
```

`src/proto.c`, `src/session.c` 를 리눅스용으로 컴파일한 시뮬레이터(`host/sim_main.c`, W5500 대신 POSIX 소켓)에 Pi 쪽 `comm_core.PicoLink` 를 실제로 붙여 HELLO, 위치 지령/STATE, 값 잘림, 토크 해제, Pico 워치독, ESTOP, 새 연결 우선, 깨진 바이트 재동기화를 검사합니다. 설정 생성기의 핀 검사도 같이 돌립니다. W5500/UART/PWM 드라이버는 실제 보드에서 확인해야 합니다.

### 가상 입력 -> 펌웨어 출력 확인

```bash
python3 pico_fw/host/pipeline_check.py --out report.md
```

팀 UDP 포트(CH2 지령, CH3 슬립, CH4 VLA)에 가상 입력을 넣고, 실제 `comm_core.CommCore` 가 보낸 TCP 지령을 펌웨어가 처리한 결과를 기록합니다.
(서보로 나가는 목표는 변화율 제한 때문에 여러 주기에 나눠 나갑니다.)
이 경우 펌웨어는 서보 루프(`actuators.c`, `sts_bus.c`)까지 포함해 `host/hal_stub.c` (Pico SDK 대체, 가짜 STS3215 6개) 위에서 돕니다.
기록하는 출력은 세 가지입니다: Pi 로 가는 TCP 프레임, 서보 버스 UART 패킷, PWM 펄스 폭. 시나리오별로 분류해 보고서로 냅니다.

### 고장 주입

```bash
python3 pico_fw/host/fault_check.py --out fault_report.md    # 시나리오 하나만: --only dead_servo
```

같은 시뮬레이터에 stdin 으로 고장을 넣고(`fault uart_flip 3000`, `fault dead 4 1`, `fault tcp_rx_flip 2000` 등, 목록은 `host/hal_stub.c` 의 `hal_fault()` 와 `host/sim_main.c` 머리말), Pi 쪽 `PicoLink` 가 100 Hz 로 위치 지령을 보내며 STATE/DIAG 를 기록합니다.
시나리오 14 개(정상, 버스 잡음 약/강, 에코 오류, 튄 측정값, 서보 1 개 무응답, 버스 전체 무응답, 서보 리셋, 놓친 쓰기, 과열, Pi→Pico 잡음, Pico→Pi 잡음, Pico 멈춤, 계단 지령)와 W5500 SPI 시험(`host/w5500_fault_test.c`: 실제 `w5500.c` 를 레지스터 에뮬레이터에 붙여 설정 쓰기 오류, 칩 리셋, 읽기 잡음, 흔들리는 레지스터, Sn_SR 오독, 클럭 낮춤 확인)을 돌리고 각각 기대대로인지 판정합니다. `tests/test_pico_fw.py` 가 전부 통과하는지 검사합니다.

## 업로드된 컨트롤러 PCB 검토 (middleware_pcb_rev0)

업로드 파일은 거버가 아니라 Altium 원본(`*.PcbDoc`, `*.SchDoc` 16장)입니다. PcbDoc 의 패드별 넷과 회로도의 U2 핀 이름을 맞춰 읽었습니다. **넷 라벨 이름은 실제 GPIO 번호와 다릅니다** (예: 넷 `SCK` 는 GPIO6, 넷 `SDO` 는 GPIO32). 아래는 U2 핀 기준입니다.

### PCB 에서 읽은 핀 (U2 = RP2350B QFN-80)

| U2 핀 | GPIO | 넷 | 연결 대상 |
|---|---|---|---|
| 3 | GPIO6 | SCK | U7.P5 SCK (SJA1105) |
| 48 | GPIO39 | SDI | U7.N5 SDI |
| 40 | GPIO32 | SDO | U7.P4 SDO |
| 53 | GPIO42 | GPIO42_ADC2 | U7.P3 RST_N |
| 49 | GPIO40 | OE | U9.5 OE (NTSX2102 레벨 시프터) |
| 56 | GPIO45 | GPIO45_ADC5 | U9.2 A1 → U9.7 B1 → R41 (그 뒤 연결 없음) |
| 72 | QSPI_SD0 | QSPI_SD0 | U9.3 A2 → B2 → U11.B2 (NX20P0477) |
| 2 | GPIO5 | GPIO5 | U1 CHRG (충전 상태, LED1) |
| 52 / 54 | GPIO41 / GPIO43 | CHRG / CHRG_1 | U12 / U4 CHRG |
| 1 | GPIO4 | PROG_1 | U5.5 PROG |
| 77~80 | GPIO0~3 | SDA0/SCL0/SDA1/SCL1 | 풀업 저항만 (I2C 장치 없음) |
| 30/31 | XIN/XOUT | | Y1 12 MHz |
| 33/34 | SWCLK/SWDIO | | R6/R7 → J2 (3핀 SWD 헤더) |
| 66/67 | USB_DM/DP | | R5/R4 → 커넥터 없음 |

GPIO6/39/32 는 RP2350 의 SPI0 SCK/TX/RX 기능 핀이 맞으므로, 이 세 핀은 그대로 W5500 에 쓸 수 있습니다.

### 그대로는 동작하지 않는 부분 (펌웨어로 해결 불가, PCB 수정 필요)

1. **W5500 이 없습니다.** 이더넷 쪽 칩은 U7 `SJA1105PELY` (차량용 5포트 이더넷 **스위치**) 입니다. 스위치에는 TCP/IP 스택이 없고 RP2350 에는 MAC 이 없어서 이 구성으로는 Pi 와 TCP 연결을 할 수 없습니다. SJA1105 의 MII0~4 포트는 아무 데도 연결되어 있지 않고, RJ45(J4) 는 ESD 다이오드(D1)에만 연결되어 PHY 가 없습니다. → 프로젝트 결정대로 **W5500 (PHY 내장) 을 SPI0 (GPIO6/39/32) 에 연결**하는 것을 권장합니다.
2. **SPI CS 가 없습니다.** U7 `SS_N` 이 USB-C CC2 넷(U3, R25)에 묶여 있습니다. 펌웨어는 GPIO33 (SPI0 CSn 기능 핀, 미사용) 을 추정값으로 씁니다.
3. **QSPI 플래시가 없습니다.** U2 의 QSPI 핀이 비어 있어(SD0 은 레벨 시프터로 감) 펌웨어를 저장하고 부팅할 수 없습니다. W25Q 계열 플래시 추가가 필요합니다 (`boards/middleware_pcb_rev0.h` 는 W25Q080, 4 MB 가정).
4. **서보 버스 배선이 없습니다.** STS3215 커넥터와 UART 연결이 없습니다. U9 레벨 시프터의 A1 = GPIO45 (UART0 RX 기능 핀) 이므로 서보 버스용으로 보고, TX 는 GPIO44 (UART0 TX, 미사용) 로 추정했습니다. U9 A2 에 QSPI_SD0 이 연결된 것은 오류로 보입니다.
5. **PWM 출력과 상태 LED 가 없습니다.** PWM 4채널은 GPIO7~10 으로 추정, LED 는 없음으로 설정했습니다.
6. **구리 배선이 없습니다.** PcbDoc 에 트랙이 0 개 (Mechanical/Overlay 레이어만 있음) 로 아직 라우팅 전 상태입니다.
7. 그 외: GPIO4 가 충전 IC(U5) 의 PROG 핀(충전 전류 설정 저항 핀)에 연결됨, U6 (RAA270005) 와 U8 (MFS2613) 의 SPI 가 RP2350 에 연결되지 않음, USB 데이터선(R4/R5 뒤)이 커넥터에 연결되지 않음 (J9 는 D4 를 거치는 전원 입력만). 펌웨어 업로드는 J2 SWD 로 해야 합니다.

추정값은 `middleware_pcb_rev0.yaml` 의 `inferred` 에 적혀 있고 빌드할 때마다 경고로 출력됩니다. PCB 를 고치면 이 YAML 만 고치면 됩니다.
