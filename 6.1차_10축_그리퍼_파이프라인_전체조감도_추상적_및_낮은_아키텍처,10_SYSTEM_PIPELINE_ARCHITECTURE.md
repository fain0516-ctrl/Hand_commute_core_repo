# 5지 로봇 그리퍼 통합 시스템 파이프라인 블루프린트
(Full System Pipeline Blueprint: Software & Physical Architecture)

**문서 번호**: AI-PRIORITY-PIPE-20260922  
**작성자**: AI 제어 수석 연구원 (팀 3 제어팀)  
**대상**: 팀 미팅 보고 및 전 팀(1팀 기구, 2팀 회로, 3팀 제어, 4팀 비전) 공통 공유용 SSOT (Single Source of Truth)

---

## 1. 종합 시스템 아키텍처 개요 (System Overview)

본 시스템은 **시각/AI 상위 의사결정 계층**, **하위 실시간 지능 제어 계층**, 그리고 **생체모방 하이브리드 기구부(직접 구동 + 언더액추에이티드 텐던)**로 구성된 3계층 피지컬 AI(Physical AI) 로봇 핸드입니다.

```mermaid
flowchart TB
    subgraph Layer1["1. 상위 인지 및 정책 계층 (High-Level AI Layer / 10~20 Hz)"]
        A["Vision / VLA Model (물체 인식 및 목표 결정)"] --> B["SAC-HER Physical AI Policy"]
        B -->|"목표 파지 자세 q_target / 접촉 의도"| C["Trajectory / Setpoint Dispatcher"]
    end

    subgraph Layer2["2. 미드레벨 지능형 제어 및 중재 계층 (Mid-Level Control Layer / 100 Hz)"]
        C --> D["Grasp FSM (상태 머신)"]
        D --> E["Tactile-Latched Grasp Controller (촉각 래치)"]
        E --> F["Virtual Admittance Controller (가상 순응성)"]
        F --> G["Kinematic Decoupling & Slack Engine (연성 보상)"]
        G --> H["Safety Sequence Interlock (모터 충돌 방지 락)"]
    end

    subgraph Layer3["3. 하드웨어 드라이버 계층 (Low-Level Driver Layer / 100~500 Hz)"]
        H -->|"Half-Duplex UART (1 Mbps)"| I["STS3215 Serial Bus Driver"]
        H -->|"I2C PCA9685 PWM (50~330 Hz)"| J["40KG PWM Motor Driver"]
        I -->|"실시간 피드백 (Pos, Load, Temp)"| K["State Estimator / Load Observer"]
        K -.->|"부하/외력 피드백"| D
    end

    subgraph PhysicalHand["4. 물리 기구 및 액추에이터 계층 (Physical Gripper Mechanism)"]
        I --> M1["[엄지] STS3215 x 2 (직접 구동 / 대립 회전 + 굴곡)"]
        I --> M2["[검지~새끼] STS3215 x 4 (MCP 관절 직접 구동)"]
        J --> M3["[검지~새끼] 40KG PWM x 4 (텐던 와이어 구동)"]
        M3 -->|"장력 인가"| S1["PIP/DIP 복귀 스프링 압축 및 말아쥐기"]
        M1 & M2 & S1 --> OBJ["대상 물체 (취약 물체 / 0.5N 파지)"]
    end
```

---

## 2. [소프트웨어 파이프라인] (Software Pipeline)

소프트웨어 파이프라인은 상위의 불확실한 AI 지령을 하위의 엄밀한 물리 법칙 및 안전 제약 내에서 제어 가능한 명령으로 변환하는 다단계 필터링 구조를 가집니다.

```mermaid
flowchart LR
    subgraph S1["Step 1. 관측 (Sensing)"]
        Raw_UART["STS3215 엔코더 / 부하(Load)"] --> Obs_Builder
        Raw_Tactile["손끝 택타일 센서 / 외력(F_ext)"] --> Obs_Builder
        Obs_Builder["3D 상태 정규화: qpos, force, slip"]
    end

    subgraph S2["Step 2. 판단 (Decision)"]
        Obs_Builder --> Policy["SAC Actor Policy (actor_v6_step_150000.pth)"]
        Policy --> Raw_Cmd["공칭 목표 각도 (mu)"]
    end

    subgraph S3["Step 3. 중재 (Arbitration & Compliance)"]
        Raw_Cmd --> Latch["Tactile Latch (F >= 0.15N 감지 시 고정)"]
        Latch --> Admittance["Admittance Control: M d2(e) + D d(e) + K e = -(F_ext - 0.5N)"]
        Admittance --> TDPA["TDPA 가변 댐핑 소산 (통신 지연 발진 방지)"]
    end

    subgraph S4["Step 4. 분배 및 안전 락 (Safety & Dispatch)"]
        TDPA --> Decouple["기구학 연성 보상: ΔL = r_mcp * Δq_mcp"]
        Decouple --> Interlock{"동작 모드 판단"}
        Interlock -- "파지(Grasp)" --> Seq_Close["MCP 각도 선행 -> 40KG 텐던 당김"]
        Interlock -- "해제(Release)" --> Seq_Open["[1단계] 40KG 슬랙 선행 -> [2단계] STS 원복"]
    end

    subgraph S5["Step 5. 모터 인가 (Actuation)"]
        Seq_Close & Seq_Open --> Cmd_STS["STS3215 위치 패킷 송신 (0x2A)"]
        Seq_Close & Seq_Open --> Cmd_PWM["PCA9685 PWM 펄스 폭 인가 (Max 25% 제한)"]
    end
```

### 2.1 주요 제어 알고리즘 세부 사양

| 모듈명 | 주기 | 입력 데이터 | 출력 데이터 | 핵심 역할 및 수식 |
|:---|:---:|:---|:---|:---|
| **Tactile Latch** | 100 Hz | $F_{ext}$, $q_{mcp}$ | $q_{ref}$ | 접촉 시 목표각을 $q_{contact} + 1.4^\circ$로 래칭하여 뒤로 튕기는 핑퐁 원천 차단 |
| **Admittance** | 100 Hz | $F_{ext}$, $F_{des}(0.5\text{N})$, $q_{ref}$ | $x_{cmd}, v_{cmd}$ | $M_d \ddot{e} + D_d \dot{e} + K_d e = -(F_{ext} - F_{des})$ ($K_d=5.0, D_d=1.5$) |
| **Kinematic Decoupler** | 100 Hz | $\Delta q_{mcp}$ | $\Delta \theta_{40KG}$ | MCP 회전에 따른 텐던 유효 길이 변화 보상: $\Delta L = r_{mcp} \Delta q_{mcp}$ |
| **Sequence Interlock** | 이벤트 | State (Grasp / Release) | Motor Enable Flags | 손가락 펼 때 **[40KG 텐던 슬랙 선행 $\to$ STS3215 원복]** 강제 |
| **Torque Clamp** | 100 Hz | PWM Duty | Clamped PWM | 40KG 모터 최대 토크를 $25\%\sim30\%$ (약 $8\sim10\text{ kgf}$)로 소프트웨어 리미트 |

---

## 3. [물리 및 기구 파이프라인] (Physics & Hardware Pipeline)

물리 파이프라인은 모터의 회전 운동이 텐던, 풀리, 스프링, 그리고 마찰을 거쳐 최종적으로 물체에 부드러운 수직 항력($0.5\text{ N}$)으로 전달되는 기구학/동역학 경로입니다.

### 3.1 손가락 구조별 물리 전달 경로

```mermaid
flowchart TD
    subgraph Thumb["A. 엄지 손가락 물리 경로 (2-DoF Direct Drive / 완전 선형)"]
        T_M1["STS3215 (ID 0)"] -->|"직접 결합"| J0["관절 0-0: 대립각(Opposition) 회전"]
        T_M2["STS3215 (ID 1)"] -->|"직접 결합"| J1["관절 0-1: 굴곡/맞물림 회전"]
        J0 & J1 -->|"자코비안 전치: τ = J^T * F"| T_Tip["엄지 손끝: 2자유도 능동 위치/파지력 제어"]
    end

    subgraph Fingers["B. 검지~새끼 4지 물리 경로 (Hybrid Underactuated / 비선형 텐던)"]
        F_M1["STS3215 (MCP 직접 구동)"] -->|"다이렉트 드라이브"| MCP_Joint["MCP 관절 (기저 각도 제어)"]
        F_M2["40KG PWM 모터"] -->|"스풀 회전 (장력 T_motor)"| Tendon["텐던 와이어 (Dura4 Braid Polyethylene)"]
        
        Tendon --> Guide1["손바닥 가이드 핀"]
        Guide1 -->|"Capstan 마찰 감쇠: η1 = exp(-μ*θ1)"| Guide2["MCP 관절 상단 핀 (r_mcp)"]
        Guide2 -->|"Capstan 마찰 감쇠: η2 = exp(-μ*θ2)"| Guide3["PIP 관절 상단 핀 (r_pip)"]
        
        Guide3 --> Action_Tendon["유효 장력 T_eff = T_motor * Π η_i"]
        Action_Tendon --> Overcome["스프링 대항: T_eff > K_spring * Δx"]
        Overcome --> Curl["PIP/DIP 순차적 말아쥐기 (Enveloping Grasp)"]
        
        Spring["복귀 비틀림/인장 스프링"] -.->|"신전 시 복원력 제공"| Restore["40KG 슬랙 시 PIP/DIP 자동 펼침"]
    end

    subgraph Contact["C. 물체 접촉 및 동역학 완충 (Contact Shock Absorption)"]
        T_Tip & Curl --> Obj_Surface["물체 표면 접촉 (Target Object)"]
        Obj_Surface --> Sensor["접촉력 발생 (MuJoCo solref / 실물 택타일)"]
        Sensor --> Compliance["어드미턴스 가상 댐퍼가 모터 각도를 양보하여 0.5N 안착"]
    end
```

### 3.2 물리 파라미터 및 하드웨어 링크 규격 (SSOT)

1. **엄지 손가락 (Finger 0)**:
   - `0-0` (Base) ~ `0-1` (Joint 1): 길이 $L_{t1} = 0.045\text{ m}$ (45 mm)
   - `0-1` ~ 끝점 (Tip): 길이 $L_{t2} = 0.035\text{ m}$ (35 mm)
   - 구동: STS3215 2개 직렬 직접 구동, 텐던 없음.

2. **일반 손가락 (Fingers 1~4)**:
   - MCP ~ PIP (Proximal Link): $L_{prox} = 0.040\text{ m}$ (40 mm)
   - PIP ~ DIP (Middle Link): $L_{mid} = 0.0158\text{ m}$ (15.8 mm)
   - DIP ~ 끝점 (Distal Link): $L_{dist} = 0.0155\text{ m}$ (15.5 mm)
   - 텐던 연동 커플링: $\theta_{DIP} = 0.88 \cdot \theta_{PIP}$ (생체모방 기구학적 종속)
   - PIP 복귀 스프링: 관절 마찰을 이기고 손가락을 펴주는 상시 신전 토크 인가.

3. **마찰 및 장력 특성 (논문 56번 공식 반영)**:
   - 텐던 재질: 고분자 폴리에틸렌 (Dura4 Braid), 마찰계수 $\mu \approx 0.2$
   - 라우팅 핀 마찰: $\eta_{routing} = e^{-\mu \theta} \approx 0.5 \sim 0.7$ (각도에 따라 30~50% 장력 손실)
   - 40KG 모터의 역할: **스프링 강성($F_{spring}$) + 핀 마찰 손실($\Delta F_{friction}$)을 이기고 손끝에 $5\sim 10\text{ N}$을 가하기 위한 충분한 토크 마진 확보**.

---

## 4. [동작 시퀀스 및 안전 인터락] (State Sequence & Interlock)

모터 간 충돌(STS3215 기어 파손)을 100% 방지하기 위한 소프트웨어 상태 머신(FSM) 타임라인입니다.

```mermaid
sequenceDiagram
    autonumber
    participant AI as 상위 AI / 제어기
    participant STS as STS3215 (MCP / 엄지)
    participant PWM as 40KG PWM 모터 (텐던)
    participant HW as 실물 기구부 & 센서

    Note over AI, HW: [모드 1: 파지 시퀀스 (Grasp Sequence)]
    AI->>STS: 1. MCP 목표 자세 각도 지령 송신 (접근 위치)
    STS->>HW: MCP 기저 관절 회전
    AI->>PWM: 2. 40KG 모터 PWM 구동 (텐던 권취 시작)
    PWM->>HW: 복귀 스프링을 누르며 PIP/DIP 말아쥐기
    HW-->>AI: 3. 촉각 접촉 감지 (Force >= 0.15N)
    AI->>AI: 4. Tactile Latch 발동 (목표각 고정 + 프리로드)
    AI->>STS: 5. 어드미턴스 보정 지령 인가 (0.50N 정밀 유지)

    Note over AI, HW: [모드 2: 해제 시퀀스 (Release Sequence - 절대 안전 수칙)]
    AI->>PWM: 1. [선행] 40KG 모터 릴리즈 (PWM = 0도 / 언와인딩)
    PWM->>HW: 텐던 장력 T = 0 N (슬랙 확보)
    HW->>HW: 2. 복귀 스프링에 의해 PIP/DIP 마디가 먼저 펴짐
    Note over STS: MCP에 걸려 있던 외란 토크(T * r) 완전 소멸!
    AI->>STS: 3. [후행] STS3215 MCP 0도 원복 명령
    STS->>HW: 무부하 상태에서 가볍고 안전하게 원복 완료
```

---

## 5. [전기 및 통신 파이프라인] (Hardware Wiring & Bus Architecture)

```mermaid
flowchart TD
    subgraph Host["Host Controller (Raspberry Pi 5 / Ubuntu 24.04 ROS2)"]
        ROS2["ROS2 Control Node (sts3215_hardware_node)"]
        UART_Port["/dev/ttyUSB0 (FTDI USB-to-UART, 1Mbps)"]
        I2C_Port["/dev/i2c-1 (I2C Bus, 400kHz)"]
        ROS2 <--> UART_Port
        ROS2 --> I2C_Port
    end

    subgraph SerialBus["Half-Duplex UART Daisy-Chain (7.4V ~ 12V)"]
        UART_Port <--> Board["TTL Half-Duplex 변환 보드"]
        Board <--> S0["STS3215 (ID 0: 엄지 Base)"]
        S0 <--> S1["STS3215 (ID 1: 엄지 Tip)"]
        S1 <--> S2["STS3215 (ID 2: 검지 MCP)"]
        S2 <--> S3["STS3215 (ID 3: 중지 MCP)"]
        S3 <--> S4["STS3215 (ID 4: 약지 MCP)"]
        S4 <--> S5["STS3215 (ID 5: 새끼 MCP)"]
    end

    subgraph PWMBus["I2C PWM 서보 제어기 (6.0V ~ 8.4V 독립 고전류 전원)"]
        I2C_Port --> PCA["PCA9685 16-Ch PWM 드라이버"]
        PCA --> P1["40KG PWM 모터 1 (검지 텐던)"]
        PCA --> P2["40KG PWM 모터 2 (중지 텐던)"]
        PCA --> P3["40KG PWM 모터 3 (약지 텐던)"]
        PCA --> P4["40KG PWM 모터 4 (새끼 텐던)"]
    end
```

> [!IMPORTANT]
> **전원 분리 원칙 (Power Isolation Rule)**:
> 40KG 모터 4개가 동시에 당길 때 발생하는 피크 전류(최대 $10\sim15\text{ A}$)가 STS3215 및 라즈베리파이 로직 전원으로 역류하지 않도록, **40KG 서보 전원(고전류 7.4V SMPS/배터리)**과 **제어기/STS3215 전원은 반드시 그라운드(GND)만 공통으로 묶고 VCC 라인은 완벽히 분리**해야 합니다.

---

## 6. 팀별 협업 인계 체크리스트 (Action Items for Teams)

1. **팀 1 (기구팀)**:
   - [x] 엄지손가락: STS3215 2개 직렬 직접 구동 확인 (텐던 불필요)
   - [x] 일반 손가락: PIP 상단 텐던 라우팅 및 복귀 스프링 장착 확인
   - [ ] 텐던이 MCP 관절 축을 통과할 때의 모멘트 암($r_{mcp}$) 정밀 치수(mm) 공유 필요.
2. **팀 2 (회로/전원팀)**:
   - [ ] STS3215 시리얼 라인(1Mbps)과 PCA9685 I2C 라인 배선 분리.
   - [ ] 40KG 모터 순간 피크 전류(15A) 대응 대용량 캐패시터 및 독립 전원 레일 구축.
3. **팀 3 (제어팀 - 우리)**:
   - [x] 1지 MVP 15만 회 훈련 및 촉각 래치 파지 알고리즘 검증 완료 ($0.5\text{ N}$ 안착, 파손율 $0\%$).
   - [x] ADR 004 (3D 뷰어 동결) 및 ADR 005 (텐던 연성 및 해제 시퀀스 락) 수립 완료.
   - [ ] C++ ROS2 노드에 `[40KG 슬랙 선행 -> STS 원복]` 시퀀스 인터락 및 피드포워드 슬랙 보상 구현.
4. **팀 4 (비전/상위팀)**:
   - [ ] 물체 위치 및 크기($1.5\text{cm} \sim 4.0\text{cm}$) 추정 토픽 규격 확정 (`/target_object_info`).
