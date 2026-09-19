# 14 DoF 그리퍼 다중 팀 통합 데이터 플로우 & 아키텍처

본 문서는 **통신 코어(Communication Core, 사용자 전담)**를 허브로 하여, **저수준 제어 팀, 촉각 인식 팀, 손/물체 비전 인식 팀, VLA 상위 지능 팀**이 유기적으로 데이터를 주고받는 **전체 시스템 데이터 플로우 및 입출력 포트 명세서**입니다.

---

## 1. 전체 다중 팀 데이터 플로우 다이어그램 (Mermaid Flowchart)

```mermaid
flowchart TB
    subgraph VLA_LAYER["[1. 상위 인공지능 계층 - VLA 팀]"]
        VLA["VLA 상위 정책 코어 (OpenVLA / Octo / RT-2)<br/>언어 명령 및 비전 기반 상위 작업 계획"]
    end

    subgraph PERCEPTION_LAYER["[2. 인식 계층 - 비전 & 촉각 팀]"]
        VISION["손 / 물체 비전 인식 팀<br/>• 손목 6D 포즈 (T_wrist)<br/>• 5개 손끝 3D 키포인트<br/>• 물체 6D 바운딩 박스"]
        TACTILE["촉각 인식 팀 (Tactile Hub)<br/>• 텍셀 압력 행렬 (5지 x 16텍셀)<br/>• 슬립(Slip) 감지 & 접촉 판정<br/>• 파지 안정도 지수 추론"]
    end

    subgraph COMM_LAYER["[3. 통신 코어 (사용자 전담) - IPC 통합 라우터]"]
        direction TB
        ROUTER["통신 코어 허브 (socket_server.py)<br/>• Non-blocking UDP 멀티플렉서<br/>• 안전 워치독 (100ms Failsafe)<br/>• 토크 한계 클램퍼 (±2.5 Nm)<br/>• 조인트 동적 매핑 (joint_mapping.yaml)"]
    end

    subgraph CTRL_LAYER["[4. 저수준 기구학 제어 계층 - 제어 팀 (Pure Python)]"]
        CTRL["파이썬 제어 코어 (Control Core)<br/>• 14 DoF 기구학 수식 (FK / IK)<br/>• 텐던 결합 연산 (θ_DIP = 0.8 θ_PIP)<br/>• 목표 궤적 추종 & 임피던스 제어"]
    end

    subgraph HW_LAYER["[5. 물리 / 시뮬레이션 계층]"]
        CAD["단일 에셋 (assets/)<br/>• meshes/*.STL (44개)<br/>• models/*.xml"]
        PHYSICS["MuJoCo 물리 엔진 /<br/>실물 모터 버스 (CAN/UART)"]
        CAD --> PHYSICS
    end

    %% Multi-team Socket Port Connections
    PHYSICS -- "관절 위치(14), 속도(14), 모터 피드백(10)" --> ROUTER
    ROUTER -- "data.ctrl[:] 모터 토크 인가" --> PHYSICS

    ROUTER == "Port 5555 (UDP 100Hz): 관절/모터 텔레메트리" ==> CTRL
    CTRL == "Port 5556 (UDP 100Hz): 모터 토크 지령" ==> ROUTER

    PHYSICS -- "원시 텍셀 압력" --> ROUTER
    ROUTER == "Port 5557 TX (UDP 100Hz): 원시 텍셀 스트림" ==> TACTILE
    TACTILE == "Port 5557 RX (UDP): 접촉/슬립 판정 이벤트" ==> ROUTER

    VISION == "Port 5558 (UDP 30~60Hz): 손/물체 6D 포즈 & 키포인트" ==> ROUTER

    ROUTER == "Port 5559 TX (UDP 20Hz): 멀티모달 관측 번들 (관절+비전+촉각)" ==> VLA
    VLA == "Port 5559 RX (UDP): 상위 액션 (손끝 목표 웨이포인트/시너지)" ==> ROUTER

    classDef comm fill:#e0e7ff,stroke:#4f46e5,stroke-width:2px;
    classDef ctrl fill:#dcfce7,stroke:#16a34a,stroke-width:2px;
    classDef perc fill:#fef3c7,stroke:#d97706,stroke-width:2px;
    classDef vla fill:#fce7f3,stroke:#db2777,stroke-width:2px;
    classDef hw fill:#f1f5f9,stroke:#64748b,stroke-width:2px;

    class COMM_LAYER comm;
    class CTRL_LAYER ctrl;
    class PERCEPTION_LAYER perc;
    class VLA_LAYER vla;
    class HW_LAYER hw;
```

---

## 2. 5대 통신 포트 및 인터페이스 총괄표

| 포트 (Port) | 프로토콜 | 전송 방향 | 협업 대상 팀 | 통신 주기 | 데이터 포맷 | 주요 역할 |
| :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **`5555`** | UDP | Comm $\rightarrow$ Control | **저수준 제어 팀** | 100 Hz (10ms) | JSON | 14개 관절 각도/속도, 10개 모터 실제 토크 |
| **`5556`** | UDP | Control $\rightarrow$ Comm | **저수준 제어 팀** | 최대 100 Hz | JSON | 10개 모터 지령 토크 (100ms 워치독 보호) |
| **`5557`** | UDP | Comm $\leftrightarrow$ Tactile | **촉각 인식 팀** | 100 ~ 200 Hz | JSON | 원시 텍셀 압력 행렬 $\leftrightarrow$ 슬립/접촉 이벤트 |
| **`5558`** | UDP | Vision $\rightarrow$ Comm | **손/물체 비전 팀** | 30 ~ 60 Hz | JSON | 손목 6D 포즈, 손끝 3D 키포인트, 타깃 물체 포즈 |
| **`5559`** | UDP | Comm $\leftrightarrow$ VLA | **VLA 상위 지능 팀** | 10 ~ 30 Hz | JSON | 멀티모달 상태 관측치 $\leftrightarrow$ 상위 작업 액션 지령 |

---

## 3. 실시간 통합 제어 시퀀스 (Sequence Diagram)

```mermaid
sequenceDiagram
    autonumber
    participant VLA as VLA 상위 코어 (Port 5559)
    participant Vis as 비전 인식 팀 (Port 5558)
    participant Tac as 촉각 인식 팀 (Port 5557)
    participant Comm as 통신 코어 (Comm Hub)
    participant Ctrl as 저수준 제어 팀 (Port 5555/5556)
    participant Phys as 물리 엔진 / 실물 모터

    Note over Vis,Phys: [1] 감각 입력 수집 및 전송
    Vis->>Comm: 손목 6D 포즈 및 물체 좌표 전송 (Port 5558, 30Hz)
    Phys->>Comm: 원시 텍셀 압력 + 관절 센서 피드백 추출
    Comm->>Tac: 원시 텍셀 압력 스트리밍 (Port 5557, 100Hz)
    Tac->>Comm: 슬립(Slip) 감지 및 접촉 판정 피드백 (Port 5557)

    Note over Comm,VLA: [2] VLA 상위 추론 루프 (20Hz)
    Comm->>VLA: 멀티모달 관측 번들 (관절 상태 + 비전 포즈 + 촉각 요약)
    VLA->>Comm: 상위 액션 출력 (손끝 3D 목표 웨이포인트 및 최대 접촉력)

    Note over Comm,Ctrl: [3] 저수준 100Hz 실시간 제어 루프
    Comm->>Ctrl: 관절 텔레메트리 (q[14], dq[14]) 전송 (Port 5555)
    Ctrl->>Comm: 기구학/임피던스 계산 후 모터 토크 (torques[10]) 전송 (Port 5556)

    Note over Comm,Phys: [4] 액추에이션 및 안전 가드
    Comm->>Phys: 토크 한계 클램핑 (±2.5 Nm) 후 data.ctrl[:] 에 주입 및 모터 구동
```
