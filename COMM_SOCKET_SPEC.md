# 다중 팀 협업용 그리퍼 소켓 통신 총괄 규격서 (COMM_SOCKET_SPEC)

본 문서는 **통신 코어(Communication Core, 사용자 전담)**를 중심으로, 로봇 핸드 시스템 개발에 참여하는 **4대 협업 팀(제어 팀, 촉각 인식 팀, 손/물체 비전 인식 팀, VLA 상위 인공지능 팀)** 간의 실시간 소켓 통신 규격을 정의한 공식 인터페이스 문서입니다.

---

## 1. 5대 입출력 소켓 채널 총괄표

| 채널 번호 | 포트 (Port) | 프로토콜 | 데이터 흐름 | 협업 대상 팀 | 전송 주기 | 주요 데이터 내용 |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **CH 1** | **`5555`** | UDP (TX) | Comm $\rightarrow$ Control | **저수준 제어 팀** | 100 Hz (10ms) | 14 관절 각도/속도, 10 모터 실제 토크 |
| **CH 2** | **`5556`** | UDP (RX) | Control $\rightarrow$ Comm | **저수준 제어 팀** | 최대 100 Hz | 10개 모터 지령 토크 (클램핑 $\pm 2.5\text{ Nm}$) |
| **CH 3** | **`5557`** | UDP (양방향) | Comm $\leftrightarrow$ Tactile | **촉각 인식 팀** | 100 ~ 200 Hz | 원시 텍셀 압력 행렬 $\leftrightarrow$ 슬립/접촉 이벤트 |
| **CH 4** | **`5558`** | UDP (RX) | Vision $\rightarrow$ Comm | **손/물체 비전 팀** | 30 ~ 60 Hz | 손목 6D 포즈, 손끝 3D 키포인트, 타깃 물체 포즈 |
| **CH 5** | **`5559`** | UDP (양방향) | Comm $\leftrightarrow$ VLA | **VLA 상위 지능 팀** | 10 ~ 30 Hz | 멀티모달 상태 관측치 $\leftrightarrow$ 작업 공간 목표 액션 |

---

## 2. 채널별 상세 패킷 규격 (JSON Schema)

### CH 1. 관절 텔레메트리 (Port `5555`: Comm Core $\rightarrow$ 저수준 제어 팀, 100Hz)
```json
{
  "seq": 1042,
  "timestamp": 1726712345.123,
  "status": "NORMAL",
  "q": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "dq": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "actuator_pos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "actuator_vel": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "actuator_torque": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "tactile_force": [0.0, 0.0, 0.0, 0.0, 0.0]
}
```

---

### CH 2. 모터 제어 지령 (Port `5556`: 저수준 제어 팀 $\rightarrow$ Comm Core, 100Hz)
```json
{
  "mode": "torque",
  "torques": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}
```
- **안전 규칙**: 100ms 동안 패킷 미수신 시 안전 워치독에 의해 전 모터 출력 $0.0\text{ Nm}$ 강제 차단.

---

### CH 3. 고해상도 촉각 센서 허브 (Port `5557`: Comm Core $\leftrightarrow$ 촉각 인식 팀)

#### A. 원시 텍셀 스트림 (Comm Core $\rightarrow$ 촉각 팀, 100~200Hz)
각 손가락 팁에 장착된 텍셀(Taxel) 어레이 원시 센서 데이터:
```json
{
  "seq": 1042,
  "timestamp": 1726712345.125,
  "taxels": {
    "thumb": [0.0, 0.12, 0.45, 0.89, 0.0, ...],   // 16개 텍셀 압력값
    "index": [0.0, 0.05, 0.33, 0.72, 0.0, ...],
    "middle": [0.0, 0.0, 0.10, 0.20, 0.0, ...],
    "ring": [0.0, 0.0, 0.0, 0.0, 0.0, ...],
    "pinky": [0.0, 0.0, 0.0, 0.0, 0.0, ...]
  }
}
```

#### B. 해석된 촉각 이벤트 피드백 (촉각 팀 $\rightarrow$ Comm Core)
촉각 인식 모델이 추론한 실시간 접촉 및 슬립 판정 결과:
```json
{
  "contact_detected": [true, true, false, false, false],  // 5개 손가락 접촉 여부
  "slip_detected": [false, false, false, false, false],     // 미끄러짐 감지 플래그
  "normal_forces_N": [1.45, 0.82, 0.0, 0.0, 0.0],          // 손끝 법선력 추정치 (N)
  "grasp_stability_score": 0.88                             // 파지 안정도 지수 (0.0 ~ 1.0)
}
```

---

### CH 4. 손 및 물체 비전 인식 스트림 (Port `5558`: 손 인식 팀 $\rightarrow$ Comm Core, 30~60Hz)
외부 광학 카메라 / Depth 센서 / 모션 트래커가 추적한 6D 포즈 정보:
```json
{
  "timestamp": 1726712345.130,
  "frame_id": "camera_optical_frame",
  "palm_pose": {
    "position": [0.120, -0.035, 0.250],                   // 손목/손바닥 중심 [X, Y, Z] (m)
    "orientation_quat": [0.0, 0.7071, 0.0, 0.7071]       // 쿼터니언 [qx, qy, qz, qw]
  },
  "fingertip_keypoints_3d": [
    [0.08, -0.02, 0.04],  // Thumb 3D
    [0.13, -0.01, 0.05],  // Index 3D
    [0.14,  0.00, 0.05],  // Middle 3D
    [0.13,  0.02, 0.05],  // Ring 3D
    [0.11,  0.04, 0.04]   // Pinky 3D
  ],
  "target_object": {
    "name": "can",
    "position": [0.150, 0.010, 0.220],
    "bounding_box_size": [0.065, 0.065, 0.120]
  },
  "confidence": 0.96
}
```

---

### CH 5. VLA 상위 지능 코어 (Port `5559`: Comm Core $\leftrightarrow$ VLA 인공지능 팀, 10~30Hz)

#### A. 멀티모달 상태 관측치 (Comm Core $\rightarrow$ VLA 모델)
관절 상태 + 비전 포즈 + 촉각 피드백을 단일 벡터로 번들링하여 VLA 입력으로 공급:
```json
{
  "seq": 1042,
  "timestamp": 1726712345.135,
  "q": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "dq": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "palm_pose": [0.12, -0.035, 0.25, 0.0, 0.7071, 0.0, 0.7071],
  "object_pose": [0.15, 0.01, 0.22, 0.0, 0.0, 0.0, 1.0],
  "tactile_summary": [1.45, 0.82, 0.0, 0.0, 0.0]
}
```

#### B. VLA 상위 액션 지령 (VLA 모델 $\rightarrow$ Comm Core)
VLA 모델이 언어 명령 및 비전에 기반하여 출력한 상위 태스크 지령:
```json
{
  "task": "pick_and_lift",
  "synergy_mode": "precision_pinch",
  "target_fingertip_waypoints": [
    [0.05, -0.02, 0.03],  // Thumb 목표 위치
    [0.09, -0.01, 0.02],  // Index 목표 위치
    [0.09,  0.00, 0.02],  // Middle 목표 위치
    [0.08,  0.02, 0.02],  // Ring 목표 위치
    [0.07,  0.04, 0.02]   // Pinky 목표 위치
  ],
  "max_contact_force_limit_N": 3.0
}
```
통신 코어는 이 VLA 지령을 저수준 제어팀(`Port 5555`)으로 릴레이하거나 내부 기구학 제어기로 전달하여 손가락이 목표 웨이포인트를 추종하도록 지시합니다.

---

## 3. 팀별 10줄 연결 스니펫

### A. 촉각 인식 팀 (Port 5557 연결)
```python
import socket, json
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 5557))
# 1. 원시 텍셀 수신
raw_data, _ = sock.recvfrom(8192)
taxels = json.loads(raw_data.decode())["taxels"]
# 2. 파지/슬립 판정 후 이벤트 송신
feedback = {"contact_detected": [True, True, False, False, False], "slip_detected": [False]*5}
sock.sendto(json.dumps(feedback).encode(), ("127.0.0.1", 5557))
```

### B. 손/물체 비전 인식 팀 (Port 5558 송신)
```python
import socket, json
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
vision_data = {
    "palm_pose": {"position": [0.12, -0.03, 0.25], "orientation_quat": [0, 0.707, 0, 0.707]},
    "target_object": {"name": "mug", "position": [0.15, 0.01, 0.22]}
}
sock.sendto(json.dumps(vision_data).encode(), ("127.0.0.1", 5558))
```

### C. VLA 상위 인공지능 팀 (Port 5559 연결)
```python
import socket, json
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 5559))
# 1. 멀티모달 관측치 수신
obs_bytes, _ = sock.recvfrom(4096)
obs = json.loads(obs_bytes.decode())
# 2. VLA 추론 후 상위 액션 지령 송신
action = {"task": "grasp", "synergy_mode": "pinch", "max_contact_force_limit_N": 2.5}
sock.sendto(json.dumps(action).encode(), ("127.0.0.1", 5559))
```
