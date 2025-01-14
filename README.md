# KAI Control Project

## 각 패키지 설명

### 1. `control_setup`
- **1_open_can_port.sh**: CAN 포트를 활성화하는 스크립트입니다.
- **2_activate_can_bridge.sh**: CAN 브릿지 노드를 실행하는 스크립트입니다.
- **3_activate_rosserial.sh**: ROS Serial을 실행하는 스크립트입니다.
- **사용 방법**: 위 스크립트들은 **이름 순서대로 실행**하면 됩니다.

---

### 2. `KAI_control`
- **esp32_control/esp32_control.ino**: 통합 종-횡 제어 코드입니다.
- 현재는 더 효율적인 방식으로 구현된 **종방향 노드**와 **횡방향 노드**가 분리되었으므로 사용되지 않습니다.

---

### 3. `integrated_controller`
- **esp32_control/esp32_control.ino**: 종방향 제어기 코드입니다.
- 대회때 사용한 종방향 제어, 모드 전환 스위치,각종 액추에이터 제어 코드.

---

### 4. `ros_kai_ws/src`
#### **can_bridge_pkg**
- **스티어링 모터와 ROS 간 통신**을 위한 CAN 브릿지 패키지입니다.
- 주요 구성:
  - `CMakeLists.txt` 및 `package.xml`: 패키지 설정 파일.
  - `launch/hardware_activate.launch`: CAN 브릿지 실행을 위한 런치 파일.
  - `script/open_can_port.sh`: CAN 포트를 활성화하는 스크립트.
  - `src`: CAN 브릿지 소스 파일들(`can_bridge.cpp`, `can_bridge_estop_deactivate.cpp` 등).

#### **custom_msg_pkg**
- 테스트 및 제어 메시지 관련 패키지입니다.
- 주요 구성:
  - `msg/ControlMsg.msg`: 제어 메시지 정의.
  - `msg/FeedbackMsg.msg`: 피드백 메시지 정의.
  - `scripts/sine_input.py`: 테스트용 사인 입력 스크립트.
  - `scripts/step_input.py`: 테스트용 스텝 입력 스크립트.
  - `scripts/tune_pid.sh`: PID 튜닝 스크립트.
  - `scripts/view_rqt_plot.sh`: RQT 플롯 뷰어 실행 스크립트.

---

## 프로젝트 목적
이 프로젝트는 자율주행 차량의 통신과 제어를 다룹니다.  
- **CAN 브릿지**: 스티어링 모터와 ROS 간의 통신을 중계.
- **테스트 스크립트**: 사인파 및 스텝 입력을 통해 제어기 성능 테스트.

---

### 사용 방법
1. `control_setup` 내부 스크립트를 **순서대로 실행**:
   ```bash
   ./1_open_can_port.sh
   ./2_activate_can_bridge.sh
   ./3_activate_rosserial.sh
   
2. `ROS 노드 실행` 
    ```bash
    roslaunch can_bridge_pkg hardware_activate.launch
    
3. `테스트 스크립트 실행`

    ```bash
    python3 ros_kai_ws/src/custom_msg_pkg/scripts/sine_input.py
    python3 ros_kai_ws/src/custom_msg_pkg/scripts/step_input.py

## 캔 브릿지 노드 추가 설명

1. 랙피니언 구조의 스티어링 시스템 이므로 steering ratio가 매 각도구간 마다 달라진다.
   -> **회귀 분석을 통해 구한 스티어링 궤적 함수를 이용하여, 낮은 각도 구간에서 회전에 대한 가중치를 높게 설정함**
   -> **스티어링 각도 구간에 상관없이, 항상 일정한 조향 경향성을 유지할 수 있다.**

2. 제어 토픽을 서브스크라이브 하면, 자동 비트 연산을 통해 스티어링 모터로 캔 신호를 전송한다.

3. 스티어링 모터에서 발생하는 일정주기의 신호(하트비트 신호)를, 읽어들이고 이를 피드백 토픽으로 변경한다.
   -> **현재 모터의 각도를 알 수 있어 디버그에 용이함**

