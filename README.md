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

# integrated_controller 추가 설명

## 소개
이 코드는 ESP32를 기반으로 자율주행 차량의 제어 유닛을 구현한 코드입니다. ROS(로봇 운영 체제)를 활용한 종방향 제어기능과 동적 PID 파라미터 튜닝 기능을 포함하고 있습니다. 차량은 **자율 주행(AUTONOMOUS)**, **수동(MANUAL)**, **비상 정지(EMERGENCY)** 모드를 지원하며, 속도 제어, 브레이크 제어, 모드 전환 등을 외부 입력에 따라 관리합니다.

---

## 코드 구조
### 주요 기능
1. **속도 제어**:
   - PID 제어기를 이용하여 목표 속도를 유지.
   - 엔코더 데이터를 기반으로 실시간 속도 계산.
2. **브레이크 제어**:
   - 지수 함수를 사용하여 동적 브레이크 각도 계산 및 적용.
3. **모드 전환**:
   - 외부 스위치와 E-Stop 신호를 기반으로 세 가지 모드 전환 관리.
4. **ROS 통합**:
   - ROS 메시지를 이용해 제어 명령을 구독하고, 현재 차량 상태를 퍼블리시.

---

## 핀 번호 정의
| 핀 번호         | 역할                         |
|----------------|-----------------------------|
| `ESTOP_PIN_1`  | E-Stop (리모컨) 버튼 입력 핀 |
| `ESTOP_PIN_2`  | E-Stop (폴링) 버튼 입력 핀   |
| `ASMS_MODE_PIN`| 자율/수동 모드 전환 스위치   |
| `BRAKE_SERVO_PIN` | 서보모터 제어 핀           |
| `MOTOR_PWM_PIN`  | 모터 PWM 제어 핀           |
| `ENCODER_PIN_A`  | 엔코더 A 핀                |
| `ENCODER_PIN_B`  | 엔코더 B 핀                |

---

## 주요 함수 설명

### 1. `calculatePID`
- **기능**: PID 제어기를 이용해 목표 속도와 현재 속도의 차이를 계산하여 출력값을 반환.
- **인자**:
  - `target_speed`: 목표 속도.
  - `kp`, `ki`, `kd`: PID 게인.
  - `pid_activate_flag`: PID 활성화 여부.
- **출력**: PID 계산 결과값 (PWM 출력 범위: -4095 ~ 4095).

### 2. `longitudinalControl`
- **기능**: PID 출력에 따라 차량의 종방향(전후방) 제어를 수행.
- **동작**:
  - 목표 속도가 양수면 모터 제어, 음수면 브레이크를 작동.
  - 브레이크 각도는 `getBrakeAngle` 함수에서 계산.

### 3. `update_ASMS_mode`
- **기능**: 현재 차량의 모드(AUTONOMOUS, MANUAL, EMERGENCY)를 스위치와 E-Stop 신호로 업데이트.
- **동작**:
  - 디바운싱 처리를 통해 입력 상태를 안정적으로 감지.
  - 모드 전환 시 브레이크 초기화.

### 4. `displayFloatWithDecimal`
- **기능**: TM1637 디스플레이에 소수점 포함 숫자를 출력.
- **동작**:
  - 속도를 소수점 한 자리까지 표시.
  - 디스플레이 모듈과의 SPI 통신으로 숫자 출력.

---

## ROS 메시지 통신
### 1. **구독**
- **토픽명**: `control_topic`
  - **메시지 타입**: `custom_msg_pkg::ControlMsg`
  - **내용**: 목표 속도(`target_speed`) 명령 수신.
- **토픽명**: `pid_tuning_topic`
  - **메시지 타입**: `std_msgs::Float32MultiArray`
  - **내용**: PID 게인(kp, ki, kd) 실시간 업데이트.

### 2. **퍼블리시**
- **토픽명**: `mcu_feedback_topic`
  - **메시지 타입**: `custom_msg_pkg::FeedbackMsg`
  - **내용**:
    - `current_speed`: 현재 속도.
    - `is_estop_activated`: E-Stop 활성 상태.

---

## 모드 설명
1. **AUTONOMOUS (자율주행 모드)**:
   - PID 제어기를 이용하여 목표 속도 유지.
2. **MANUAL (수동 모드)**:
   - 모터 출력 0으로 설정하여 수동 모드 전환.
   - 브레이크 강제로 해제.
3. **EMERGENCY (비상 정지 모드)**:
   - 모든 동작을 정지하고 브레이크를 최대 각도로 설정.

---

## 사용 방법
1. 하드웨어 연결:
   - 핀 번호에 맞게 하드웨어 구성.
2. ROS 환경 설정:
   - ROS 메시지 토픽(`control_topic`, `mcu_feedback_topic`) 설정.
3. 코드 업로드:
   - Arduino IDE 또는 PlatformIO를 사용하여 ESP32에 코드 업로드.
4. 시스템 실행:
   - 모드 스위치와 E-Stop 버튼을 사용해 차량 제어.

---

## 참고
- 이 코드는 ESP32와 ROS 기반 자율주행 차량 개발을 목적으로 작성되었습니다.
- 기능 개선 및 추가를 위해 코드 리뷰와 테스트를 권장합니다.


