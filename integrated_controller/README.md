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
