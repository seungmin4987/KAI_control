#include <ros.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <ESP32Servo.h>
#include "FastInterruptEncoder.h"
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <custom_msg_pkg/ControlMsg.h>
#include <custom_msg_pkg/FeedbackMsg.h>
#include <TM1637Display.h>


//======================= 핀 번호 정의 =========================
#define ESTOP_PIN_1 27  // 리모컨(E-Stop) 버튼 핀 번호
#define ESTOP_PIN_2 25  // 폴링(E-Stop) 버튼 핀 번호
#define ASMS_MODE_PIN 33  // 자율주행/수동 모드 전환 스위치 핀 번호

#define BRAKE_SERVO_PIN 26  // 서보모터 제어 핀 번호
#define BRAKE_START_ANGLE 85 // 서보모터 초기 각도
#define SERVO_MAX_ANGLE 120  // 서보모터 최대 각도 제한

#define DEBOUNCE_DELAY 150  // 디바운스 지연 시간(ms)

#define MOTOR_PWM_PIN 5  // 모터 속도 제어 핀 번호
#define ENCODER_PIN_A 18  // 엔코더 A 핀 번호
#define ENCODER_PIN_B 19  // 엔코더 B 핀 번호

#define CLK 2
#define DI0 4
#define SEG_UNDERBAR SEG_D

#define WHEEL_ROUND 1.57 // 바퀴 둘레 [m]


const unsigned long control_time_interval = 100;  // 제어 시간 간격(0.1초) -> 10Hz
const float GEAR_RATIO = 4.25;  // 기어비 설정
const int ENCODER_RESOLUTION = 1024;  // 엔코더 분해능(한 바퀴당 펄스 수)

Encoder enc(ENCODER_PIN_A, ENCODER_PIN_B, SINGLE, 0);  // 엔코더 객체 생성
TM1637Display display(CLK, DI0);
//==============================================================

volatile float g_target_speed = 0.0;  // 제어 레퍼런스 속도값
volatile float g_current_speed = 0.0; // 현재 차량의 속도값

unsigned long g_lastMsgTime = 0;
bool g_pid_activate_flag = true;

volatile float g_kp = 90.0;
volatile float g_ki = 0.025;
volatile float g_kd = 20.0;

volatile float g_error = 0.0;

ros::NodeHandle nh;

custom_msg_pkg::FeedbackMsg pub_msg;
ros::Publisher pub("mcu_feedback_topic", &pub_msg);

void messageCb(const custom_msg_pkg::ControlMsg& msg) {
  g_target_speed = msg.target_speed;
  g_lastMsgTime = millis();
  g_pid_activate_flag = true;
  //콜백함수에서, 메시지를 받을때 pid 계산을 활성화함.
}

void pidTuningCb(const std_msgs::Float32MultiArray& msg) {
    g_kp = msg.data[0];
    g_ki = msg.data[1];
    g_kd = msg.data[2];
    Serial.print("New PID values: kp=");
    Serial.print(g_kp);
    Serial.print(", ki=");
    Serial.print(g_ki);
    Serial.print(", kd=");
    Serial.println(g_kd);
}

ros::Subscriber<custom_msg_pkg::ControlMsg> sub("control_topic", &messageCb);
ros::Subscriber<std_msgs::Float32MultiArray> pidSub("pid_tuning_topic", &pidTuningCb);

Servo brakeServo;  // 서보모터 객체 생성

const int motor_pwm_freq = 5000;
const int motor_pwm_resolution = 12;
const int motor_pwm_channel = 2;

enum Mode { AUTONOMOUS, EMERGENCY, MANUAL };  // 차량 모드 정의
volatile Mode g_currentMode = AUTONOMOUS;  // 현재 모드 상태 (초기값은 자율 주행 모드)
volatile Mode prevMode = EMERGENCY;  // 이전 모드 상태 저장
volatile bool g_estopActivated = false;  // 비상 정지(E-Stop) 활성화 상태
volatile bool g_asmsActivated = false;  // ASMS 활성화 상태

//======================== 함수 시그니처 =======================
void update_ASMS_mode(unsigned long current_time, const unsigned long debounce_delay);
float calculatePID(float target_rpm, float kp, float ki, float kd, bool pid_activate_flag);
void longitudinalControl(float target_rpm, float kp, float ki, float kd);
void displayFloatWithDecimal(float number);
int getBrakeAngle(int brake_input_raw, float k);
void displayFloatWithDecimal(float number);
//==============================================================

int getBrakeAngle(int brake_input_raw, float k) {
    // brake_input_raw의 범위는 0에서 4095입니다.
    // 이를 0에서 1로 정규화합니다.
    float brake_input = brake_input_raw / 4095.0;

    // 지수 함수를 사용하여 brake_input을 angle로 변환 (게인 k 사용)
    int angle = (int)((exp(k * brake_input) - 1) / (exp(k) - 1) * (SERVO_MAX_ANGLE-BRAKE_START_ANGLE));

    // 각도를 0도에서 max_angle로 제한
    angle = constrain(angle, 0, (SERVO_MAX_ANGLE-BRAKE_START_ANGLE)) + BRAKE_START_ANGLE;

    return angle;  // 계산된 브레이크 각도 반환
}


float calculatePID(float target_speed, float kp, float ki, float kd, bool pid_activate_flag) {
    static float last_error = 0.0;  // 이전 오차 값 저장 변수
    static float integral = 0.0;  // 적분 값 저장 변수

    // 적분항 클램핑 (예시로 +-1000 범위로 제한)
    //integral = constrain(integral, -1000, 1000);

    if(pid_activate_flag == false){
      last_error = 0.0;
      integral = 0.0;
      return 0.0;
    }

    g_error = target_speed - g_current_speed;  // 현재 RPM과 목표 RPM의 차이 계산 (오차)
    integral += g_error * control_time_interval;  // 적분 계산 (누적 오차)
    float derivative = (g_error - last_error) / control_time_interval;  // 미분 계산 (오차 변화율)
    float output = (kp * g_error) + (ki * integral) + (kd * derivative);  // PID 출력 계산
    last_error = g_error;  // 이전 오차 값 업데이트
    output = constrain(output, -4095, 4095);  // PWM 범위 제한
    return output;  // 계산된 PID 출력 반환
}

void longitudinalControl(float target_rpm, float kp, float ki, float kd) {
    int pid_output = calculatePID(g_target_speed, kp, ki, kd, g_pid_activate_flag);  // PID 제어 값 계산

    if(g_target_speed < 0.0){
      ledcWrite(motor_pwm_channel, 0);  // 모터 PWM 출력 0으로 설정
      brakeServo.write(SERVO_MAX_ANGLE);
      return;
      //목표속도값이 음수일 경우, 비상 정지상황임을 가정.
    }

    if (g_error >= 0) {  // PID 출력이 양수인 경우 (모터 제어)
        brakeServo.write(BRAKE_START_ANGLE);
        ledcWrite(motor_pwm_channel, pid_output);  // 모터 PWM 제어
        //Serial.print("motor_pwm: ");
        //Serial.println(pid_output);
    } else {  // PID 출력이 음수인 경우 (브레이크 제어)
        ledcWrite(motor_pwm_channel, 0);  // 모터 PWM 출력 0으로 설정
        int brake_input_raw = -g_error*300;  // 오차값 양수로 변경
        brakeServo.write(getBrakeAngle(brake_input_raw, 1.15));
    }
}

//======================== ASMS 모드 업데이트 ====================
  void update_ASMS_mode(unsigned long current_time, const unsigned long debounce_delay) {

  static unsigned long lastUpdateTime = 0;
  static bool lastLeverState = HIGH;

  // 디바운스 처리
  if (current_time - lastUpdateTime >= debounce_delay) {
    bool currentLeverState = digitalRead(ASMS_MODE_PIN);  // ASMS 스위치 상태 읽기    
    if (currentLeverState == LOW) {
      g_currentMode = MANUAL;
      Serial.println("Switching to MANUAL mode.");
    } 
    else {
      bool currentEStopState1 = digitalRead(ESTOP_PIN_1);
      bool currentEStopState2 = digitalRead(ESTOP_PIN_2);
      if (currentEStopState1 == LOW || currentEStopState2 == HIGH) {
        g_estopActivated = true;
        g_currentMode = EMERGENCY;
        Serial.println("E-Stop Activated! Switching to EMERGENCY mode.");
      }
      else if (currentEStopState1 == HIGH && currentEStopState2 == LOW) {
        g_estopActivated = false;
        g_currentMode = AUTONOMOUS;
        Serial.println("Switching to AUTONOMOUS mode.");
      }
    }
    lastUpdateTime = current_time;
  }
}

void displayFloatWithDecimal(float number) {
  int roundedNumber = (int)(number * 10 + 0.5);
  int integerPart = roundedNumber / 10;
  int decimalPart = roundedNumber % 10;

  uint8_t data[4] = {0x00, 0x00, 0x00, 0x00};

  if (integerPart >= 10) {
    data[0] = display.encodeDigit(integerPart / 10);
    data[1] = display.encodeDigit(integerPart % 10);
    data[2] = SEG_UNDERBAR;
    data[3] = display.encodeDigit(decimalPart);
  } else {
    data[1] = display.encodeDigit(integerPart);
    data[2] = SEG_UNDERBAR;
    data[3] = display.encodeDigit(decimalPart);
  }

  display.setSegments(data);
}


//======================== setup 함수 ==========================
void setup() {
    Serial.begin(57600);

    display.setBrightness(0x0f);

    nh.initNode();

    nh.subscribe(sub);
    nh.subscribe(pidSub);

    nh.advertise(pub);

    brakeServo.attach(BRAKE_SERVO_PIN, 500, 2400);  // 서보모터 핀 설정 및 초기화
    delay(1000);
    brakeServo.write(BRAKE_START_ANGLE);  // 서보모터 초기 위치 설정 (브레이크 해제)

    if (enc.init()) {  // 엔코더 초기화 시도
      Serial.println("Encoder Initialization OK");  // 초기화 성공 메시지 출력
    } else {
      Serial.println("Encoder Initialization Failed");  // 초기화 실패 메시지 출력
      while(1);  // 실패 시 무한 루프
    }

    // E-Stop 핀 설정
    pinMode(ESTOP_PIN_1, INPUT_PULLDOWN);  
    pinMode(ESTOP_PIN_2, INPUT_PULLUP); 
    // ASMS 핀 설정
    pinMode(ASMS_MODE_PIN, INPUT_PULLUP);

    ledcSetup(motor_pwm_channel, motor_pwm_freq, motor_pwm_resolution);  // 타이머 설정
    ledcAttachPin(MOTOR_PWM_PIN, motor_pwm_channel);       // 채널에 PWM 핀 연결

    if (digitalRead(ESTOP_PIN_2) == HIGH || digitalRead(ESTOP_PIN_1) == HIGH){
      g_currentMode = EMERGENCY;
    }
    else if (digitalRead(ESTOP_PIN_1) == LOW and digitalRead(ESTOP_PIN_2) == LOW){
      if (digitalRead(ASMS_MODE_PIN) == HIGH) {
          g_currentMode = AUTONOMOUS;
      } 
      else {
          g_currentMode = MANUAL;
      }
    }
}

//======================== loop 함수 ===========================
void loop() {
    static unsigned long lastUpdateTime = 0;
    unsigned long current_time = millis();
    static unsigned long last_debouncing_time = 0;

    // 시스템 상태 업데이트 (ASMS 모드)
    update_ASMS_mode(current_time, DEBOUNCE_DELAY);
    
    // 모드 변경 감지: EMERGENCY에서 다른 모드로 변경되었을 때 브레이크 초기화
    if (prevMode == EMERGENCY && g_currentMode != EMERGENCY) {
          brakeServo.write(BRAKE_START_ANGLE);  // 브레이크 초기 위치로 이동
      }

    // 주기적으로 실행할 작업 (100ms 주기)
    if (current_time - lastUpdateTime >= 100) {
        lastUpdateTime = current_time;

        Serial.print("Current Mode: ");
        long pulse_count = 0;  // pulse_count를 switch 외부에서 선언

        enc.loop();  // 엔코더 값 읽기
        pulse_count = enc.getTicks();  // 엔코더에서 펄스 수 읽기
        float speed_km_per_h = 1.3*2.0*0.5*0.8*((((pulse_count*0.1*1000)/(float) ENCODER_RESOLUTION)*WHEEL_ROUND *3.6/10.0)/3.0);
        g_current_speed = speed_km_per_h;

        if(speed_km_per_h >= 0){
          displayFloatWithDecimal(speed_km_per_h);
        }

  // 현재 속도 계산

        Serial.println(g_current_speed);
        enc.resetTicks();  // 엔코더 펄스 카운트 초기화

        switch (g_currentMode) {
            case AUTONOMOUS:
                Serial.println("AUTONOMOUS");
                longitudinalControl(g_target_speed, g_kp, g_ki, g_kd);
                break;
            case MANUAL:
                Serial.println("MANUAL");
                ledcWrite(motor_pwm_channel, 0); // 수동모드에서는 우발상황 방지를 위해 모터 스로틀 출력 0으로 하기
                delay(15);
                brakeServo.write(BRAKE_START_ANGLE); //브레이크 강제 해제
                delay(15);
                break;
            case EMERGENCY:
                Serial.println("EMERGENCY");
                ledcWrite(motor_pwm_channel,0);
                delay(15);
                brakeServo.write(SERVO_MAX_ANGLE);
                delay(15);
                break;
        }
        prevMode = g_currentMode;
        
        pub_msg.current_speed = g_current_speed;
        pub_msg.is_estop_activated = g_estopActivated;
        // 메시지 퍼블리시
        pub.publish(&pub_msg);
    }
    // ROS 통신 유지

    // 타임아웃 처리: 1초 동안 메시지를 받지 못하면 target_speed를 0으로 설정
    if (millis() - g_lastMsgTime > 1000 || g_currentMode == EMERGENCY) {  
        g_target_speed = 0;
        g_pid_activate_flag = false;
        //안전을 위해 목표토픽이 1초 이내로 전송되지 않는 경우 pid 계산을 비활성화 하고,
        //목표 속도도 0으로 초기화함.
    }

    nh.spinOnce();
    Serial.print("target_speed : ");
    Serial.println(g_target_speed);
}


