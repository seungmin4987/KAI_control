#include <Arduino.h>
#include <TM1637Display.h>
#include <freertos/FreeRTOS.h>  // FreeRTOS 기본 헤더 파일
#include <freertos/task.h>  // FreeRTOS 태스크 관리 헤더 파일
#include <freertos/semphr.h>  // FreeRTOS 세마포어 관리 헤더 파일
#include "FastInterruptEncoder.h"  // 고속 인터럽트 엔코더 라이브러리
#include <ros.h>  // ROS 통신을 위한 헤더 파일
#include <ESP32Servo.h>  // ESP32에서 서보모터 제어를 위한 헤더 파일
#include <ACAN_ESP32.h>  // ESP32에서 CAN 통신을 위한 헤더 파일
#include <custom_msg_pkg/ControlMsg.h>   // 커스텀 메시지: ControlMsg를 위한 헤더 파일
#include <custom_msg_pkg/FeedbackMsg.h>  // 커스텀 메시지: FeedbackMsg를 위한 헤더 파일

// TM1637 디스플레이 핀 설정
#define CLK 25
#define DIO 26

TM1637Display display(CLK, DIO);

//=======================상수 정의 ========================================================
const unsigned long control_time_interval = 100;  // 제어 태스크 실행 간격(0.1초)
const unsigned long encoder_time_interval = 100;  // 엔코더 데이터 읽기 간격(0.1초)
const unsigned long communication_time_interval = 100;  // 통신 태스크 실행 간격(0.1초)
const float GEAR_RATIO = 5.0;  // 기어비 설정
const int ENCODER_RESOLUTION = 1024;  // 엔코더 분해능(한 바퀴당 펄스 수)
const int debounceDelay = 50;  // 스위치 디바운스 지연 시간(밀리초)

const int pwmFreq = 5000;     // PWM 주파수 (5kHz)
const int pwmResolution = 12; // PWM 해상도 (12비트: 0~4095 범위)

enum Mode { AUTONOMOUS, EMERGENCY, MANUAL };  // 차량 모드 정의(AUTONOMOUS: 자율주행, EMERGENCY: 비상, MANUAL: 수동)

//=======================================================================================

//=======================핀번호 정의=========================================================
#define MOTOR_PWM_PIN 2  // 모터 속도 제어 핀 번호
#define BRAKE_SERVO_PIN 4  // 브레이크 서보모터 제어 핀 번호
#define ENCODER_PIN_A 18  // 엔코더 A 핀 번호
#define ENCODER_PIN_B 19  // 엔코더 B 핀 번호
#define STEERING_CAN_TX 21  // 스티어링 CAN 통신 TX 핀 번호
#define STEERING_CAN_RX 22  // 스티어링 CAN 통신 RX 핀 번호
#define ESTOP_PIN 7  // 비상 정지(E-Stop) 버튼 핀 번호
#define ASMS_MODE_PIN 8  // 자율주행/수동 모드 전환 스위치 핀 번호
#define RESET_PIN 33  // 비상 정지 해제 스위치 핀 번호
//=========================================================================================

//========================전역 변수 및 객체 =================================================
Encoder enc(ENCODER_PIN_A, ENCODER_PIN_B, SINGLE, 250);  // 엔코더 객체 생성 (싱글 모드, 디바운스 시간 250us)
Servo brake_servo;  // 서보모터 객체 생성

ros::NodeHandle nh;  // ROS 노드 핸들러 객체 생성
custom_msg_pkg::FeedbackMsg feedback_msg;  // 피드백 메시지 객체 생성
ros::Publisher pub("feedback_topic", &feedback_msg);  // ROS 퍼블리셔 생성 (피드백 메시지를 'feedback_topic' 토픽에 퍼블리시)

// Critical Section을 위한 MUX 선언
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//========================함수 선언부 =================================================
void controlCallback(const custom_msg_pkg::ControlMsg& msg);  // 커스텀 메시지 콜백 함수 선언

// ROS Subscriber 객체를 별도로 정의
ros::Subscriber<custom_msg_pkg::ControlMsg> control_subscriber("control_topic", &controlCallback);

//==================================== 태스크 선언부 ==========================================
void vControlTask(void *pvParameters);  // 제어 태스크 함수 선언
void vEncoderTask(void *pvParameters);  // 엔코더 읽기 태스크 함수 선언
void vCommunicationTask(void *pvParameters);  // 통신 태스크 함수 선언

SemaphoreHandle_t g_xMutexCurrentRPM;  // 현재 RPM 보호용 세마포어 선언
SemaphoreHandle_t g_xMutexTargetValues;  // 목표 값 보호용 세마포어 선언
SemaphoreHandle_t modeSemaphore;  // 모드 전환 보호용 세마포어 선언
//==========================================================================================

//========================모드 전환 스위치=====================================================
volatile Mode g_currentMode = AUTONOMOUS;  // 현재 모드 상태 (초기값은 자율주행 모드)
volatile bool g_estopActivated = false;  // 비상 정지(E-Stop) 활성화 상태
//==========================================================================================

//======================센서 데이터 ==========================================================
float g_current_rpm = 0.0;  // 현재 RPM 값
//==========================================================================================

//======================목표 제어값===========================================================
float g_target_rpm = 0.0;  // 목표 RPM 값
int g_target_angle_int = 0;  // 목표 핸들 각도 값
//==========================================================================================

// 디스플레이에 RPM을 표시하는 함수
void displayRPM(float rpm) {
  display.clear();

  int roundedRPM = (int)(rpm * 100 + 0.5);  // 소수 둘째 자리에서 반올림
  int integerPart = roundedRPM / 100;       // 정수 부분
  int firstDecimalPart = (roundedRPM / 10) % 10;  // 첫 번째 소수 부분
  int secondDecimalPart = roundedRPM % 10;  // 두 번째 소수 부분

  uint8_t data[4] = {0x00, 0x00, 0x00, 0x00};  // 디스플레이 데이터를 초기화

  if (integerPart >= 10) {
    data[0] = display.encodeDigit(integerPart / 10);  // 10의 자리
    data[1] = display.encodeDigit(integerPart % 10) | 0b10000000;  // 1의 자리, 소수점 포함
    data[2] = display.encodeDigit(firstDecimalPart);  // 첫 번째 소수 자리
    data[3] = display.encodeDigit(secondDecimalPart); // 두 번째 소수 자리
  } else {
    data[0] = display.encodeDigit(integerPart);  // 1의 자리
    data[1] = display.encodeDigit(firstDecimalPart) | 0b10000000;  // 첫 번째 소수 자리, 소수점 포함
    data[2] = display.encodeDigit(secondDecimalPart);  // 두 번째 소수 자리
  }

  display.setSegments(data);  // 디스플레이에 데이터 전송
}

//=================================함수 선언부 ===============================================
void longitudinalControl(float target_rpm, float kp, float ki, float kd);  // 종방향 제어 함수 선언
float calculatePID(float target_rpm, float kp, float ki, float kd);  // PID 계산 함수 선언
int getBrakeAngle(float pid_output);  // 브레이크 각도 결정 함수 선언

void lateralControl(int targetAngleInt);  // 횡방향 제어 함수 선언

void setupCANCommunication(int rxPin, int txPin, uint32_t baudRate);  // CAN 통신 설정 함수 선언
void printCANSettings(const ACAN_ESP32_Settings& settings, uint32_t result);  // CAN 통신 설정 디버깅 출력 함수 선언

void IRAM_ATTR EStop_Activate_ISR();  // 비상 정지(E-Stop) 활성화 ISR 함수 선언
void IRAM_ATTR EStop_Deactivate_ISR();  // 비상 정지(E-Stop) 비활성화 ISR 함수 선언
void update_ASMS_Mode(bool* lastLeverState, unsigned long current_time, const unsigned long falling_debounce_delay);  // ASMS 모드 업데이트 함수 선언
Mode check_ASMS_mode();  // 현재 모드를 반환하는 함수 선언

// 자율주행 모드 활성화/비활성화 함수 선언
void deactivateAutonomousMode();  // 자율주행 모드 해제
void activateAutonomousMode();  // 자율주행 모드 재활성화
//===========================================================================================

void setup() {
    Serial.begin(115200);  // 시리얼 통신 초기화

    setupCANCommunication(STEERING_CAN_RX, STEERING_CAN_TX, 250000UL);  // CAN 통신 초기화 및 설정 (250kbps)

    display.setBrightness(0x0f);  // 디스플레이 밝기 설정

    ledcAttach(MOTOR_PWM_PIN, pwmFreq, pwmResolution);

    if (enc.init()) {  // 엔코더 초기화 시도
        Serial.println("Encoder Initialization OK");  // 초기화 성공 시 메시지 출력
    } else {
        Serial.println("Encoder Initialization Failed");  // 초기화 실패 시 메시지 출력
        while(1);  // 실패 시 무한 루프
    }

    nh.initNode();  // ROS 노드 초기화
    nh.advertise(pub);  // ROS 퍼블리셔 설정
    nh.subscribe(control_subscriber);  // ROS 구독자 설정

    while (!nh.connected()) {  // ROS에 연결될 때까지 대기
        nh.spinOnce();  // ROS 통신 유지
        Serial.println("Connecting to ROS...");  // 연결 시도 메시지 출력
        delay(1000);  // 1초 대기
    }
    Serial.println("Connected to ROS!");  // ROS 연결 성공 메시지 출력

    g_xMutexCurrentRPM = xSemaphoreCreateMutex();  // 현재 RPM 보호용 세마포어 생성
    g_xMutexTargetValues = xSemaphoreCreateMutex();  // 목표 값 보호용 세마포어 생성
    modeSemaphore = xSemaphoreCreateMutex();  // 모드 보호용 세마포어 생성

    if (g_xMutexCurrentRPM == NULL || g_xMutexTargetValues == NULL || modeSemaphore == NULL) {  // 세마포어 생성 실패 시
        Serial.println("Mutex creation failed");  // 오류 메시지 출력
        while (1);  // 무한 루프
    }

    brake_servo.attach(BRAKE_SERVO_PIN);  // 서보모터 핀 설정 및 초기화
    brake_servo.write(0);  // 서보모터 초기 위치 설정 (0도)

    pinMode(ESTOP_PIN, INPUT);  // 비상 정지(E-Stop) 핀 설정
    pinMode(RESET_PIN, INPUT);  // 비상 정지 해제 스위치 핀 설정
    attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), EStop_Activate_ISR, FALLING);  // E-Stop 인터럽트 설정
    attachInterrupt(digitalPinToInterrupt(RESET_PIN), EStop_Deactivate_ISR, FALLING);  // E-Stop 해제 인터럽트 설정

    pinMode(ASMS_MODE_PIN, INPUT);  // ASMS 모드 스위치 핀 설정

    // FreeRTOS 태스크 생성
    xTaskCreatePinnedToCore(vControlTask, "ControlTask", 1024, NULL, 1, NULL, 0);  // 제어 태스크 생성 (코어 0)
    xTaskCreatePinnedToCore(vEncoderTask, "EncoderTask", 1024, NULL, 2, NULL, 1);  // 엔코더 태스크 생성 (코어 1)
    xTaskCreatePinnedToCore(vCommunicationTask, "CommunicationTask", 1024, NULL, 1, NULL, 1);  // 통신 태스크 생성 (코어 1)
}

//======================= vControlTask 함수 =====================
void vControlTask(void *pvParameters) { 
    unsigned long last_time = 0;  // 마지막 실행 시간 저장 변수
    unsigned long current_time = 0;  // 현재 시간 저장 변수
    const unsigned long falling_debounce_delay = 500;  // 디바운스 지연 시간 설정

    bool lastLeverState = LOW;  // 레버 스위치의 이전 상태

    Mode current_mode;  // 현재 모드를 저장할 변수

    float kp = 1.0;  // PID 제어의 비례 게인
    float ki = 0.1;  // PID 제어의 적분 게인
    float kd = 0.01;  // PID 제어의 미분 게인

    for (;;) {  // 무한 루프
      current_time = millis();  // 현재 시간 업데이트

      update_ASMS_Mode(&lastLeverState, current_time, falling_debounce_delay);  // ASMS 모드 업데이트

      if((current_mode = check_ASMS_mode())== EMERGENCY){  // 비상 모드 체크
          brake_servo.write(30);  // 비상 정지 시 브레이크 30도 작동
          vTaskDelay(100 / portTICK_PERIOD_MS);  // 0.1초 대기 (비상 모드에서 CPU 부하 줄이기)
      }
      else if(current_mode == AUTONOMOUS){  // 자율주행 모드인 경우
        if (current_time - last_time >= control_time_interval) {  // 제어 시간 간격이 지났는지 확인

            float local_target_rpm;  // 로컬 목표 RPM
            int local_target_angle_int;  // 로컬 목표 각도

            if (xSemaphoreTake(g_xMutexTargetValues, portMAX_DELAY) == pdTRUE) {  // 목표 값 세마포어 획득
                local_target_rpm = g_target_rpm;  // 목표 RPM 가져오기
                local_target_angle_int = g_target_angle_int;  // 목표 각도 가져오기
                xSemaphoreGive(g_xMutexTargetValues);  // 세마포어 해제
            } 
            longitudinalControl(local_target_rpm, kp, ki, kd);  // 종방향 제어 수행
            //lateralControl(local_target_angle_int);  // 횡방향 제어 수행

            last_time = current_time;  // 마지막 실행 시간 업데이트
        }
      }
      else{  // 수동 모드인 경우
        brake_servo.write(0);  // 브레이크 해제
        ledcWrite(MOTOR_PWM_PIN, 0);  // 모터 PWM 출력 0으로 설정
        vTaskDelay(100 / portTICK_PERIOD_MS);  // 0.1초 대기 (수동 모드에서 CPU 부하 줄이기)
      } 

      vTaskDelay(1 / portTICK_PERIOD_MS);  // 1ms 대기(다른 태스크에게 CPU 시간 양보)
    }
}

//======================= vEncoderTask 함수 =====================
void vEncoderTask(void *pvParameters) {
    unsigned long last_time = 0;  // 마지막 실행 시간 저장 변수
    unsigned long current_time = 0;  // 현재 시간 저장 변수
    long pulse_count = 0;  // 엔코더 펄스 수 저장 변수

    for (;;) {  // 무한 루프
        enc.loop();  // 타이머 인터럽트 대신 루프에서 엔코더 값 읽기

        current_time = millis();  // 현재 시간 업데이트
        if (current_time - last_time >= encoder_time_interval) {  // 엔코더 읽기 간격이 지났는지 확인
            pulse_count = enc.getTicks();  // 엔코더에서 펄스 수 읽기

            if (xSemaphoreTake(g_xMutexCurrentRPM, portMAX_DELAY) == pdTRUE) {  // 현재 RPM 세마포어 획득
                g_current_rpm = (pulse_count / (float)ENCODER_RESOLUTION) * 60 * (1000 / (float)encoder_time_interval) * GEAR_RATIO;  // 현재 RPM 계산
                xSemaphoreGive(g_xMutexCurrentRPM);  // 세마포어 해제
            }

            enc.resetTicks();  // 엔코더 펄스 카운트 초기화

            last_time = current_time;  // 마지막 실행 시간 업데이트
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);  // 1ms 대기(다른 태스크에게 CPU 시간 양보)
    }
}

//======================= vCommunicationTask 함수 ===============
void vCommunicationTask(void *pvParameters) { 
    unsigned long last_time = 0;  // 마지막 실행 시간 저장 변수
    unsigned long current_time = 0;  // 현재 시간 저장 변수
    float local_rpm = 0;

    for (;;) {  // 무한 루프
        current_time = millis();  // 현재 시간 업데이트
        if (current_time - last_time >= communication_time_interval) {  // 통신 시간 간격이 지났는지 확인

            if (!nh.connected()) {  // ROS와 연결이 끊겼는지 확인
                deactivateAutonomousMode();  // 자율주행 모드 해제
                while (!nh.connected()) {  // 다시 연결될 때까지 대기
                    nh.spinOnce();  // ROS 통신 유지
                    Serial.println("Reconnecting to ROS...");  // 재연결 시도 메시지 출력
                    delay(1000);  // 1초 대기
                }
                activateAutonomousMode();  // 자율주행 모드 재활성화
            }

            if (xSemaphoreTake(g_xMutexCurrentRPM, portMAX_DELAY) == pdTRUE) {  // 현재 RPM 세마포어 획득
                local_rpm = g_current_rpm;  // 전역 변수에서 현재 RPM 값을 로컬 변수로 저장
                xSemaphoreGive(g_xMutexCurrentRPM);  // 세마포어 해제       
            }

            feedback_msg.current_rpm = local_rpm;  // 피드백 메시지에 로컬 변수의 RPM 값 저장
            displayRPM(local_rpm);
            pub.publish(&feedback_msg);  // 피드백 메시지 퍼블리시
            nh.spinOnce();  // ROS 통신 유지

            last_time = current_time;  // 마지막 실행 시간 업데이트
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);  // 1ms 대기(다른 태스크에게 CPU 시간 양보)
    }
}


//======================= 커스텀 메시지 콜백 함수 =====================
void controlCallback(const custom_msg_pkg::ControlMsg& msg) {
    if (xSemaphoreTake(g_xMutexTargetValues, portMAX_DELAY) == pdTRUE) {  // 목표 값 세마포어 획득
        g_target_rpm = msg.target_rpm;  // 수신된 메시지에서 목표 RPM 값 설정
        g_target_angle_int = msg.target_angle;  // 수신된 메시지에서 목표 각도 설정
        xSemaphoreGive(g_xMutexTargetValues);  // 세마포어 해제
    }
}

//======================= 종방향 제어 함수 =======================
void longitudinalControl(float target_rpm, float kp, float ki, float kd) {
    int pid_output = calculatePID(target_rpm, kp, ki, kd);  // PID 제어 값 계산

    if (pid_output >= 0) {  // PID 출력이 양수인 경우 (모터 제어)
        brake_servo.write(0);
        ledcWrite(MOTOR_PWM_PIN, pid_output);  // 모터 PWM 제어
    } else {  // PID 출력이 음수인 경우 (브레이크 제어)
        ledcWrite(MOTOR_PWM_PIN, 0);  // 모터 PWM 출력 0으로 설정
        int brake_angle = getBrakeAngle(abs(pid_output));  // 브레이크 각도 계산
        brake_servo.write(brake_angle);  // 서보모터로 브레이크 제어
    }
}

//======================= 횡방향 제어 함수 =======================
void lateralControl(int targetAngleInt) {
    CANMessage frame;  // CAN 메시지 객체 생성
    frame.id = 0x06000001;  // 메시지 ID 설정
    frame.ext = true;  // 확장 ID 사용 설정
    frame.rtr = false;  // 데이터 프레임 설정
    frame.len = 8;  // 데이터 길이 설정

    frame.data[0] = 0x23;  // 데이터 바이트 설정 (명령어)
    frame.data[1] = 0x02;  // 데이터 바이트 설정 (명령어)
    frame.data[2] = 0x20;  // 데이터 바이트 설정 (명령어)
    frame.data[3] = 0x01;  // 데이터 바이트 설정 (명령어)
    frame.data[4] = (targetAngleInt >> 24) & 0xFF;  // 목표 각도 상위 바이트 설정
    frame.data[5] = (targetAngleInt >> 16) & 0xFF;  // 목표 각도 중상위 바이트 설정
    frame.data[6] = (targetAngleInt >> 8) & 0xFF;  // 목표 각도 중하위 바이트 설정
    frame.data[7] = targetAngleInt & 0xFF;  // 목표 각도 하위 바이트 설정

    if (ACAN_ESP32::can.tryToSend(frame)) {  // CAN 메시지 전송 시도
        Serial.println("데이터 전송 성공");  // 전송 성공 시 메시지 출력
        Serial.print("전송된 데이터: ");  // 전송된 데이터 출력
        for (int i = 0; i<8; i++) {  // 데이터 바이트 출력
            Serial.print(frame.data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}

//======================= getBrakeAngle 함수 ====================
int getBrakeAngle(float pid_output) {
    int angle = 0;  // 기본 브레이크 각도 0도 (브레이크 해제)

    switch ((int)pid_output) {  // PID 출력에 따라 브레이크 각도 설정
        case 0 ... 9:
            angle = 0;  // PID 출력이 0~9일 때 브레이크 각도 0도
            break;
        case 10 ... 19:
            angle = 15;  // PID 출력이 10~19일 때 브레이크 각도 15도
            break;
        case 20 ... 29:
            angle = 30;  // PID 출력이 20~29일 때 브레이크 각도 30도
            break;
        default:
            angle = 30;  // PID 출력이 30 이상일 때 브레이크 각도 30도
            break;
    }

    return angle;  // 계산된 브레이크 각도 반환
}

//======================= calculatePID 함수 =====================
float calculatePID(float target_rpm, float kp, float ki, float kd) {
    static float last_error = 0.0;  // 이전 오차 값 저장 변수
    static float integral = 0.0;  // 적분 값 저장 변수

    float current_rpm = 0.0;  // 현재 RPM 값 저장 변수
    if (xSemaphoreTake(g_xMutexCurrentRPM, portMAX_DELAY) == pdTRUE) {  // 현재 RPM 세마포어 획득
        current_rpm = g_current_rpm;  // 현재 RPM 값 가져오기
        xSemaphoreGive(g_xMutexCurrentRPM);  // 세마포어 해제
    }

    float error = target_rpm - current_rpm;  // 현재 RPM과 목표 RPM의 차이 계산 (오차)
    integral += error * control_time_interval;  // 적분 계산 (누적 오차)
    float derivative = (error - last_error) / control_time_interval;  // 미분 계산 (오차 변화율)
    float output = (kp * error) + (ki * integral) + (kd * derivative);  // PID 출력 계산
    last_error = error;  // 이전 오차 값 업데이트
    return output;  // 계산된 PID 출력 반환
}

//======================= E-Stop 스위치 ISR ====================

void IRAM_ATTR EStop_Debounce(bool activate) {
    static unsigned long last_time = 0;  // 마지막 인터럽트 처리 시간 저장 변수
    unsigned long current_time = millis();  // 현재 시간 저장 변수
    if (current_time - last_time >= debounceDelay) {  // 디바운스 지연 시간 체크
        g_estopActivated = activate;  // E-Stop 활성화 상태 업데이트
        last_time = current_time;  // 마지막 처리 시간 업데이트
    }
}

void IRAM_ATTR EStop_Activate_ISR() {
    portENTER_CRITICAL_ISR(&mux);  // 인터럽트 비활성화
    EStop_Debounce(true);  // E-Stop 활성화 처리
    portEXIT_CRITICAL_ISR(&mux);  // 인터럽트 활성화
}

void IRAM_ATTR EStop_Deactivate_ISR() {
    portENTER_CRITICAL_ISR(&mux);  // 인터럽트 비활성화
    EStop_Debounce(false);  // E-Stop 비활성화 처리
    portEXIT_CRITICAL_ISR(&mux);  // 인터럽트 활성화

    if (xSemaphoreTakeFromISR(modeSemaphore, NULL)) {
        g_currentMode = AUTONOMOUS;  // 자율주행 모드로 복귀
        xSemaphoreGiveFromISR(modeSemaphore, NULL);
    }
}

//======================== 폴링 스위치 ==========================
void update_ASMS_Mode(bool* lastLeverState, unsigned long current_time, const unsigned long falling_debounce_delay) {
    static unsigned long last_time = 0;  // 마지막 스위치 처리 시간 저장 변수
    if(current_time - last_time >= falling_debounce_delay){  // 디바운스 지연 시간 체크
      bool currentLeverState = digitalRead(ASMS_MODE_PIN);  // 현재 스위치 상태 읽기
      if (currentLeverState != *lastLeverState) {  // 스위치 상태가 변경되었는지 확인
          if (currentLeverState == LOW && g_currentMode == AUTONOMOUS) {  // 자율주행 모드에서 수동 모드로 전환
              g_currentMode = MANUAL;  // 모드 업데이트
              *lastLeverState = currentLeverState;  // 스위치 상태 업데이트
          } else if (currentLeverState == HIGH && g_currentMode == MANUAL) {  // 수동 모드에서 자율주행 모드로 전환
              g_currentMode = AUTONOMOUS;  // 모드 업데이트
              *lastLeverState = currentLeverState;  // 스위치 상태 업데이트
          }
      }
    }
}

Mode check_ASMS_mode(){  // 현재 모드 체크 함수
    Mode current_mode = g_currentMode;  // 현재 모드 저장
    if(g_estopActivated) current_mode = EMERGENCY;  // E-Stop이 활성화된 경우 비상 모드로 전환
    return current_mode;  // 현재 모드 반환
}

//======================= CAN 통신 설정 함수 정의 =====================
void setupCANCommunication(int rxPin, int txPin, uint32_t baudRate) {
  ACAN_ESP32_Settings settings(baudRate);  // CAN 통신 설정 객체 생성

  settings.mRxPin = (gpio_num_t)rxPin;  // RX 핀 설정
  settings.mTxPin = (gpio_num_t)txPin;  // TX 핀 설정

  const uint32_t ret = ACAN_ESP32::can.begin(settings);  // CAN 통신 초기화

  printCANSettings(settings, ret);  // CAN 통신 설정 디버깅 출력
}

//======================= CAN 통신 설정 출력 함수 정의 =====================
void printCANSettings(const ACAN_ESP32_Settings& settings, uint32_t result) {
  if (result == 0) {    
    Serial.print("Bit Rate prescaler: ");
    Serial.println(settings.mBitRatePrescaler);  // 비트율 프리스케일러 출력
    Serial.print("Time Segment 1:     ");
    Serial.println(settings.mTimeSegment1);  // 시간 세그먼트 1 출력
    Serial.print("Time Segment 2:     ");
    Serial.println(settings.mTimeSegment2);  // 시간 세그먼트 2 출력
    Serial.print("RJW:                ");
    Serial.println(settings.mRJW);  // 재동기화 점퍼 윈도우 출력
    Serial.print("Triple Sampling:    ");
    Serial.println(settings.mTripleSampling ? "yes" : "no");  // 트리플 샘플링 여부 출력
    Serial.print("Actual bit rate:    ");
    Serial.print(settings.actualBitRate());  // 실제 비트율 출력
    Serial.println(" bit/s");
    Serial.print("Exact bit rate ?    ");
    Serial.println(settings.exactBitRate() ? "yes" : "no");  // 정확한 비트율 여부 출력
    Serial.print("Distance            ");
    Serial.print(settings.ppmFromDesiredBitRate());  // 비트율과의 거리 출력
    Serial.println(" ppm");
    Serial.print("Sample point:       ");
    Serial.print(settings.samplePointFromBitStart());  // 샘플링 지점 출력
    Serial.println("%");
    Serial.println("Configuration OK!");
  } else {
    Serial.print("Configuration error 0x");
    Serial.println(result, HEX);  // 설정 오류 시 에러 코드 출력
  }
}

void deactivateAutonomousMode() {
    Serial.println("Autonomous mode deactivated");
    // 자율주행 모드 비활성화 관련 로직 추가
}

void activateAutonomousMode() {
    Serial.println("Autonomous mode activated");
    // 자율주행 모드 활성화 관련 로직 추가
}

void loop() {}  // 메인 루프(실제로는 사용되지 않음, 모든 작업은 태스크에서 수행)

