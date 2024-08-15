#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "FastInterruptEncoder.h"
#include <ros.h>
#include <ESP32Servo.h>
#include <ACAN_ESP32.h>
#include <custom_msg_pkg/ControlMsg.h>   // 커스텀 메시지: ControlMsg
#include <custom_msg_pkg/FeedbackMsg.h>  // 커스텀 메시지: FeedbackMsg

//=======================상수 정의 ========================================================

const unsigned long control_time_interval = 100;       // 0.1초 간격으로 제어
const unsigned long encoder_time_interval = 100;       // 0.1초 간격으로 엔코더 데이터 읽기
const unsigned long communication_time_interval = 100; // 0.1초 간격으로 통신
const float GEAR_RATIO = 5.0;                          // 기어비 1:5
const int ENCODER_RESOLUTION = 1024;                   // 엔코더 분해능
const int debounceDelay = 50;                          // 스위치 디바운스 지연 시간 (밀리초)

enum Mode { AUTONOMOUS, EMERGENCY, MANUAL }; // 모드 정의

//=======================================================================================


//=======================핀번호 정의=========================================================
#define MOTOR_PWM_PIN 2 // 모터 속도 제어 핀
#define BRAKE_SERVO_PIN 4 // 브레이크 서보 제어 핀
#define ENCODER_PIN_A 18 // 엔코더 A핀
#define ENCODER_PIN_B 19 // 엔코더 B핀
#define STEERING_CAN_TX 16
#define STEERING_CAN_RX 17
#define ESTOP_PIN 7 // E-Stop 버튼 핀 번호 (자율주행/비상 모드 전환용)
#define ASMS_MODE_PIN 8 // 레버 스위치 핀 번호 (자율주행/수동 모드 전환용)
//=========================================================================================


//=========================================================================================
Encoder enc(ENCODER_PIN_A, ENCODER_PIN_B, SINGLE, 250); // 엔코더 생성자
Servo g_brake_servo;  // 서보모터 객체 생성

ros::NodeHandle nh; // ros 노드 핸들러
custom_msg_pkg::FeedbackMsg feedback_msg;  // 피드백 메시지 객체
ros::Publisher pub("feedback_topic", &feedback_msg);  // 피드백 메시지 퍼블리셔
//=========================================================================================


//==================================== 태스크 선언부 ==========================================
void vControlTask(void *pvParameters); // 제어 태스크
void vEncoderTask(void *pvParameters); // 엔코더 읽기 태스크 
void vCommunicationTask(void *pvParameters); // 통신 태스크

SemaphoreHandle_t g_xMutexCurrentRPM;  // 엔코더 RPM 관련 뮤텍스
SemaphoreHandle_t g_xMutexTargetValues;  // 목표 값(목표 RPM, 목표 각도) 관련 뮤텍스
TaskHandle_t g_controlTaskHandle = NULL; // 제어 태스크 핸들 선언
//==========================================================================================


//========================모드 전환 스위치=====================================================
volatile Mode g_currentMode = AUTONOMOUS; // 현재 모드 상태 (초기값은 자율주행 모드)
volatile bool g_estopActivated = false; // E-Stop 활성화 상태
//==========================================================================================


//======================센서 데이터 ==========================================================
float g_current_rpm = 0.0; // 현재 RPM
//==========================================================================================


//======================목표 제어값===========================================================
float g_target_rpm = 0.0;   // 목표 RPM
int g_target_angle_int = 0; // 목표 핸들 각도
//==========================================================================================


//=======================함수 시그니처 ========================================================
void longitudinalControl(float target_rpm, float kp, float ki, float kd); // 종방향 제어 함수
float calculatePID(float target_rpm, float kp, float ki, float kd);
int getBrakeAngle(float pid_output); // 브레이크 룩업 테이블 함수

void lateralControl(int targetAngleInt); // 횡방향 제어 함수

void controlCallback(const custom_msg_pkg::ControlMsg& msg);  // 커스텀 메시지 콜백 함수

void setupCANCommunication(int rxPin, int txPin, uint32_t baudRate); // 캔통신 활성화
void printCANSettings(const ACAN_ESP32_Settings& settings, uint32_t result); // 캔통신 세팅값 디버깅 출력

void activateEmergencyMode();  // 긴급 모드 활성화 함수
void deactivateEmergencyMode();  // 긴급 모드 비활성화 함수

void deactivateAutonomousMode();  // 자율 모드 비활성화 함수 (수동 주행 모드 전환 시 호출)
void activateAutonomousMode();  // 자율 모드 활성화 함수 (자율주행 모드로 전환 시 호출)

void stopControlTask(); // 제어 태스크 중단 함수
void resumeControlTask(); // 제어 태스크 재시작 함수 
void IRAM_ATTR handleEStop();  // E-Stop ISR
//===========================================================================================

void setup() {
    Serial.begin(115200);
    // 캔통신 초기화 및 설정 (예: RX: 17번 핀, TX: 16번 핀, 250kbps 통신 속도)
    setupCANCommunication(STEERING_CAN_RX, STEERING_CAN_TX, 250000UL); 
    //추후 여기에 스티어링 모터 활성화 기능 추가할 예정

    if (enc.init()) {
        Serial.println("Encoder Initialization OK");
    } else {
        Serial.println("Encoder Initialization Failed");
        while(1);
    }

    nh.initNode();
    nh.advertise(pub);
    nh.subscribe(ros::Subscriber<custom_msg_pkg::ControlMsg>("control_topic", &controlCallback));

    // ROS 연결 시도
    while (!nh.connected()) {
        nh.spinOnce();
        Serial.println("Connecting to ROS...");
        delay(1000);  // 1초마다 연결 시도
    }
    Serial.println("Connected to ROS!");

    g_xMutexCurrentRPM = xSemaphoreCreateMutex();  // 엔코더 RPM 관련 뮤텍스 생성
    g_xMutexTargetValues = xSemaphoreCreateMutex();  // 목표 값 관련 뮤텍스 생성

    if (g_xMutexCurrentRPM == NULL || g_xMutexTargetValues == NULL) {
        Serial.println("Mutex creation failed");
        //나중에 피드백 메시지로 로그 띄우기 시리얼 프린트가 아니라
        while (1);
    }

    //pinMode(MOTOR_PWM_PIN, OUTPUT); // 아날로그 라이트는 핀모드 설정 안해도 됨

    // 서보모터 초기화
    g_brake_servo.attach(BRAKE_SERVO_PIN); // 50Hz로 서보모터 제어, 가능하면 높은 주파수도 사용해보기
    g_brake_servo.write(0);  // 초기 위치 설정 (0도)

    // E-Stop 핀 초기화 및 인터럽트 설정
    pinMode(ESTOP_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), handleEStop, FALLING);

    // ASMS 스위치 핀 초기화
    pinMode(ASMS_MODE_PIN, INPUT_PULLUP);

    // FreeRTOS 태스크 생성
    xTaskCreatePinnedToCore(vControlTask, "ControlTask", 1024, NULL, 1, &g_controlTaskHandle, 0);   // 코어 0에 할당, 우선순위 1(낮음)
    xTaskCreatePinnedToCore(vEncoderTask, "EncoderTask", 1024, NULL, 2, NULL, 1);  // 코어 1에 할당, 우선순위 2 (높음)
    xTaskCreatePinnedToCore(vCommunicationTask, "CommunicationTask", 1024, NULL, 1, NULL, 1); // 코어 1에 할당, 우선순위 1 (낮음)
}

//======================= vControlTask 함수 =====================
void vControlTask(void *pvParameters) { 
    unsigned long last_time = 0;
    unsigned long current_time = 0;

    bool lastLeverState = LOW; // 레버 스위치의 이전 상태

    // PID 게인 설정
    float kp = 1.0;
    float ki = 0.1;
    float kd = 0.01;

    for (;;) {
        current_time = millis();
        check_EMERGENCY_mode();

        if (current_time - last_time >= control_time_interval) {
            check_ASMS_Mode(&lastLeverState);

            float local_target_rpm;
            int local_target_angle_int;

            // 목표 값 보호용 뮤텍스
            if (xSemaphoreTake(g_xMutexTargetValues, portMAX_DELAY) == pdTRUE) {
                local_target_rpm = g_target_rpm;
                local_target_angle_int = g_target_angle_int;
                xSemaphoreGive(g_xMutexTargetValues);
            }

            // 종방향 제어 수행
            longitudinalControl(local_target_rpm, kp, ki, kd);
            // 횡방향 제어 수행
            lateralControl(local_target_angle_int);

            last_time = current_time;  // 시간 업데이트
        }
        // 주기적으로 실행
        vTaskDelay(1 / portTICK_PERIOD_MS); // 짧은 지연으로 다른 태스크에 CPU 시간을 양보
    }
}

//======================= vEncoderTask 함수 =====================
void vEncoderTask(void *pvParameters) {
    unsigned long last_time = 0;
    unsigned long current_time = 0;
    long pulse_count = 0;

    for (;;) {
        enc.loop(); // 타이머 인터럽트 대신 루프에서 호출

        current_time = millis();
        if (current_time - last_time >= encoder_time_interval) {
            pulse_count = enc.getTicks();  // 엔코더에서 펄스 수 읽기

            // 엔코더 RPM 보호용 뮤텍스
            if (xSemaphoreTake(g_xMutexCurrentRPM, portMAX_DELAY) == pdTRUE) {  
                g_current_rpm = (pulse_count / (float)ENCODER_RESOLUTION) * 60 * (1000 / (float)encoder_time_interval) * GEAR_RATIO;
                xSemaphoreGive(g_xMutexCurrentRPM);  // 뮤텍스 해제
            }

            enc.resetTicks();  // 펄스 카운트를 초기화

            last_time = current_time;  // 시간 업데이트
        }
        // 주기적으로 실행
        vTaskDelay(1 / portTICK_PERIOD_MS); // 짧은 지연으로 다른 태스크에 CPU 시간을 양보
    }
}

//======================= vCommunicationTask 함수 ===============
void vCommunicationTask(void *pvParameters) { 
    unsigned long last_time = 0;
    unsigned long current_time = 0;

    for (;;) {
        current_time = millis();
        if (current_time - last_time >= communication_time_interval) {

            if (!nh.connected()) { // PC와 연결 해제시 예외처리
                deactivateAutonomousMode();  // 수동 주행 모드 돌입
                while (!nh.connected()) {
                    nh.spinOnce();
                    Serial.println("Reconnecting to ROS...");
                    delay(1000);
                }
                activateAutonomousMode();   // 자율주행 모드 재시작
            }

            // 엔코더 RPM 변수 보호용 뮤텍스
            if (xSemaphoreTake(g_xMutexCurrentRPM, portMAX_DELAY) == pdTRUE) {  
                feedback_msg.current_rpm = g_current_rpm;  // 피드백 메시지에 현재 RPM 할당
                xSemaphoreGive(g_xMutexCurrentRPM);  // 뮤텍스 해제
            }

            pub.publish(&feedback_msg);  // 현재 엔코더 RPM 값 퍼블리시
            nh.spinOnce();  // ROS 통신 유지

            last_time = current_time;  // 시간 업데이트
        }
        // 주기적으로 실행
        vTaskDelay(1 / portTICK_PERIOD_MS); // 짧은 지연으로 다른 태스크에 CPU 시간을 양보
    }
}

//======================= 커스텀 메시지 콜백 함수 =====================
void controlCallback(const custom_msg_pkg::ControlMsg& msg) {
    // 목표 값 보호용 뮤텍스
    if (xSemaphoreTake(g_xMutexTargetValues, portMAX_DELAY) == pdTRUE) {
        g_target_rpm = msg.target_rpm;
        g_target_angle_int = msg.target_angle;
        xSemaphoreGive(g_xMutexTargetValues);
    }
}

//======================= 종방향 제어 함수 =======================
void longitudinalControl(float target_rpm, float kp, float ki, float kd) {
    int pid_output = calculatePID(target_rpm, kp, ki, kd);  // PID 계산

    if (pid_output >= 0) {
        analogWrite(MOTOR_PWM_PIN, pid_output); // 모터 제어
    } else {
        analogWrite(MOTOR_PWM_PIN, 0); // 모터 PWM 출력 0으로 설정
        int brake_angle = getBrakeAngle(abs(pid_output)); // 브레이크 각도 결정
        g_brake_servo.write(brake_angle);  // 서보모터로 브레이크 제어
    }
}


//======================= 횡방향 제어 함수 =======================
void lateralControl(int targetAngleInt) {
    // 횡방향 제어: CAN 메시지 전송
    CANMessage frame;
    frame.id = 0x06000001;
    frame.ext = true; // 확장 ID 사용 여부 설정
    frame.rtr = false; // 데이터 프레임
    frame.len = 8; // 보낼 데이터의 길이

    // 바이트 순서를 맞추기 위해 명령어와 데이터를 결합
    frame.data[0] = 0x23;
    frame.data[1] = 0x02;
    frame.data[2] = 0x20;
    frame.data[3] = 0x01;
    frame.data[4] = (targetAngleInt >> 24) & 0xFF;
    frame.data[5] = (targetAngleInt >> 16) & 0xFF;
    frame.data[6] = (targetAngleInt >> 8) & 0xFF;
    frame.data[7] = targetAngleInt & 0xFF;

    // 전송
    if (ACAN_ESP32::can.tryToSend(frame)) {
        Serial.println("데이터 전송 성공");
        Serial.print("전송된 데이터: ");
        for (int i = 0; i < 8; i++) {
            Serial.print(frame.data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}

//======================= getBrakeAngle 함수 ====================
int getBrakeAngle(float pid_output) {
    int angle = 0; // 기본값 0도 (브레이크 해제)

    switch ((int)pid_output) {
        case 0 ... 9:
            angle = 0; // pid_output이 0에서 9 사이일 때 브레이크 각도 0도
            break;
        case 10 ... 19:
            angle = 15; // pid_output이 10에서 19 사이일 때 브레이크 각도 15도
            break;
        case 20 ... 29:
            angle = 30; // pid_output이 20에서 29 사이일 때 브레이크 각도 30도
            break;
        default:
            angle = 30; // pid_output이 30 이상일 때도 브레이크 각도 30도
            break;
    }

    return angle;
}

//======================= calculatePID 함수 =====================
float calculatePID(float target_rpm, float kp, float ki, float kd) {
    static float last_error = 0.0;
    static float integral = 0.0;

    // 엔코더 RPM 보호용 뮤텍스
    float current_rpm = 0.0;
    if (xSemaphoreTake(g_xMutexCurrentRPM, portMAX_DELAY) == pdTRUE) {
        current_rpm = g_current_rpm;
        xSemaphoreGive(g_xMutexCurrentRPM);  // 뮤텍스 해제
    }

    float error = target_rpm - current_rpm;
    integral += error * control_time_interval;
    float derivative = (error - last_error) / control_time_interval;
    float output = (kp * error) + (ki * integral) + (kd * derivative);
    last_error = error;
    return output;
}

// 캔 통신 초기화 및 설정 함수 정의
void setupCANCommunication(int rxPin, int txPin, uint32_t baudRate) {
  // 캔통신의 통신속도를 설정하는 부분
  ACAN_ESP32_Settings settings(baudRate);

  // 실제로 통신할거기 때문에 루프백모드를 설정하면 안된다
  // settings.mRequestedCANMode = ACAN_ESP32_Settings::LoopBackMode;

  // RX와 TX 핀 설정
  settings.mRxPin = (gpio_num_t)rxPin;  // 여기서 rxPin은 정수값 17
  settings.mTxPin = (gpio_num_t)txPin;  // 여기서 txPin은 정수값 16

  // 통신속도와 연결핀을 기준으로 캔통신 초기화
  const uint32_t ret = ACAN_ESP32::can.begin(settings);

  // 캔통신 설정을 시리얼 모니터에 출력
  printCANSettings(settings, ret);
}

// 캔 통신 설정 출력 함수 정의
void printCANSettings(const ACAN_ESP32_Settings& settings, uint32_t result) {
  if (result == 0) {    
    Serial.print("Bit Rate prescaler: ");
    Serial.println(settings.mBitRatePrescaler);
    Serial.print("Time Segment 1:     ");
    Serial.println(settings.mTimeSegment1);
    Serial.print("Time Segment 2:     ");
    Serial.println(settings.mTimeSegment2);
    Serial.print("RJW:                ");
    Serial.println(settings.mRJW);
    Serial.print("Triple Sampling:    ");
    Serial.println(settings.mTripleSampling ? "yes" : "no");
    Serial.print("Actual bit rate:    ");
    Serial.print(settings.actualBitRate());
    Serial.println(" bit/s");
    Serial.print("Exact bit rate ?    ");
    Serial.println(settings.exactBitRate() ? "yes" : "no");
    Serial.print("Distance            ");
    Serial.print(settings.ppmFromDesiredBitRate());
    Serial.println(" ppm");
    Serial.print("Sample point:       ");
    Serial.print(settings.samplePointFromBitStart());
    Serial.println("%");
    Serial.println("Configuration OK!");
  } else {
    Serial.print("Configuration error 0x");
    Serial.println(result, HEX);    
  }
}

//======================= E-Stop 스위치 ISR ====================
void IRAM_ATTR handleEStop() {
    g_estopActivated = true;  // E-Stop 활성화
    Serial.println("E-Stop Activated!");
}

//======================== 폴링 스위치 ==========================
void check_ASMS_Mode(bool* lastLeverState) {

    int currentLeverState = digitalRead(ASMS_MODE_PIN);

    if (currentLeverState != *lastLeverState) {
        if (currentLeverState == LOW && g_currentMode == AUTONOMOUS) {
            g_currentMode = MANUAL;
            Serial.println("Switching to Manual Driving Mode");
            deactivateAutonomousMode();
        } else if (currentLeverState == HIGH && g_currentMode == MANUAL) {
            g_currentMode = AUTONOMOUS;
            Serial.println("Switching to Autonomous Driving Mode");
            activateAutonomousMode();
        }
    }
    *lastLeverState = currentLeverState; // 이전 상태 업데이트
}

void check_EMERGENCY_mode(){
    // E-Stop이 활성화되면 제어 루프 중단
    if (g_estopActivated) {
        activateEmergencyMode();  // 현재 태스크를 일시 중단
    }
}

//======================= stopControlTask 함수 ====================
void stopControlTask() {
    if (g_controlTaskHandle != NULL) {
        vTaskSuspend(g_controlTaskHandle);  // 제어 태스크 중단
    }
    analogWrite(MOTOR_PWM_PIN, 0); // 모터 PWM 출력을 0으로 설정
}

//======================= resumeControlTask 함수 ====================
void resumeControlTask() {
    if (g_controlTaskHandle != NULL) {
        vTaskResume(g_controlTaskHandle);  // 제어 태스크 재시작
    }
}

//======================= 긴급 모드 활성화 함수 ====================
void activateEmergencyMode() {
    stopControlTask();
    g_brake_servo.write(90); // 브레이크 작동 (각도는 예시)
}

//======================= 긴급 모드 비활성화 함수 ====================
void deactivateEmergencyMode() {
    resumeControlTask(); // 자율 모드로 복귀
}

//=============== 자율 모드 비활성화 함수(수동 모드 진입) ====================
void deactivateAutonomousMode() {
    stopControlTask();
}

//======================= 자율 모드 활성화 함수 ====================
void activateAutonomousMode() {
    resumeControlTask();
}

void loop() {}
