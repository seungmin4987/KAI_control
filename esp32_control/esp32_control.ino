#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "FastInterruptEncoder.h"
#include <ros.h>
#include <ESP32Servo.h>
#include <ACAN_ESP32.h>
#include <custom_msg_pkg/ControlMsg.h>   // 커스텀 메시지: ControlMsg
#include <custom_msg_pkg/FeedbackMsg.h>  // 커스텀 메시지: FeedbackMsg

//=======================상수 정의 ==============================
const unsigned long control_time_interval = 100;       // 0.1초 간격으로 제어
const unsigned long encoder_time_interval = 100;       // 0.1초 간격으로 엔코더 데이터 읽기
const unsigned long communication_time_interval = 100; // 0.1초 간격으로 통신
const float GEAR_RATIO = 5.0;                          // 기어비 1:5
const int ENCODER_RESOLUTION = 1024;                   // 엔코더 분해능
const int debounceDelay = 50;                          // 디바운스 지연 시간 (밀리초)

//=======================핀번호 정의==============================
#define MOTOR_PWM_PIN 2 // 모터 속도 제어 핀
#define BRAKE_SERVO_PIN 4 // 브레이크 서보 제어 핀
#define ENCODER_PIN_A 18 // 엔코더 A핀
#define ENCODER_PIN_B 19 // 엔코더 B핀
#define STEERING_CAN_TX 5
#define STEERING_CAN_RX 6
#define ESTOP_PIN 7 // E-Stop 버튼 핀 번호
#define ASMS_MODE_PIN 8
#define RECV_CH1_PIN 19  // CH1: 속도
#define BUTTON_PIN 25 // 토글 스위치 핀 번호 (자율주행/비상 모드 전환용)
#define LEVER_SWITCH_PIN 26 // 레버 스위치 핀 번호 (자율주행/수동 모드 전환용)
//==============================================================

enum Mode { AUTONOMOUS, EMERGENCY, MANUAL }; // 모드 정의
volatile Mode g_currentMode = AUTONOMOUS; // 현재 모드 상태 (초기값은 자율주행 모드)
volatile bool g_buttonPressed = false; // 버튼이 눌렸음을 표시

Encoder enc(ENCODER_PIN_A, ENCODER_PIN_B, SINGLE, 250); // 엔코더 설정
ros::NodeHandle nh;

custom_msg_pkg::FeedbackMsg feedback_msg;  // 피드백 메시지 객체
ros::Publisher pub("feedback_topic", &feedback_msg);  // 피드백 메시지 퍼블리셔

// 글로벌 변수
SemaphoreHandle_t g_xMutexCurrentRPM;  // 엔코더 RPM 관련 뮤텍스
SemaphoreHandle_t g_xMutexTargetValues;  // 목표 값 관련 뮤텍스
TaskHandle_t g_controlTaskHandle = NULL; // 제어 태스크 핸들 선언

volatile bool g_estopActivated = false; // E-Stop 활성화 상태

float g_current_rpm = 0.0;
float g_target_rpm = 0.0;   // 목표 RPM
int g_target_angle_int = 0; // 목표 핸들 각도
Servo g_brake_servo;  // 서보모터 객체 생성

//=======================함수 시그니처 ==========================
void vControlTask(void *pvParameters); // 태스크
void vEncoderTask(void *pvParameters);
void vCommunicationTask(void *pvParameters);
void longitudinalControl(float target_rpm, float kp, float ki, float kd); // 종방향 제어 함수
void lateralControl(int targetAngleInt); // 횡방향 제어 함수
float calculatePID(float target_rpm, float kp, float ki, float kd);
int getBrakeAngle(float pid_output); // 브레이크 룩업 테이블 함수
void stopControlTask(); // 제어 태스크 중단 및 안전 상태 설정
void resumeControlTask(); // 제어 태스크 재시작
void controlCallback(const custom_msg_pkg::ControlMsg& msg);  // 커스텀 메시지 콜백 함수
void IRAM_ATTR handleEStop();  // E-Stop 인터럽트 핸들러
void activateEStop();  // E-Stop 활성화 함수
void deactivateEStop();  // E-Stop 비활성화 함수
void IRAM_ATTR handleButtonPress(); // 토글 스위치 인터럽트 핸들러
void activateEmergencyMode();  // 긴급 모드 활성화 함수
void deactivateEmergencyMode();  // 긴급 모드 비활성화 함수
void deactivateAutonomousMode();  // 자율 모드 비활성화 함수 (수동 주행 모드 전환 시 호출)
void activateAutonomousMode();  // 자율 모드 활성화 함수 (자율주행 모드로 전환 시 호출)
//==============================================================

void setup() {
    Serial.begin(115200);

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
        while (1);
    }

    // 서보모터 초기화
    g_brake_servo.attach(BRAKE_SERVO_PIN);  // 서보모터 핀 설정
    g_brake_servo.write(0);  // 초기 위치 설정 (0도)

    // E-Stop 핀 초기화 및 인터럽트 설정
    pinMode(ESTOP_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), handleEStop, FALLING);

    // 토글 스위치 핀 초기화 및 인터럽트 설정
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);

    // 레버 스위치 핀 초기화
    pinMode(LEVER_SWITCH_PIN, INPUT_PULLUP);

    // FreeRTOS 태스크 생성
    xTaskCreatePinnedToCore(vControlTask, "ControlTask", 1024, NULL, 1, &g_controlTaskHandle, 0);   // 코어 0에 할당, 우선순위 1 (낮음)
    xTaskCreatePinnedToCore(vEncoderTask, "EncoderTask", 1024, NULL, 3, NULL, 1);  // 코어 1에 할당, 우선순위 3 (높음)
    xTaskCreatePinnedToCore(vCommunicationTask, "CommunicationTask", 1024, NULL, 2, NULL, 1); // 코어 1에 할당, 우선순위 2 (중간)
}

//======================= vControlTask 함수 =====================
void vControlTask(void *pvParameters) { 
    unsigned long last_time = 0;
    unsigned long current_time = 0;

    unsigned long lastButtonPressTime = 0; // 버튼의 마지막 눌린 시간
    unsigned long lastLeverSwitchTime = 0; // 레버 스위치의 마지막 전환 시간
    bool lastLeverState = HIGH; // 레버 스위치의 이전 상태

    // PID 게인 설정
    float kp = 1.0;
    float ki = 0.1;
    float kd = 0.01;

    for (;;) {
        // E-Stop이 활성화되면 제어 루프 중단
        if (g_estopActivated) {
            vTaskSuspend(NULL);  // 현재 태스크를 일시 중단
        }

        current_time = millis();
        if (current_time - last_time >= control_time_interval) {
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

            // 모드 전환 처리 (로컬 변수 사용)
            if (g_buttonPressed) {
                g_buttonPressed = false; // 상태 초기화

                if (g_currentMode == AUTONOMOUS) {
                    g_currentMode = EMERGENCY;
                    Serial.println("Switching to Emergency Mode");
                    activateEmergencyMode();
                } else if (g_currentMode == EMERGENCY) {
                    g_currentMode = AUTONOMOUS;
                    Serial.println("Switching to Autonomous Mode");
                    deactivateEmergencyMode();
                }
            }

            int currentLeverState = digitalRead(LEVER_SWITCH_PIN);

            if (currentLeverState != lastLeverState && (current_time - lastLeverSwitchTime > debounceDelay)) {
                lastLeverSwitchTime = current_time; // 마지막 전환 시간 업데이트

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
            lastLeverState = currentLeverState; // 이전 상태 업데이트

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
        }

        last_time = current_time;  // 시간 업데이트

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

            if (!nh.connected()) {
                Serial.println("Lost connection to ROS, stopping control task.");
                stopControlTask();  // 제어 태스크 중단 및 안전 상태 설정

                // ROS 재연결 시도
                while (!nh.connected()) {
                    nh.spinOnce();
                    Serial.println("Reconnecting to ROS...");
                    delay(1000);
                }
                Serial.println("Reconnected to ROS!");

                // 제어 태스크 재시작
                resumeControlTask();
            }

            // 엔코더 RPM 보호용 뮤텍스
            if (xSemaphoreTake(g_xMutexCurrentRPM, portMAX_DELAY) == pdTRUE) {  
                feedback_msg.current_rpm = g_current_rpm;  // 피드백 메시지에 현재 RPM 할당
                xSemaphoreGive(g_xMutexCurrentRPM);  // 뮤텍스 해제
            }

            pub.publish(&feedback_msg);  // 현재 엔코더 RPM 값 퍼블리시
            nh.spinOnce();  // ROS 통신 유지
        }

        last_time = current_time;  // 시간 업데이트

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

//======================= E-Stop 핸들러 함수 ====================
void IRAM_ATTR handleEStop() {
    g_estopActivated = true;  // E-Stop 활성화
    Serial.println("E-Stop Activated!");
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

//======================= 자율 모드 비활성화 함수 ====================
void deactivateAutonomousMode() {
    stopControlTask();
}

//======================= 자율 모드 활성화 함수 ====================
void activateAutonomousMode() {
    resumeControlTask();
}

//======================= 토글 스위치 인터럽트 핸들러 ====================
void IRAM_ATTR handleButtonPress() {
    unsigned long currentTime = millis();
    static unsigned long lastButtonPressTime = 0;

    // 디바운스 처리: 마지막 눌린 시간 이후 일정 시간이 지났을 때만 처리
    if (currentTime - lastButtonPressTime > debounceDelay) {
        g_buttonPressed = true; // 버튼이 눌렸음을 표시
        lastButtonPressTime = currentTime;
    }
}

void loop() {}
