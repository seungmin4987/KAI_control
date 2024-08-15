#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "FastInterruptEncoder.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <ESP32Servo.h>
#include <ACAN_ESP32.h>

//=======================상수 정의 ==============================
const unsigned long encoder_time_interval = 100;        // 0.1초 간격으로 엔코더 데이터 읽기
const unsigned long communication_time_interval = 100; // 0.1초 간격으로 통신
const unsigned long control_time_interval = 100;       // 0.1초 간격으로 제어
const float GEAR_RATIO = 5.0;                          // 기어비 1:5
const int ENCODER_RESOLUTION = 1024;                   // 엔코더 분해능

//=======================핀번호 정의==============================
#define MOTOR_PWM_PIN 2 // 모터 속도 제어 핀
#define BRAKE_SERVO_PIN 4 // 브레이크 서보 제어 핀
#define ENCODER_PIN_A 18 // 엔코더 A핀
#define ENCODER_PIN_B 19 // 엔코더 B핀
#define STEERING_CAN_TX 5
#define STEERING_CAN_RX 6
#define ESTOP_PIN 7
#define ASMS_MODE_PIN 8
#define RECV_CH1_PIN 19  // CH1: 속도
//==============================================================

Encoder enc(ENCODER_PIN_A, ENCODER_PIN_B, SINGLE, 250); // 엔코더 설정
ros::NodeHandle nh;
std_msgs::Float32 rpm_msg;
ros::Publisher pub("encoder_rpm", &rpm_msg);

SemaphoreHandle_t xMutex;  // 뮤텍스 핸들 선언

TaskHandle_t controlTaskHandle = NULL; // 제어 태스크 핸들 선언

float g_current_rpm = 0.0;
Servo brake_servo;  // 서보모터 객체 생성

float targetAngle = 0.0;
int targetAngleInt = 0;
unsigned long lastUpdateTime = 0;
bool isMotorActivated = false;

//=======================함수 시그니처 ==========================
void vEncoderTask(void *pvParameters);
void vCommunicationTask(void *pvParameters);
void vControlTask(void *pvParameters); //태스크

float calculatePID(float setpoint, float current_value, float kp, float ki, float kd, float &last_error, float &integral);
void longitudinalControl(float setpoint, float kp, float ki, float kd); // 제어 작업을 수행하는 함수
void lateralControl(int targetAngleInt); // 횡방향 제어 작업을 수행하는 함수
int getBrakeAngle(float pid_output); // 브레이크 룩업 테이블 함수
void stopControlTask(); // 제어 태스크 중단 및 안전 상태 설정
void resumeControlTask(); // 제어 태스크 재시작
void sendTargetAngleMsg(int targetAngleInt); // 목표 각도 CAN 메시지 전송 함수
void printCANSettings(const ACAN_ESP32_Settings& settings, uint32_t result); // CAN 설정 출력 함수
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

    // ROS 연결 시도
    while (!nh.connected()) {
        nh.spinOnce();
        Serial.println("Connecting to ROS...");
        delay(1000);  // 1초마다 연결 시도
    }
    Serial.println("Connected to ROS!");

    xMutex = xSemaphoreCreateMutex();  // 뮤텍스 생성
    if (xMutex == NULL) {
        Serial.println("Mutex creation failed");
        while (1);
    }

    // 서보모터 초기화
    brake_servo.attach(BRAKE_SERVO_PIN);  // 서보모터 핀 설정
    brake_servo.write(0);  // 초기 위치 설정 (0도)

    // FreeRTOS 태스크 생성
    xTaskCreatePinnedToCore(vEncoderTask, "EncoderTask", 1024, NULL, 3, NULL, 1);  // 코어 1에 할당, 우선순위 3 (높음)
    xTaskCreatePinnedToCore(vCommunicationTask, "CommunicationTask", 1024, NULL, 2, NULL, 1); // 코어 1에 할당, 우선순위 2 (중간)
    xTaskCreatePinnedToCore(vControlTask, "ControlTask", 1024, NULL, 1, &controlTaskHandle, 0);   // 코어 0에 할당, 우선순위 1 (낮음)
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

            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {  // 뮤텍스 획득
                g_current_rpm = (pulse_count / (float)ENCODER_RESOLUTION) * 60 * (1000 / (float)encoder_time_interval) * GEAR_RATIO;
                xSemaphoreGive(xMutex);  // 뮤텍스 해제
            }

            enc.resetTicks();  // 펄스 카운트를 초기화
        }

        last_time = current_time;  // 시간 업데이트

        // 주기적으로 실행
        vTaskDelay(1 / portTICK_PERIOD_MS); // 짧은 지연으로 다른 태스크에 CPU 시간을 양보
    }
}
//==============================================================

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

            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {  // 뮤텍스 획득
                rpm_msg.data = g_current_rpm;
                xSemaphoreGive(xMutex);  // 뮤텍스 해제
            }

            pub.publish(&rpm_msg);  // 엔코더 RPM 값 퍼블리시
            nh.spinOnce();  // ROS 통신 유지
        }

        last_time = current_time;  // 시간 업데이트

        // 주기적으로 실행
        vTaskDelay(1 / portTICK_PERIOD_MS); // 짧은 지연으로 다른 태스크에 CPU 시간을 양보
    }
}
//==============================================================

//======================= vControlTask 함수 =====================
void vControlTask(void *pvParameters) { 
    unsigned long last_time = 0;
    unsigned long current_time = 0;

    // PID 게인 설정 (예시)
    float kp = 1.0;
    float ki = 0.1;
    float kd = 0.01;
    float setpoint = 100.0; // 목표 RPM (예시)

    for (;;) {
        current_time = millis();
        if (current_time - last_time >= control_time_interval) {
            // 종방향 제어
            longitudinalControl(setpoint, kp, ki, kd);  // 종방향 제어 수행

            // 횡방향 제어
            if (!isMotorActivated && millis() - lastUpdateTime < 1000) {
                // 1초 동안 모터 활성화 메시지 전송
                sendTargetAngleMsg(0);  // 모터 활성화 메시지 전송
            } else {
                isMotorActivated = true; // 1초 후 모터 활성화 완료
                if (millis() - lastUpdateTime > lateral_control_interval) {
                    // 100ms마다 실행
                    float targetAngle = ((pulseIn(RECV_CH1_PIN, HIGH, 50000) - 1000.0) / 1100.0) * 10000;
                    targetAngleInt = (int)targetAngle;

                    lateralControl(targetAngleInt);  // 횡방향 제어 수행

                    Serial.println(targetAngleInt);
                    lastUpdateTime = millis();  // 시간 업데이트
                }
            }

            last_time = current_time;  // 시간 업데이트
        }

        // 주기적으로 실행
        vTaskDelay(1 / portTICK_PERIOD_MS); // 짧은 지연으로 다른 태스크에 CPU 시간을 양보
    }
}

//======================= longitudinalControl 함수 =======================
void longitudinalControl(float setpoint, float kp, float ki, float kd) {
    int pid_output = 0;
    float last_error = 0.0;
    float integral = 0.0;

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {  // 뮤텍스 획득
        pid_output = (int) calculatePID(setpoint, g_current_rpm, kp, ki, kd, last_error, integral);  // PID 계산
        xSemaphoreGive(xMutex);  // 뮤텍스 해제
    }

    // PID 출력값에 따른 동작 결정 (0도 양수로 간주)
    if (pid_output >= 0) {
        // PID 출력값이 양수일 때, 모터 제어
        analogWrite(MOTOR_PWM_PIN, pid_output); // PID 출력값으로 모터 제어
    } else {
        // PID 출력값이 음수일 때, 모터 PWM 출력 0으로 설정
        analogWrite(MOTOR_PWM_PIN, 0);
        
        // 브레이크 제어
        int brake_angle = getBrakeAngle(abs(pid_output)); // 룩업 테이블로 브레이크 각도 결정
        brake_servo.write(brake_angle);  // 서보모터로 브레이크 제어
    }
}
//==============================================================

//======================= lateralControl 함수 =======================
void lateralControl(int targetAngleInt) {
    sendTargetAngleMsg(targetAngleInt);  // 목표 각도 값 전송
}
//==============================================================

//======================= stopControlTask 함수 ====================
void stopControlTask() {
    if (controlTaskHandle != NULL) {
        vTaskSuspend(controlTaskHandle);  // 제어 태스크 중단
    }
    analogWrite(MOTOR_PWM_PIN, 0); // 모터 PWM 출력을 0으로 설정
}
//==============================================================

//======================= resumeControlTask 함수 ====================
void resumeControlTask() {
    if (controlTaskHandle != NULL) {
        vTaskResume(controlTaskHandle);  // 제어 태스크 재시작
    }
}
//==============================================================

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
//==============================================================

//======================= calculatePID 함수 =====================
float calculatePID(float setpoint, float current_value, float kp, float ki, float kd, float &last_error, float &integral) {
    float error = setpoint - current_value;
    integral += error * control_time_interval;
    float derivative = (error - last_error) / control_time_interval;
    float output = (kp * error) + (ki * integral) + (kd * derivative);
    last_error = error;
    return output;
}
//==============================================================

//======================= sendTargetAngleMsg 함수 =====================
void sendTargetAngleMsg(int targetAngleInt) {
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
//==============================================================

//======================= printCANSettings 함수 =====================
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
//==============================================================

void loop() {}
