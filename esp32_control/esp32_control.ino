#include <ros.h>
#include <std_msgs/String.h>
#include <custom_msg_pkg/control_msg.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

//=======================상수 정의 ==============================
const unsigned long encoder_time_interval = 100;
const unsigned long communication_time_interval = 100; // 0.1초 간격으로 제어
const unsigned long control_time_interval = 100;
//=============================================================


//=======================핀번호 정의==============================

//-------------------------종방향 제어----------------------------
#define MOTOR_PWM_PIN 2 // 모터 속도 제어 핀
#define ENCODER_PIN 3 // 속도 센서 인터럽트 핀
#define BRAKE_PWM_PIN 4 //브레이크 pwm 핀
//-------------------------------------------------------------

//-------------------------횡방향 제어---------------------------
#define STEERING_CAN_TX 5
#define STEERING_CAN_RX 6
//-------------------------------------------------------------

//-------------------------ASMS 스위치--------------------------
#define ESTOP_PIN 7
#define ASMS_MODE_PIN 8
//-------------------------------------------------------------

//==============================================================


void vEncoderTask(void *pvParameters);
void vCommunicationTask(void *pvParameters);
void vControlTask(void *pvParameters);

ros::NodeHandle nh;

//--------센서 데이터 전역변수---------
volatile unsigned int g_pulseCount = 0;
float g_current_rpm = 0.0;
//---------------------------------

// ROS 퍼블리셔 설정
custom_msg_pkg::control_msg pub_msg;
ros::Publisher pub("feedback_topic", &pub_msg);

// portMUX_TYPE 변수 선언
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR countPulse() {
    portENTER_CRITICAL_ISR(&mux);
    g_pulseCount++;
    portEXIT_CRITICAL_ISR(&mux);
}

void setup(void) {

    Serial.begin(115200);
    nh.getHardware()->setBaud(115200);

    // ROS 노드 핸들 초기화
    nh.initNode();

    // 퍼블리셔 등록
    nh.advertise(pub);

    // FreeRTOS 태스크 생성
    xTaskCreatePinnedToCore(vEncoderTask, "EncoderTask", 1024, NULL, 2, NULL, 0);  
    xTaskCreatePinnedToCore(vCommunicationTask, "CommunicationTask", 1024, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(vControlTask, "ControlTask", 1024, NULL, 1, NULL, 1);   

    // 핀모드 설정 및 인터럽트 초기화
    pinMode(ENCODER_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countPulse, RISING);
}

void vEncoderTask(void *pvParameters) {
    // 엔코더 값읽기를 수행하는 태스크
    unsigned long last_time = 0;
    unsigned long current_time = 0;
    unsigned long currentPulseCount;

    for (;;) {
        if(((current_time = millis())-last_time) > encoder_time_interval ){
          last_time = current_time;
          // 크리티컬 섹션에서 펄스 카운트 캡처 및 초기화
          portENTER_CRITICAL(&mux);
          currentPulseCount = g_pulseCount;
          g_pulseCount = 0;
          portEXIT_CRITICAL(&mux);

          g_current_rpm = (currentPulseCount * 60.0) / 20.0; // 예: 20펄스 당 1회전, 필요 시 수정
        }
    }
}

void vCommunicationTask(void *pvParameters) { // 변경된 센서 데이터를 퍼블리시함
    unsigned long last_time = 0;
    unsigned long current_time = 0;

    for (;;) {
        unsigned long current_time = millis();
        if (current_time - last_time >= communication_time_interval) {
            last_time = current_time;

            // // 디버깅 메시지 출력
            // Serial.print("Pulse Count: ");
            // Serial.println(currentPulseCount);
            // Serial.print("Current RPM: ");
            // Serial.println(g_current_rpm);

            // 엔코더 RPM 값 퍼블리시
            pub_msg.current_rpm = g_current_rpm;
            pub.publish(&pub_msg);

            // ROS 통신 유지
            nh.spinOnce();
        }

        // 주기적으로 실행
        vTaskDelay(1 / portTICK_PERIOD_MS); // 짧은 지연으로 다른 태스크에 CPU 시간을 양보
    }
}

void vControlTask(void *pvParameters) { // 변경된 센서 데이터를 퍼블리시함
    unsigned long last_time = 0;
    unsigned long current_time = 0;

    for (;;) {
        unsigned long current_time = millis();
        if (current_time - last_time >= control_time_interval) {
          last_time = current_time;
          //제어태스크 추가
        }
        // 주기적으로 실행
        vTaskDelay(1 / portTICK_PERIOD_MS); // 짧은 지연으로 다른 태스크에 CPU 시간을 양보
    }
}

void loop() {}
