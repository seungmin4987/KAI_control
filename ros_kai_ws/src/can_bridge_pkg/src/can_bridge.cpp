#include <ros/ros.h>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/string.h>
#include <custom_msg_pkg/ControlMsg.h>
#include <custom_msg_pkg/FeedbackMsg.h>
#include <iostream>
#include <vector>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <math.h>  // fabs() 함수 사용

// CAN 인터페이스 설정
const char* CAN_INTERFACE = "can0";
int s;

// 스케일링 비율
const double scaling_factor = 2700.0 / 14.25;

// Cubic Spline 보간 함수 (음수에도 대칭적, 음수 입력에 대해 음수 출력)
double symmetric_spline(double x) {
    double abs_x = fabs(x);  // 절대값을 취해서 대칭성을 구현
    double result;
    if (abs_x < 22.5) {
        // 첫 번째 구간 Cubic Spline 보간
        if (abs_x < 11) {
            result = scaling_factor * (0.000266 * (abs_x - 0) * (abs_x - 0) * (abs_x - 0) 
                 - 0.016800 * (abs_x - 0) * (abs_x - 0) 
                 + 0.445866 * (abs_x - 0));
        } else {
            result = scaling_factor * (0.000266 * (abs_x - 11) * (abs_x - 11) * (abs_x - 11) 
                 - 0.008016 * (abs_x - 11) * (abs_x - 11) 
                 + 0.172889 * (abs_x - 11) + 3.226);
        }
    } else {
        // 두 번째 구간 Cubic Spline 보간
        if (abs_x < 45) {
            result = scaling_factor * (-0.000006 * (abs_x - 22.5) * (abs_x - 22.5) * (abs_x - 22.5) 
                 + 0.001168 * (abs_x - 22.5) * (abs_x - 22.5) 
                 + 0.094142 * (abs_x - 22.5) + 4.559);
        } else {
            result = scaling_factor * (-0.000006 * (abs_x - 45) * (abs_x - 45) * (abs_x - 45) 
                 + 0.000732 * (abs_x - 45) * (abs_x - 45) 
                 + 0.136901 * (abs_x - 45) + 7.195);
        }
    }
    return (x >= 0) ? result : -result;  // 입력이 음수일 경우 음수로 반환
}

// CAN 프레임 전송 함수
void send_can_frame(uint32_t can_id, std::vector<uint8_t> data) {
    struct can_frame frame;
    frame.can_id = can_id | CAN_EFF_FLAG;  // 확장 CAN ID 사용
    frame.can_dlc = data.size();
    memcpy(frame.data, data.data(), frame.can_dlc);

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        ROS_WARN("Buffer full, dropping CAN frame");
    } else {
        ROS_INFO("Sent CAN frame with ID: %X", can_id);
    }
}

// 목표 각도와 모터 속도값을 추출하여 CAN 프레임 전송
void control_callback(const custom_msg_pkg::ControlMsg::ConstPtr& msg) {
    ROS_INFO("Received control message");

    // Cubic Spline을 통해 목표 각도를 변환
    double scaled_angle = symmetric_spline(msg->target_angle);

    // 스케일링된 각도를 ±2700 범위로 제한
    int32_t target_angle = static_cast<int32_t>(std::max(std::min(scaled_angle, 2700.0), -2700.0));

    // 각도 값을 CAN 메시지 데이터로 변환
    std::vector<uint8_t> angle_data = {
        0x23, 0x02, 0x20, 0x01,
        static_cast<uint8_t>((target_angle >> 24) & 0xFF),
        static_cast<uint8_t>((target_angle >> 16) & 0xFF),
        static_cast<uint8_t>((target_angle >> 8) & 0xFF),
        static_cast<uint8_t>(target_angle & 0xFF)
    };

    // 각도 값 CAN 메시지 전송
    send_can_frame(0x06000001, angle_data);
}

// 피드백 메시지를 처리하여 이스탑 플래그 체크
bool estop_activated = false;

void feedback_callback(const custom_msg_pkg::FeedbackMsg::ConstPtr& msg) {
    estop_activated = msg->is_estop_activated;
    if (!estop_activated) {
        ROS_INFO("E-Stop not activated, motor control enabled");
    } else {
        //ROS_WARN("E-Stop activated, motor control disabled");
    }
}

// 특정 CAN ID를 가진 신호를 수신하여 제어 캔신호 전송
void receive_can_frames() {
    struct can_frame frame;

    while (ros::ok()) {
        int nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            ROS_WARN("CAN frame reception error");
            continue;
        }

        // 확장 ID를 사용하는 경우 CAN ID에서 확장 플래그를 제거하여 비교
        uint32_t can_id = frame.can_id & CAN_EFF_MASK;

        // 특정 CAN ID와 일치하는 프레임 처리
        if (can_id == 0x07000001) {
            int16_t combined_value = (frame.data[0] << 8) | frame.data[1];

            if (combined_value >= 0x8000) {
                combined_value -= 0x10000;  // 2의 보수 처리
            }

            // 변환된 값을 로그로 출력
            ROS_INFO("Current angle: %d", combined_value);

            // 모터 활성화 및 비활성화 처리
            std::vector<uint8_t> activate_command = {0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};
            std::vector<uint8_t> deactivate_command = {0x23, 0x0C, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};
            // 현재 모터 상태가 비활성화 상태이면 캔프레임 7번째가 1임.

            if (frame.data[7] == 0x01) {
                // 이스탑이 비활성화된 경우에만 전송
                if (!estop_activated) {
                    send_can_frame(0x06000001, activate_command);
                    ROS_INFO("Motor deactivated, sending activation signal");
                }
            } 
            else if (frame.data[7] == 0x00) {
                if(estop_activated){
                    send_can_frame(0x06000001, deactivate_command);
                    ROS_INFO("estop activated, sending deactivation signal");
                }
                else{
                    ROS_INFO("Motor activated");
                }               
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "can_bridge");
    ros::NodeHandle nh;
    ROS_INFO("CAN bridge node started");

    // CAN 소켓 설정
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    struct sockaddr_can addr;
    struct ifreq ifr;

    strcpy(ifr.ifr_name, CAN_INTERFACE);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    bind(s, (struct sockaddr*)&addr, sizeof(addr));

    // ControlMsg 타입을 구독
    ros::Subscriber control_sub = nh.subscribe("control_topic", 10, control_callback);
    ros::Subscriber feedback_sub = nh.subscribe("mcu_feedback_topic", 10, feedback_callback);

    // CAN 프레임 수신 및 처리
    std::thread receive_thread(receive_can_frames);

    ros::spin();

    // 종료 시 소켓 닫기
    close(s);
    receive_thread.join();
    ROS_INFO("CAN bridge node terminated");

    return 0;
}


