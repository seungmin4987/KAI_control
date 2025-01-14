cmake_minimum_required(VERSION 3.0.2)
project(custom_msg_pkg)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp
  rospy
)

# 메시지 파일들을 이곳에 추가합니다.
add_message_files(
  FILES
  FeedbackMsg.msg
  ControlMsg.msg  	
)

# 메시지 의존성을 설정합니다.
generate_messages(
  DEPENDENCIES
  std_msgs
)

# 패키지 설정을 정의합니다.
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

# 포함 디렉터리를 설정합니다.
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Python 테스트 실행을 위해 아래 코드를 사용할 수 있습니다.
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


