cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)  #${PROJECT_NAME}

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  message_generation #(Despite its name, message_generation works for both msg and srv.)
  dxl_serial
)

# for fatal error : ros/ros.h: No such file or directory #include “ros/ros.h” 
include_directories(
  include/${PROJECT_NAME} 
  ${catkin_INCLUDE_DIRS} #모든 패키지의 include 디렉토리를 추가
)  # using catkin

## Declare ROS messages and services
add_message_files(
  FILES 
  Num.msg
)

add_service_files(
  FILES 
  AddTwoInts.srv
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES 
  std_msgs
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES beginner_tutorials
  CATKIN_DEPENDS 
  message_runtime
)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker ${PROJECT_NAME}_generate_messages_cpp)
# catkin 작업 공간 내에서 다른 패키지의 메시지를 사용하는 경우 catkin이 모든 프로젝트를 병렬로 빌드하기 때문에 해당 생성 대상에도 종속성을 추가해야 합니다. 

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

#################

add_executable(open_cr_module src/open_cr_module.cpp)
target_link_libraries(open_cr_module ${catkin_LIBRARIES})
add_dependencies(open_cr_module beginner_tutorials_generate_messages_cpp)

add_executable(soccer_demo src/soccer_demo.cpp)
target_link_libraries(soccer_demo ${catkin_LIBRARIES})
add_dependencies(soccer_demo beginner_tutorials_generate_messages_cpp)

add_executable(foot_odom src/foot_odom.cpp)
target_link_libraries(foot_odom ${catkin_LIBRARIES})
add_dependencies(foot_odom beginner_tutorials_generate_messages_cpp)


###################

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
add_dependencies(add_two_ints_server beginner_tutorials_gencpp)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
add_dependencies(add_two_ints_client beginner_tutorials_gencpp)

