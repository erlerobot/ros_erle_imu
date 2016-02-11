cmake_minimum_required(VERSION 2.8.3)
project(ros_erle_imu)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy roscpp std_msgs genmsg)

#find_package( Boost 1.40 COMPONENTS program_options REQUIRED )

## Declare ROS messages and services
# add_message_files(DIRECTORY msg FILES )

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs )

## Declare a catkin package
catkin_package(CATKIN_DEPENDS std_msgs)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_executable(imu_talker src/imu_talker.cpp src/MPU9250.cpp)
target_link_libraries(imu_talker ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(imu_talker sensor_smgs_generate_messages_cpp)

add_executable(imu_listener src/imu_listener.cpp)
target_link_libraries(imu_listener ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(imu_listener sensor_smgs_generate_messages_cpp)

catkin_install_python(PROGRAMS
  scripts/imu.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

