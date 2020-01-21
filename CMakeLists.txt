cmake_minimum_required(VERSION 2.8.3)
project(positioning_system)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Controller_param.msg
  Waypoint.msg
  Info.msg
)

add_service_files(
  FILES
  Arm.srv
  Update_Controller.srv
  Reset_Controller.srv
  Update_Pose_Reference.srv
  Update_X_Reference.srv
  Update_Y_Reference.srv
  Update_Z_Reference.srv
  Update_Yaw_Reference.srv
  SwitchBlock.srv
  Restricted_Norm_Settings.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Navio2
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Navio2/Common
  ${CMAKE_CURRENT_SOURCE_DIR}/xsens_ros_mti_driver/src
  ${CMAKE_CURRENT_SOURCE_DIR}/xsens_ros_mti_driver/src/messagepublishers
  ${CMAKE_CURRENT_SOURCE_DIR}/xsens_ros_mti_driver/lib/xspublic
  ${CMAKE_CURRENT_SOURCE_DIR}/xsens_ros_mti_driver/lib/xspublic/xscontroller
  ${CMAKE_CURRENT_SOURCE_DIR}/xsens_ros_mti_driver/lib/xspublic/xscommon
  ${CMAKE_CURRENT_SOURCE_DIR}/xsens_ros_mti_driver/lib/xspublic/xstypes
  ${catkin_INCLUDE_DIRS}
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/xsens_ros_mti_driver/lib/xspublic/xscontroller ${CMAKE_CURRENT_SOURCE_DIR}/xsens_ros_mti_driver/lib/xspublic/xscommon ${CMAKE_CURRENT_SOURCE_DIR}/xsens_ros_mti_driver/lib/xspublic/xstypes)

FILE(GLOB SOURCE_FILES *.hpp *.cpp *.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Navio2_cpp/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/xsens_ros_mti_driver/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/xsens_ros_mti_driver/lib/xspublic/xscontroller/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/xsens_ros_mti_driver/lib/xspublic/xscommon/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/xsens_ros_mti_driver/lib/xspublic/xstypes/*.cpp
        )

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} xscontroller xscommon xstypes pthread dl ${catkin_LIBRARIES})
