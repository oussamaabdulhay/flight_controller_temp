cmake_minimum_required(VERSION 2.8.3)
project(flight_controller)

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
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ROSUnits
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Navio2/Common
  ${catkin_INCLUDE_DIRS}
)


FILE(GLOB SOURCE_FILES *.hpp *.cpp *.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Navio2_cpp/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/ROSUnits/*.cpp
        )

add_executable(${PROJECT_NAME} flight_controller_node/flight_controller_node.cpp ${SOURCE_FILES})
add_executable(providers_node providers_node/providers_node.cpp ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(providers_node ${catkin_LIBRARIES})
