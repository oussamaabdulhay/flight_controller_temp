# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "flight_controller: 5 messages, 12 services")

set(MSG_I_FLAGS "-Iflight_controller:/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(flight_controller_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Z_Reference.srv" NAME_WE)
add_custom_target(_flight_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flight_controller" "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Z_Reference.srv" ""
)

get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/PID_param.msg" NAME_WE)
add_custom_target(_flight_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flight_controller" "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/PID_param.msg" ""
)

get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/MRFT_param.msg" NAME_WE)
add_custom_target(_flight_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flight_controller" "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/MRFT_param.msg" ""
)

get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_PID.srv" NAME_WE)
add_custom_target(_flight_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flight_controller" "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_PID.srv" "flight_controller/PID_param"
)

get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_MRFT.srv" NAME_WE)
add_custom_target(_flight_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flight_controller" "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_MRFT.srv" "flight_controller/MRFT_param"
)

get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/SM_param.msg" NAME_WE)
add_custom_target(_flight_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flight_controller" "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/SM_param.msg" ""
)

get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Yaw_Reference.srv" NAME_WE)
add_custom_target(_flight_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flight_controller" "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Yaw_Reference.srv" ""
)

get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/SwitchBlock.srv" NAME_WE)
add_custom_target(_flight_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flight_controller" "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/SwitchBlock.srv" ""
)

get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_X_Reference.srv" NAME_WE)
add_custom_target(_flight_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flight_controller" "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_X_Reference.srv" ""
)

get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Pose_Reference.srv" NAME_WE)
add_custom_target(_flight_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flight_controller" "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Pose_Reference.srv" "flight_controller/Waypoint"
)

get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Y_Reference.srv" NAME_WE)
add_custom_target(_flight_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flight_controller" "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Y_Reference.srv" ""
)

get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Arm.srv" NAME_WE)
add_custom_target(_flight_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flight_controller" "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Arm.srv" ""
)

get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Info.msg" NAME_WE)
add_custom_target(_flight_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flight_controller" "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Info.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Waypoint.msg" NAME_WE)
add_custom_target(_flight_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flight_controller" "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Waypoint.msg" ""
)

get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Reset_Controller.srv" NAME_WE)
add_custom_target(_flight_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flight_controller" "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Reset_Controller.srv" ""
)

get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Restricted_Norm_Settings.srv" NAME_WE)
add_custom_target(_flight_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flight_controller" "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Restricted_Norm_Settings.srv" ""
)

get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_SM.srv" NAME_WE)
add_custom_target(_flight_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flight_controller" "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_SM.srv" "flight_controller/SM_param"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/SM_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
)
_generate_msg_cpp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
)
_generate_msg_cpp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
)
_generate_msg_cpp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/PID_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
)
_generate_msg_cpp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/MRFT_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
)

### Generating Services
_generate_srv_cpp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Z_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
)
_generate_srv_cpp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_PID.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/PID_param.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
)
_generate_srv_cpp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_MRFT.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/MRFT_param.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
)
_generate_srv_cpp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_X_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
)
_generate_srv_cpp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Yaw_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
)
_generate_srv_cpp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/SwitchBlock.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
)
_generate_srv_cpp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Pose_Reference.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
)
_generate_srv_cpp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Y_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
)
_generate_srv_cpp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Arm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
)
_generate_srv_cpp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Reset_Controller.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
)
_generate_srv_cpp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Restricted_Norm_Settings.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
)
_generate_srv_cpp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_SM.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/SM_param.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
)

### Generating Module File
_generate_module_cpp(flight_controller
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(flight_controller_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(flight_controller_generate_messages flight_controller_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Z_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_cpp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/PID_param.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_cpp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/MRFT_param.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_cpp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_PID.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_cpp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_MRFT.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_cpp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/SM_param.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_cpp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Yaw_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_cpp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/SwitchBlock.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_cpp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_X_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_cpp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Pose_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_cpp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Y_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_cpp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Arm.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_cpp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Info.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_cpp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Waypoint.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_cpp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Reset_Controller.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_cpp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Restricted_Norm_Settings.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_cpp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_SM.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_cpp _flight_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flight_controller_gencpp)
add_dependencies(flight_controller_gencpp flight_controller_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flight_controller_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/SM_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
)
_generate_msg_eus(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
)
_generate_msg_eus(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
)
_generate_msg_eus(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/PID_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
)
_generate_msg_eus(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/MRFT_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
)

### Generating Services
_generate_srv_eus(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Z_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
)
_generate_srv_eus(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_PID.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/PID_param.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
)
_generate_srv_eus(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_MRFT.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/MRFT_param.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
)
_generate_srv_eus(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_X_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
)
_generate_srv_eus(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Yaw_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
)
_generate_srv_eus(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/SwitchBlock.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
)
_generate_srv_eus(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Pose_Reference.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
)
_generate_srv_eus(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Y_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
)
_generate_srv_eus(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Arm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
)
_generate_srv_eus(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Reset_Controller.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
)
_generate_srv_eus(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Restricted_Norm_Settings.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
)
_generate_srv_eus(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_SM.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/SM_param.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
)

### Generating Module File
_generate_module_eus(flight_controller
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(flight_controller_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(flight_controller_generate_messages flight_controller_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Z_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_eus _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/PID_param.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_eus _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/MRFT_param.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_eus _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_PID.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_eus _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_MRFT.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_eus _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/SM_param.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_eus _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Yaw_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_eus _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/SwitchBlock.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_eus _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_X_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_eus _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Pose_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_eus _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Y_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_eus _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Arm.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_eus _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Info.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_eus _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Waypoint.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_eus _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Reset_Controller.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_eus _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Restricted_Norm_Settings.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_eus _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_SM.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_eus _flight_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flight_controller_geneus)
add_dependencies(flight_controller_geneus flight_controller_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flight_controller_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/SM_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
)
_generate_msg_lisp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
)
_generate_msg_lisp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
)
_generate_msg_lisp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/PID_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
)
_generate_msg_lisp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/MRFT_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
)

### Generating Services
_generate_srv_lisp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Z_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
)
_generate_srv_lisp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_PID.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/PID_param.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
)
_generate_srv_lisp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_MRFT.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/MRFT_param.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
)
_generate_srv_lisp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_X_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
)
_generate_srv_lisp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Yaw_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
)
_generate_srv_lisp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/SwitchBlock.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
)
_generate_srv_lisp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Pose_Reference.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
)
_generate_srv_lisp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Y_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
)
_generate_srv_lisp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Arm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
)
_generate_srv_lisp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Reset_Controller.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
)
_generate_srv_lisp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Restricted_Norm_Settings.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
)
_generate_srv_lisp(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_SM.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/SM_param.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
)

### Generating Module File
_generate_module_lisp(flight_controller
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(flight_controller_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(flight_controller_generate_messages flight_controller_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Z_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_lisp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/PID_param.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_lisp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/MRFT_param.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_lisp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_PID.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_lisp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_MRFT.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_lisp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/SM_param.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_lisp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Yaw_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_lisp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/SwitchBlock.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_lisp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_X_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_lisp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Pose_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_lisp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Y_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_lisp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Arm.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_lisp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Info.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_lisp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Waypoint.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_lisp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Reset_Controller.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_lisp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Restricted_Norm_Settings.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_lisp _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_SM.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_lisp _flight_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flight_controller_genlisp)
add_dependencies(flight_controller_genlisp flight_controller_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flight_controller_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/SM_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
)
_generate_msg_nodejs(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
)
_generate_msg_nodejs(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
)
_generate_msg_nodejs(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/PID_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
)
_generate_msg_nodejs(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/MRFT_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
)

### Generating Services
_generate_srv_nodejs(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Z_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
)
_generate_srv_nodejs(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_PID.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/PID_param.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
)
_generate_srv_nodejs(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_MRFT.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/MRFT_param.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
)
_generate_srv_nodejs(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_X_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
)
_generate_srv_nodejs(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Yaw_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
)
_generate_srv_nodejs(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/SwitchBlock.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
)
_generate_srv_nodejs(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Pose_Reference.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
)
_generate_srv_nodejs(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Y_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
)
_generate_srv_nodejs(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Arm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
)
_generate_srv_nodejs(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Reset_Controller.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
)
_generate_srv_nodejs(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Restricted_Norm_Settings.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
)
_generate_srv_nodejs(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_SM.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/SM_param.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
)

### Generating Module File
_generate_module_nodejs(flight_controller
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(flight_controller_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(flight_controller_generate_messages flight_controller_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Z_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_nodejs _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/PID_param.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_nodejs _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/MRFT_param.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_nodejs _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_PID.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_nodejs _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_MRFT.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_nodejs _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/SM_param.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_nodejs _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Yaw_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_nodejs _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/SwitchBlock.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_nodejs _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_X_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_nodejs _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Pose_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_nodejs _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Y_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_nodejs _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Arm.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_nodejs _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Info.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_nodejs _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Waypoint.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_nodejs _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Reset_Controller.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_nodejs _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Restricted_Norm_Settings.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_nodejs _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_SM.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_nodejs _flight_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flight_controller_gennodejs)
add_dependencies(flight_controller_gennodejs flight_controller_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flight_controller_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/SM_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
)
_generate_msg_py(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
)
_generate_msg_py(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
)
_generate_msg_py(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/PID_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
)
_generate_msg_py(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/MRFT_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
)

### Generating Services
_generate_srv_py(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Z_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
)
_generate_srv_py(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_PID.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/PID_param.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
)
_generate_srv_py(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_MRFT.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/MRFT_param.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
)
_generate_srv_py(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_X_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
)
_generate_srv_py(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Yaw_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
)
_generate_srv_py(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/SwitchBlock.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
)
_generate_srv_py(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Pose_Reference.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
)
_generate_srv_py(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Y_Reference.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
)
_generate_srv_py(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Arm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
)
_generate_srv_py(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Reset_Controller.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
)
_generate_srv_py(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Restricted_Norm_Settings.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
)
_generate_srv_py(flight_controller
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_SM.srv"
  "${MSG_I_FLAGS}"
  "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/SM_param.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
)

### Generating Module File
_generate_module_py(flight_controller
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(flight_controller_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(flight_controller_generate_messages flight_controller_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Z_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_py _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/PID_param.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_py _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/MRFT_param.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_py _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_PID.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_py _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_MRFT.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_py _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/SM_param.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_py _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Yaw_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_py _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/SwitchBlock.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_py _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_X_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_py _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Pose_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_py _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Y_Reference.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_py _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Arm.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_py _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Info.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_py _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/msg/Waypoint.msg" NAME_WE)
add_dependencies(flight_controller_generate_messages_py _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Reset_Controller.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_py _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Restricted_Norm_Settings.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_py _flight_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pedrohrpbs/catkin_ws_NAVIO/src/flight_controller/srv/Update_Controller_SM.srv" NAME_WE)
add_dependencies(flight_controller_generate_messages_py _flight_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flight_controller_genpy)
add_dependencies(flight_controller_genpy flight_controller_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flight_controller_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flight_controller
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(flight_controller_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flight_controller
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(flight_controller_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flight_controller
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(flight_controller_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flight_controller
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(flight_controller_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flight_controller
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(flight_controller_generate_messages_py std_msgs_generate_messages_py)
endif()
