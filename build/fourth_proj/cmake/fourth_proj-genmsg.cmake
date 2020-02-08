# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "fourth_proj: 9 messages, 1 services")

set(MSG_I_FLAGS "-Ifourth_proj:/home/workshop/new_ws/src/fourth_proj/msg;-Ifourth_proj:/home/workshop/new_ws/devel/share/fourth_proj/msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(fourth_proj_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg" NAME_WE)
add_custom_target(_fourth_proj_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fourth_proj" "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg" ""
)

get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionGoal.msg" NAME_WE)
add_custom_target(_fourth_proj_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fourth_proj" "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionGoal.msg" "fourth_proj/delayactionGoal:actionlib_msgs/GoalID:std_msgs/Header"
)

get_filename_component(_filename "/home/workshop/new_ws/src/fourth_proj/srv/addsrv.srv" NAME_WE)
add_custom_target(_fourth_proj_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fourth_proj" "/home/workshop/new_ws/src/fourth_proj/srv/addsrv.srv" ""
)

get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg" NAME_WE)
add_custom_target(_fourth_proj_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fourth_proj" "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg" ""
)

get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg" NAME_WE)
add_custom_target(_fourth_proj_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fourth_proj" "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg" ""
)

get_filename_component(_filename "/home/workshop/new_ws/src/fourth_proj/msg/sensmsg.msg" NAME_WE)
add_custom_target(_fourth_proj_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fourth_proj" "/home/workshop/new_ws/src/fourth_proj/msg/sensmsg.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/workshop/new_ws/src/fourth_proj/msg/cmdmsg.msg" NAME_WE)
add_custom_target(_fourth_proj_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fourth_proj" "/home/workshop/new_ws/src/fourth_proj/msg/cmdmsg.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionResult.msg" NAME_WE)
add_custom_target(_fourth_proj_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fourth_proj" "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionResult.msg" "actionlib_msgs/GoalID:fourth_proj/delayactionResult:std_msgs/Header:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionAction.msg" NAME_WE)
add_custom_target(_fourth_proj_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fourth_proj" "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionAction.msg" "fourth_proj/delayactionFeedback:std_msgs/Header:fourth_proj/delayactionGoal:fourth_proj/delayactionActionGoal:fourth_proj/delayactionActionResult:fourth_proj/delayactionActionFeedback:actionlib_msgs/GoalID:fourth_proj/delayactionResult:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionFeedback.msg" NAME_WE)
add_custom_target(_fourth_proj_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fourth_proj" "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionFeedback.msg" "fourth_proj/delayactionFeedback:actionlib_msgs/GoalID:std_msgs/Header:actionlib_msgs/GoalStatus"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fourth_proj
)
_generate_msg_cpp(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fourth_proj
)
_generate_msg_cpp(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fourth_proj
)
_generate_msg_cpp(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fourth_proj
)
_generate_msg_cpp(fourth_proj
  "/home/workshop/new_ws/src/fourth_proj/msg/sensmsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fourth_proj
)
_generate_msg_cpp(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fourth_proj
)
_generate_msg_cpp(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fourth_proj
)
_generate_msg_cpp(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionAction.msg"
  "${MSG_I_FLAGS}"
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionGoal.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionResult.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fourth_proj
)
_generate_msg_cpp(fourth_proj
  "/home/workshop/new_ws/src/fourth_proj/msg/cmdmsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fourth_proj
)

### Generating Services
_generate_srv_cpp(fourth_proj
  "/home/workshop/new_ws/src/fourth_proj/srv/addsrv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fourth_proj
)

### Generating Module File
_generate_module_cpp(fourth_proj
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fourth_proj
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(fourth_proj_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(fourth_proj_generate_messages fourth_proj_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_cpp _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionGoal.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_cpp _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/src/fourth_proj/srv/addsrv.srv" NAME_WE)
add_dependencies(fourth_proj_generate_messages_cpp _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_cpp _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_cpp _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/src/fourth_proj/msg/sensmsg.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_cpp _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/src/fourth_proj/msg/cmdmsg.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_cpp _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionResult.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_cpp _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionAction.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_cpp _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionFeedback.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_cpp _fourth_proj_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fourth_proj_gencpp)
add_dependencies(fourth_proj_gencpp fourth_proj_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fourth_proj_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fourth_proj
)
_generate_msg_eus(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fourth_proj
)
_generate_msg_eus(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fourth_proj
)
_generate_msg_eus(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fourth_proj
)
_generate_msg_eus(fourth_proj
  "/home/workshop/new_ws/src/fourth_proj/msg/sensmsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fourth_proj
)
_generate_msg_eus(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fourth_proj
)
_generate_msg_eus(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fourth_proj
)
_generate_msg_eus(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionAction.msg"
  "${MSG_I_FLAGS}"
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionGoal.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionResult.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fourth_proj
)
_generate_msg_eus(fourth_proj
  "/home/workshop/new_ws/src/fourth_proj/msg/cmdmsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fourth_proj
)

### Generating Services
_generate_srv_eus(fourth_proj
  "/home/workshop/new_ws/src/fourth_proj/srv/addsrv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fourth_proj
)

### Generating Module File
_generate_module_eus(fourth_proj
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fourth_proj
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(fourth_proj_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(fourth_proj_generate_messages fourth_proj_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_eus _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionGoal.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_eus _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/src/fourth_proj/srv/addsrv.srv" NAME_WE)
add_dependencies(fourth_proj_generate_messages_eus _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_eus _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_eus _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/src/fourth_proj/msg/sensmsg.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_eus _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/src/fourth_proj/msg/cmdmsg.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_eus _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionResult.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_eus _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionAction.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_eus _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionFeedback.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_eus _fourth_proj_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fourth_proj_geneus)
add_dependencies(fourth_proj_geneus fourth_proj_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fourth_proj_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fourth_proj
)
_generate_msg_lisp(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fourth_proj
)
_generate_msg_lisp(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fourth_proj
)
_generate_msg_lisp(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fourth_proj
)
_generate_msg_lisp(fourth_proj
  "/home/workshop/new_ws/src/fourth_proj/msg/sensmsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fourth_proj
)
_generate_msg_lisp(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fourth_proj
)
_generate_msg_lisp(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fourth_proj
)
_generate_msg_lisp(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionAction.msg"
  "${MSG_I_FLAGS}"
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionGoal.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionResult.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fourth_proj
)
_generate_msg_lisp(fourth_proj
  "/home/workshop/new_ws/src/fourth_proj/msg/cmdmsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fourth_proj
)

### Generating Services
_generate_srv_lisp(fourth_proj
  "/home/workshop/new_ws/src/fourth_proj/srv/addsrv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fourth_proj
)

### Generating Module File
_generate_module_lisp(fourth_proj
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fourth_proj
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(fourth_proj_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(fourth_proj_generate_messages fourth_proj_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_lisp _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionGoal.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_lisp _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/src/fourth_proj/srv/addsrv.srv" NAME_WE)
add_dependencies(fourth_proj_generate_messages_lisp _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_lisp _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_lisp _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/src/fourth_proj/msg/sensmsg.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_lisp _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/src/fourth_proj/msg/cmdmsg.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_lisp _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionResult.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_lisp _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionAction.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_lisp _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionFeedback.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_lisp _fourth_proj_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fourth_proj_genlisp)
add_dependencies(fourth_proj_genlisp fourth_proj_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fourth_proj_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fourth_proj
)
_generate_msg_nodejs(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fourth_proj
)
_generate_msg_nodejs(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fourth_proj
)
_generate_msg_nodejs(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fourth_proj
)
_generate_msg_nodejs(fourth_proj
  "/home/workshop/new_ws/src/fourth_proj/msg/sensmsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fourth_proj
)
_generate_msg_nodejs(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fourth_proj
)
_generate_msg_nodejs(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fourth_proj
)
_generate_msg_nodejs(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionAction.msg"
  "${MSG_I_FLAGS}"
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionGoal.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionResult.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fourth_proj
)
_generate_msg_nodejs(fourth_proj
  "/home/workshop/new_ws/src/fourth_proj/msg/cmdmsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fourth_proj
)

### Generating Services
_generate_srv_nodejs(fourth_proj
  "/home/workshop/new_ws/src/fourth_proj/srv/addsrv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fourth_proj
)

### Generating Module File
_generate_module_nodejs(fourth_proj
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fourth_proj
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(fourth_proj_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(fourth_proj_generate_messages fourth_proj_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_nodejs _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionGoal.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_nodejs _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/src/fourth_proj/srv/addsrv.srv" NAME_WE)
add_dependencies(fourth_proj_generate_messages_nodejs _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_nodejs _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_nodejs _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/src/fourth_proj/msg/sensmsg.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_nodejs _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/src/fourth_proj/msg/cmdmsg.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_nodejs _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionResult.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_nodejs _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionAction.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_nodejs _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionFeedback.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_nodejs _fourth_proj_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fourth_proj_gennodejs)
add_dependencies(fourth_proj_gennodejs fourth_proj_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fourth_proj_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fourth_proj
)
_generate_msg_py(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fourth_proj
)
_generate_msg_py(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fourth_proj
)
_generate_msg_py(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fourth_proj
)
_generate_msg_py(fourth_proj
  "/home/workshop/new_ws/src/fourth_proj/msg/sensmsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fourth_proj
)
_generate_msg_py(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fourth_proj
)
_generate_msg_py(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fourth_proj
)
_generate_msg_py(fourth_proj
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionAction.msg"
  "${MSG_I_FLAGS}"
  "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionGoal.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionResult.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fourth_proj
)
_generate_msg_py(fourth_proj
  "/home/workshop/new_ws/src/fourth_proj/msg/cmdmsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fourth_proj
)

### Generating Services
_generate_srv_py(fourth_proj
  "/home/workshop/new_ws/src/fourth_proj/srv/addsrv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fourth_proj
)

### Generating Module File
_generate_module_py(fourth_proj
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fourth_proj
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(fourth_proj_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(fourth_proj_generate_messages fourth_proj_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionFeedback.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_py _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionGoal.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_py _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/src/fourth_proj/srv/addsrv.srv" NAME_WE)
add_dependencies(fourth_proj_generate_messages_py _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionGoal.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_py _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionResult.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_py _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/src/fourth_proj/msg/sensmsg.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_py _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/src/fourth_proj/msg/cmdmsg.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_py _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionResult.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_py _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionAction.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_py _fourth_proj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/workshop/new_ws/devel/share/fourth_proj/msg/delayactionActionFeedback.msg" NAME_WE)
add_dependencies(fourth_proj_generate_messages_py _fourth_proj_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fourth_proj_genpy)
add_dependencies(fourth_proj_genpy fourth_proj_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fourth_proj_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fourth_proj)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fourth_proj
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(fourth_proj_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(fourth_proj_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fourth_proj)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fourth_proj
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(fourth_proj_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(fourth_proj_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fourth_proj)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fourth_proj
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(fourth_proj_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(fourth_proj_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fourth_proj)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fourth_proj
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(fourth_proj_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(fourth_proj_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fourth_proj)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fourth_proj\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fourth_proj
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(fourth_proj_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(fourth_proj_generate_messages_py std_msgs_generate_messages_py)
endif()
