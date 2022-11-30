# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "legged_robot: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ilegged_robot:/home/annie/Legged-Robot/ROS Nodes/src/legged_robot/msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(legged_robot_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/annie/Legged-Robot/ROS Nodes/src/legged_robot/msg/walk_msg.msg" NAME_WE)
add_custom_target(_legged_robot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "legged_robot" "/home/annie/Legged-Robot/ROS Nodes/src/legged_robot/msg/walk_msg.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(legged_robot
  "/home/annie/Legged-Robot/ROS Nodes/src/legged_robot/msg/walk_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/legged_robot
)

### Generating Services

### Generating Module File
_generate_module_cpp(legged_robot
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/legged_robot
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(legged_robot_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(legged_robot_generate_messages legged_robot_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/annie/Legged-Robot/ROS Nodes/src/legged_robot/msg/walk_msg.msg" NAME_WE)
add_dependencies(legged_robot_generate_messages_cpp _legged_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(legged_robot_gencpp)
add_dependencies(legged_robot_gencpp legged_robot_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS legged_robot_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(legged_robot
  "/home/annie/Legged-Robot/ROS Nodes/src/legged_robot/msg/walk_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/legged_robot
)

### Generating Services

### Generating Module File
_generate_module_eus(legged_robot
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/legged_robot
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(legged_robot_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(legged_robot_generate_messages legged_robot_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/annie/Legged-Robot/ROS Nodes/src/legged_robot/msg/walk_msg.msg" NAME_WE)
add_dependencies(legged_robot_generate_messages_eus _legged_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(legged_robot_geneus)
add_dependencies(legged_robot_geneus legged_robot_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS legged_robot_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(legged_robot
  "/home/annie/Legged-Robot/ROS Nodes/src/legged_robot/msg/walk_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/legged_robot
)

### Generating Services

### Generating Module File
_generate_module_lisp(legged_robot
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/legged_robot
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(legged_robot_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(legged_robot_generate_messages legged_robot_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/annie/Legged-Robot/ROS Nodes/src/legged_robot/msg/walk_msg.msg" NAME_WE)
add_dependencies(legged_robot_generate_messages_lisp _legged_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(legged_robot_genlisp)
add_dependencies(legged_robot_genlisp legged_robot_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS legged_robot_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(legged_robot
  "/home/annie/Legged-Robot/ROS Nodes/src/legged_robot/msg/walk_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/legged_robot
)

### Generating Services

### Generating Module File
_generate_module_nodejs(legged_robot
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/legged_robot
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(legged_robot_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(legged_robot_generate_messages legged_robot_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/annie/Legged-Robot/ROS Nodes/src/legged_robot/msg/walk_msg.msg" NAME_WE)
add_dependencies(legged_robot_generate_messages_nodejs _legged_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(legged_robot_gennodejs)
add_dependencies(legged_robot_gennodejs legged_robot_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS legged_robot_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(legged_robot
  "/home/annie/Legged-Robot/ROS Nodes/src/legged_robot/msg/walk_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/legged_robot
)

### Generating Services

### Generating Module File
_generate_module_py(legged_robot
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/legged_robot
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(legged_robot_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(legged_robot_generate_messages legged_robot_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/annie/Legged-Robot/ROS Nodes/src/legged_robot/msg/walk_msg.msg" NAME_WE)
add_dependencies(legged_robot_generate_messages_py _legged_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(legged_robot_genpy)
add_dependencies(legged_robot_genpy legged_robot_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS legged_robot_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/legged_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/legged_robot
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(legged_robot_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(legged_robot_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/legged_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/legged_robot
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(legged_robot_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(legged_robot_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/legged_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/legged_robot
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(legged_robot_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(legged_robot_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/legged_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/legged_robot
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(legged_robot_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(legged_robot_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/legged_robot)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/legged_robot\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/legged_robot
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(legged_robot_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(legged_robot_generate_messages_py std_msgs_generate_messages_py)
endif()
