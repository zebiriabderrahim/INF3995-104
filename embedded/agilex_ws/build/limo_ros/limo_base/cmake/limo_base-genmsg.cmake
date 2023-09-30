# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "limo_base: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ilimo_base:/home/nvidia/INF3995-104/embedded/agilex_ws/src/limo_ros/limo_base/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(limo_base_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/nvidia/INF3995-104/embedded/agilex_ws/src/limo_ros/limo_base/msg/LimoStatus.msg" NAME_WE)
add_custom_target(_limo_base_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "limo_base" "/home/nvidia/INF3995-104/embedded/agilex_ws/src/limo_ros/limo_base/msg/LimoStatus.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(limo_base
  "/home/nvidia/INF3995-104/embedded/agilex_ws/src/limo_ros/limo_base/msg/LimoStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_base
)

### Generating Services

### Generating Module File
_generate_module_cpp(limo_base
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_base
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(limo_base_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(limo_base_generate_messages limo_base_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/INF3995-104/embedded/agilex_ws/src/limo_ros/limo_base/msg/LimoStatus.msg" NAME_WE)
add_dependencies(limo_base_generate_messages_cpp _limo_base_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_base_gencpp)
add_dependencies(limo_base_gencpp limo_base_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_base_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(limo_base
  "/home/nvidia/INF3995-104/embedded/agilex_ws/src/limo_ros/limo_base/msg/LimoStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_base
)

### Generating Services

### Generating Module File
_generate_module_eus(limo_base
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_base
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(limo_base_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(limo_base_generate_messages limo_base_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/INF3995-104/embedded/agilex_ws/src/limo_ros/limo_base/msg/LimoStatus.msg" NAME_WE)
add_dependencies(limo_base_generate_messages_eus _limo_base_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_base_geneus)
add_dependencies(limo_base_geneus limo_base_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_base_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(limo_base
  "/home/nvidia/INF3995-104/embedded/agilex_ws/src/limo_ros/limo_base/msg/LimoStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_base
)

### Generating Services

### Generating Module File
_generate_module_lisp(limo_base
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_base
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(limo_base_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(limo_base_generate_messages limo_base_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/INF3995-104/embedded/agilex_ws/src/limo_ros/limo_base/msg/LimoStatus.msg" NAME_WE)
add_dependencies(limo_base_generate_messages_lisp _limo_base_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_base_genlisp)
add_dependencies(limo_base_genlisp limo_base_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_base_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(limo_base
  "/home/nvidia/INF3995-104/embedded/agilex_ws/src/limo_ros/limo_base/msg/LimoStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_base
)

### Generating Services

### Generating Module File
_generate_module_nodejs(limo_base
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_base
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(limo_base_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(limo_base_generate_messages limo_base_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/INF3995-104/embedded/agilex_ws/src/limo_ros/limo_base/msg/LimoStatus.msg" NAME_WE)
add_dependencies(limo_base_generate_messages_nodejs _limo_base_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_base_gennodejs)
add_dependencies(limo_base_gennodejs limo_base_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_base_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(limo_base
  "/home/nvidia/INF3995-104/embedded/agilex_ws/src/limo_ros/limo_base/msg/LimoStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_base
)

### Generating Services

### Generating Module File
_generate_module_py(limo_base
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_base
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(limo_base_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(limo_base_generate_messages limo_base_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/INF3995-104/embedded/agilex_ws/src/limo_ros/limo_base/msg/LimoStatus.msg" NAME_WE)
add_dependencies(limo_base_generate_messages_py _limo_base_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_base_genpy)
add_dependencies(limo_base_genpy limo_base_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_base_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_base)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_base
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(limo_base_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_base)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_base
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(limo_base_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_base)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_base
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(limo_base_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_base)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_base
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(limo_base_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_base)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_base\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_base
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(limo_base_generate_messages_py std_msgs_generate_messages_py)
endif()
