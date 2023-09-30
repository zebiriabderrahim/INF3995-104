# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "lifter_ctr: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ilifter_ctr:/home/nvidia/INF3995-104/embedded/agilex_ws/src/lifter_ctr/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(lifter_ctr_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/nvidia/INF3995-104/embedded/agilex_ws/src/lifter_ctr/msg/lifter_mode.msg" NAME_WE)
add_custom_target(_lifter_ctr_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lifter_ctr" "/home/nvidia/INF3995-104/embedded/agilex_ws/src/lifter_ctr/msg/lifter_mode.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(lifter_ctr
  "/home/nvidia/INF3995-104/embedded/agilex_ws/src/lifter_ctr/msg/lifter_mode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lifter_ctr
)

### Generating Services

### Generating Module File
_generate_module_cpp(lifter_ctr
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lifter_ctr
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(lifter_ctr_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(lifter_ctr_generate_messages lifter_ctr_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/INF3995-104/embedded/agilex_ws/src/lifter_ctr/msg/lifter_mode.msg" NAME_WE)
add_dependencies(lifter_ctr_generate_messages_cpp _lifter_ctr_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lifter_ctr_gencpp)
add_dependencies(lifter_ctr_gencpp lifter_ctr_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lifter_ctr_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(lifter_ctr
  "/home/nvidia/INF3995-104/embedded/agilex_ws/src/lifter_ctr/msg/lifter_mode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lifter_ctr
)

### Generating Services

### Generating Module File
_generate_module_eus(lifter_ctr
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lifter_ctr
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(lifter_ctr_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(lifter_ctr_generate_messages lifter_ctr_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/INF3995-104/embedded/agilex_ws/src/lifter_ctr/msg/lifter_mode.msg" NAME_WE)
add_dependencies(lifter_ctr_generate_messages_eus _lifter_ctr_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lifter_ctr_geneus)
add_dependencies(lifter_ctr_geneus lifter_ctr_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lifter_ctr_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(lifter_ctr
  "/home/nvidia/INF3995-104/embedded/agilex_ws/src/lifter_ctr/msg/lifter_mode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lifter_ctr
)

### Generating Services

### Generating Module File
_generate_module_lisp(lifter_ctr
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lifter_ctr
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(lifter_ctr_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(lifter_ctr_generate_messages lifter_ctr_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/INF3995-104/embedded/agilex_ws/src/lifter_ctr/msg/lifter_mode.msg" NAME_WE)
add_dependencies(lifter_ctr_generate_messages_lisp _lifter_ctr_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lifter_ctr_genlisp)
add_dependencies(lifter_ctr_genlisp lifter_ctr_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lifter_ctr_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(lifter_ctr
  "/home/nvidia/INF3995-104/embedded/agilex_ws/src/lifter_ctr/msg/lifter_mode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lifter_ctr
)

### Generating Services

### Generating Module File
_generate_module_nodejs(lifter_ctr
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lifter_ctr
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(lifter_ctr_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(lifter_ctr_generate_messages lifter_ctr_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/INF3995-104/embedded/agilex_ws/src/lifter_ctr/msg/lifter_mode.msg" NAME_WE)
add_dependencies(lifter_ctr_generate_messages_nodejs _lifter_ctr_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lifter_ctr_gennodejs)
add_dependencies(lifter_ctr_gennodejs lifter_ctr_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lifter_ctr_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(lifter_ctr
  "/home/nvidia/INF3995-104/embedded/agilex_ws/src/lifter_ctr/msg/lifter_mode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lifter_ctr
)

### Generating Services

### Generating Module File
_generate_module_py(lifter_ctr
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lifter_ctr
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(lifter_ctr_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(lifter_ctr_generate_messages lifter_ctr_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/INF3995-104/embedded/agilex_ws/src/lifter_ctr/msg/lifter_mode.msg" NAME_WE)
add_dependencies(lifter_ctr_generate_messages_py _lifter_ctr_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lifter_ctr_genpy)
add_dependencies(lifter_ctr_genpy lifter_ctr_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lifter_ctr_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lifter_ctr)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lifter_ctr
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(lifter_ctr_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lifter_ctr)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lifter_ctr
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(lifter_ctr_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lifter_ctr)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lifter_ctr
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(lifter_ctr_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lifter_ctr)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lifter_ctr
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(lifter_ctr_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lifter_ctr)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lifter_ctr\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lifter_ctr
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(lifter_ctr_generate_messages_py std_msgs_generate_messages_py)
endif()
