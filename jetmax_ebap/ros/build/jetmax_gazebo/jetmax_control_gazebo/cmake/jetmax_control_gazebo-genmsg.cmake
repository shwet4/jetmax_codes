# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "jetmax_control_gazebo: 0 messages, 2 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(jetmax_control_gazebo_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/IK.srv" NAME_WE)
add_custom_target(_jetmax_control_gazebo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jetmax_control_gazebo" "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/IK.srv" ""
)

get_filename_component(_filename "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/FK.srv" NAME_WE)
add_custom_target(_jetmax_control_gazebo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jetmax_control_gazebo" "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/FK.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(jetmax_control_gazebo
  "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/IK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jetmax_control_gazebo
)
_generate_srv_cpp(jetmax_control_gazebo
  "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/FK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jetmax_control_gazebo
)

### Generating Module File
_generate_module_cpp(jetmax_control_gazebo
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jetmax_control_gazebo
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(jetmax_control_gazebo_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(jetmax_control_gazebo_generate_messages jetmax_control_gazebo_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/IK.srv" NAME_WE)
add_dependencies(jetmax_control_gazebo_generate_messages_cpp _jetmax_control_gazebo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/FK.srv" NAME_WE)
add_dependencies(jetmax_control_gazebo_generate_messages_cpp _jetmax_control_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jetmax_control_gazebo_gencpp)
add_dependencies(jetmax_control_gazebo_gencpp jetmax_control_gazebo_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jetmax_control_gazebo_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(jetmax_control_gazebo
  "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/IK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jetmax_control_gazebo
)
_generate_srv_eus(jetmax_control_gazebo
  "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/FK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jetmax_control_gazebo
)

### Generating Module File
_generate_module_eus(jetmax_control_gazebo
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jetmax_control_gazebo
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(jetmax_control_gazebo_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(jetmax_control_gazebo_generate_messages jetmax_control_gazebo_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/IK.srv" NAME_WE)
add_dependencies(jetmax_control_gazebo_generate_messages_eus _jetmax_control_gazebo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/FK.srv" NAME_WE)
add_dependencies(jetmax_control_gazebo_generate_messages_eus _jetmax_control_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jetmax_control_gazebo_geneus)
add_dependencies(jetmax_control_gazebo_geneus jetmax_control_gazebo_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jetmax_control_gazebo_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(jetmax_control_gazebo
  "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/IK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jetmax_control_gazebo
)
_generate_srv_lisp(jetmax_control_gazebo
  "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/FK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jetmax_control_gazebo
)

### Generating Module File
_generate_module_lisp(jetmax_control_gazebo
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jetmax_control_gazebo
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(jetmax_control_gazebo_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(jetmax_control_gazebo_generate_messages jetmax_control_gazebo_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/IK.srv" NAME_WE)
add_dependencies(jetmax_control_gazebo_generate_messages_lisp _jetmax_control_gazebo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/FK.srv" NAME_WE)
add_dependencies(jetmax_control_gazebo_generate_messages_lisp _jetmax_control_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jetmax_control_gazebo_genlisp)
add_dependencies(jetmax_control_gazebo_genlisp jetmax_control_gazebo_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jetmax_control_gazebo_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(jetmax_control_gazebo
  "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/IK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jetmax_control_gazebo
)
_generate_srv_nodejs(jetmax_control_gazebo
  "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/FK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jetmax_control_gazebo
)

### Generating Module File
_generate_module_nodejs(jetmax_control_gazebo
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jetmax_control_gazebo
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(jetmax_control_gazebo_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(jetmax_control_gazebo_generate_messages jetmax_control_gazebo_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/IK.srv" NAME_WE)
add_dependencies(jetmax_control_gazebo_generate_messages_nodejs _jetmax_control_gazebo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/FK.srv" NAME_WE)
add_dependencies(jetmax_control_gazebo_generate_messages_nodejs _jetmax_control_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jetmax_control_gazebo_gennodejs)
add_dependencies(jetmax_control_gazebo_gennodejs jetmax_control_gazebo_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jetmax_control_gazebo_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(jetmax_control_gazebo
  "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/IK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetmax_control_gazebo
)
_generate_srv_py(jetmax_control_gazebo
  "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/FK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetmax_control_gazebo
)

### Generating Module File
_generate_module_py(jetmax_control_gazebo
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetmax_control_gazebo
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(jetmax_control_gazebo_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(jetmax_control_gazebo_generate_messages jetmax_control_gazebo_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/IK.srv" NAME_WE)
add_dependencies(jetmax_control_gazebo_generate_messages_py _jetmax_control_gazebo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hiwonder/ros/src/jetmax_gazebo/jetmax_control_gazebo/srv/FK.srv" NAME_WE)
add_dependencies(jetmax_control_gazebo_generate_messages_py _jetmax_control_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jetmax_control_gazebo_genpy)
add_dependencies(jetmax_control_gazebo_genpy jetmax_control_gazebo_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jetmax_control_gazebo_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jetmax_control_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jetmax_control_gazebo
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(jetmax_control_gazebo_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(jetmax_control_gazebo_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jetmax_control_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jetmax_control_gazebo
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(jetmax_control_gazebo_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(jetmax_control_gazebo_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jetmax_control_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jetmax_control_gazebo
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(jetmax_control_gazebo_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(jetmax_control_gazebo_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jetmax_control_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jetmax_control_gazebo
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(jetmax_control_gazebo_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(jetmax_control_gazebo_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetmax_control_gazebo)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetmax_control_gazebo\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetmax_control_gazebo
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(jetmax_control_gazebo_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(jetmax_control_gazebo_generate_messages_py std_msgs_generate_messages_py)
endif()
