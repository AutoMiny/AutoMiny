# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "fake_gps: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ifake_gps:/root/catkin_ws/src/fake_gps/msg;-Istd_msgs:/opt/odroid-x2/sdk/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/odroid-x2/sdk/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(fake_gps_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/catkin_ws/src/fake_gps/msg/Transform.msg" NAME_WE)
add_custom_target(_fake_gps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fake_gps" "/root/catkin_ws/src/fake_gps/msg/Transform.msg" "geometry_msgs/Vector3:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Transform"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(fake_gps
  "/root/catkin_ws/src/fake_gps/msg/Transform.msg"
  "${MSG_I_FLAGS}"
  "/opt/odroid-x2/sdk/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/odroid-x2/sdk/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/odroid-x2/sdk/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/odroid-x2/sdk/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Transform.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fake_gps
)

### Generating Services

### Generating Module File
_generate_module_cpp(fake_gps
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fake_gps
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(fake_gps_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(fake_gps_generate_messages fake_gps_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/fake_gps/msg/Transform.msg" NAME_WE)
add_dependencies(fake_gps_generate_messages_cpp _fake_gps_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fake_gps_gencpp)
add_dependencies(fake_gps_gencpp fake_gps_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fake_gps_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(fake_gps
  "/root/catkin_ws/src/fake_gps/msg/Transform.msg"
  "${MSG_I_FLAGS}"
  "/opt/odroid-x2/sdk/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/odroid-x2/sdk/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/odroid-x2/sdk/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/odroid-x2/sdk/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Transform.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fake_gps
)

### Generating Services

### Generating Module File
_generate_module_lisp(fake_gps
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fake_gps
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(fake_gps_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(fake_gps_generate_messages fake_gps_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/fake_gps/msg/Transform.msg" NAME_WE)
add_dependencies(fake_gps_generate_messages_lisp _fake_gps_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fake_gps_genlisp)
add_dependencies(fake_gps_genlisp fake_gps_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fake_gps_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(fake_gps
  "/root/catkin_ws/src/fake_gps/msg/Transform.msg"
  "${MSG_I_FLAGS}"
  "/opt/odroid-x2/sdk/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/odroid-x2/sdk/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/odroid-x2/sdk/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/odroid-x2/sdk/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Transform.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fake_gps
)

### Generating Services

### Generating Module File
_generate_module_py(fake_gps
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fake_gps
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(fake_gps_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(fake_gps_generate_messages fake_gps_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/fake_gps/msg/Transform.msg" NAME_WE)
add_dependencies(fake_gps_generate_messages_py _fake_gps_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fake_gps_genpy)
add_dependencies(fake_gps_genpy fake_gps_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fake_gps_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fake_gps)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fake_gps
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(fake_gps_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(fake_gps_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fake_gps)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fake_gps
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(fake_gps_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(fake_gps_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fake_gps)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fake_gps\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fake_gps
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(fake_gps_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(fake_gps_generate_messages_py geometry_msgs_generate_messages_py)
