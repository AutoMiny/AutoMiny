# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "cmvision: 2 messages, 0 services")

set(MSG_I_FLAGS "-Icmvision:/root/catkin_ws/src/cmvision/msg;-Istd_msgs:/opt/odroid-x2/sdk/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(cmvision_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/catkin_ws/src/cmvision/msg/Blob.msg" NAME_WE)
add_custom_target(_cmvision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cmvision" "/root/catkin_ws/src/cmvision/msg/Blob.msg" ""
)

get_filename_component(_filename "/root/catkin_ws/src/cmvision/msg/Blobs.msg" NAME_WE)
add_custom_target(_cmvision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cmvision" "/root/catkin_ws/src/cmvision/msg/Blobs.msg" "std_msgs/Header:cmvision/Blob"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(cmvision
  "/root/catkin_ws/src/cmvision/msg/Blob.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cmvision
)
_generate_msg_cpp(cmvision
  "/root/catkin_ws/src/cmvision/msg/Blobs.msg"
  "${MSG_I_FLAGS}"
  "/opt/odroid-x2/sdk/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/root/catkin_ws/src/cmvision/msg/Blob.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cmvision
)

### Generating Services

### Generating Module File
_generate_module_cpp(cmvision
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cmvision
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(cmvision_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(cmvision_generate_messages cmvision_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/cmvision/msg/Blob.msg" NAME_WE)
add_dependencies(cmvision_generate_messages_cpp _cmvision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/cmvision/msg/Blobs.msg" NAME_WE)
add_dependencies(cmvision_generate_messages_cpp _cmvision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cmvision_gencpp)
add_dependencies(cmvision_gencpp cmvision_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cmvision_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(cmvision
  "/root/catkin_ws/src/cmvision/msg/Blob.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cmvision
)
_generate_msg_lisp(cmvision
  "/root/catkin_ws/src/cmvision/msg/Blobs.msg"
  "${MSG_I_FLAGS}"
  "/opt/odroid-x2/sdk/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/root/catkin_ws/src/cmvision/msg/Blob.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cmvision
)

### Generating Services

### Generating Module File
_generate_module_lisp(cmvision
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cmvision
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(cmvision_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(cmvision_generate_messages cmvision_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/cmvision/msg/Blob.msg" NAME_WE)
add_dependencies(cmvision_generate_messages_lisp _cmvision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/cmvision/msg/Blobs.msg" NAME_WE)
add_dependencies(cmvision_generate_messages_lisp _cmvision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cmvision_genlisp)
add_dependencies(cmvision_genlisp cmvision_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cmvision_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(cmvision
  "/root/catkin_ws/src/cmvision/msg/Blob.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cmvision
)
_generate_msg_py(cmvision
  "/root/catkin_ws/src/cmvision/msg/Blobs.msg"
  "${MSG_I_FLAGS}"
  "/opt/odroid-x2/sdk/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/root/catkin_ws/src/cmvision/msg/Blob.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cmvision
)

### Generating Services

### Generating Module File
_generate_module_py(cmvision
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cmvision
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(cmvision_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(cmvision_generate_messages cmvision_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/cmvision/msg/Blob.msg" NAME_WE)
add_dependencies(cmvision_generate_messages_py _cmvision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/cmvision/msg/Blobs.msg" NAME_WE)
add_dependencies(cmvision_generate_messages_py _cmvision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cmvision_genpy)
add_dependencies(cmvision_genpy cmvision_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cmvision_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cmvision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cmvision
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(cmvision_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cmvision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cmvision
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(cmvision_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cmvision)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cmvision\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cmvision
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(cmvision_generate_messages_py std_msgs_generate_messages_py)
