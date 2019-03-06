# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "map_annotator: 2 messages, 0 services")

set(MSG_I_FLAGS "-Imap_annotator:/home/team4/catkin_ws/src/cse481wi19/map_annotator/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(map_annotator_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/map_annotator/msg/PoseNames.msg" NAME_WE)
add_custom_target(_map_annotator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "map_annotator" "/home/team4/catkin_ws/src/cse481wi19/map_annotator/msg/PoseNames.msg" ""
)

get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/map_annotator/msg/UserAction.msg" NAME_WE)
add_custom_target(_map_annotator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "map_annotator" "/home/team4/catkin_ws/src/cse481wi19/map_annotator/msg/UserAction.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(map_annotator
  "/home/team4/catkin_ws/src/cse481wi19/map_annotator/msg/PoseNames.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/map_annotator
)
_generate_msg_cpp(map_annotator
  "/home/team4/catkin_ws/src/cse481wi19/map_annotator/msg/UserAction.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/map_annotator
)

### Generating Services

### Generating Module File
_generate_module_cpp(map_annotator
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/map_annotator
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(map_annotator_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(map_annotator_generate_messages map_annotator_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/map_annotator/msg/PoseNames.msg" NAME_WE)
add_dependencies(map_annotator_generate_messages_cpp _map_annotator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/map_annotator/msg/UserAction.msg" NAME_WE)
add_dependencies(map_annotator_generate_messages_cpp _map_annotator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(map_annotator_gencpp)
add_dependencies(map_annotator_gencpp map_annotator_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS map_annotator_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(map_annotator
  "/home/team4/catkin_ws/src/cse481wi19/map_annotator/msg/PoseNames.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/map_annotator
)
_generate_msg_lisp(map_annotator
  "/home/team4/catkin_ws/src/cse481wi19/map_annotator/msg/UserAction.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/map_annotator
)

### Generating Services

### Generating Module File
_generate_module_lisp(map_annotator
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/map_annotator
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(map_annotator_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(map_annotator_generate_messages map_annotator_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/map_annotator/msg/PoseNames.msg" NAME_WE)
add_dependencies(map_annotator_generate_messages_lisp _map_annotator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/map_annotator/msg/UserAction.msg" NAME_WE)
add_dependencies(map_annotator_generate_messages_lisp _map_annotator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(map_annotator_genlisp)
add_dependencies(map_annotator_genlisp map_annotator_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS map_annotator_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(map_annotator
  "/home/team4/catkin_ws/src/cse481wi19/map_annotator/msg/PoseNames.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/map_annotator
)
_generate_msg_py(map_annotator
  "/home/team4/catkin_ws/src/cse481wi19/map_annotator/msg/UserAction.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/map_annotator
)

### Generating Services

### Generating Module File
_generate_module_py(map_annotator
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/map_annotator
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(map_annotator_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(map_annotator_generate_messages map_annotator_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/map_annotator/msg/PoseNames.msg" NAME_WE)
add_dependencies(map_annotator_generate_messages_py _map_annotator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/map_annotator/msg/UserAction.msg" NAME_WE)
add_dependencies(map_annotator_generate_messages_py _map_annotator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(map_annotator_genpy)
add_dependencies(map_annotator_genpy map_annotator_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS map_annotator_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/map_annotator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/map_annotator
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(map_annotator_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/map_annotator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/map_annotator
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(map_annotator_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/map_annotator)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/map_annotator\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/map_annotator
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(map_annotator_generate_messages_py std_msgs_generate_messages_py)
endif()
