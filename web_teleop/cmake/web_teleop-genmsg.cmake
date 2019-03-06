# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "web_teleop: 0 messages, 5 services")

set(MSG_I_FLAGS "")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(web_teleop_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/SetTorso.srv" NAME_WE)
add_custom_target(_web_teleop_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "web_teleop" "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/SetTorso.srv" ""
)

get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/CloseGripper.srv" NAME_WE)
add_custom_target(_web_teleop_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "web_teleop" "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/CloseGripper.srv" ""
)

get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/SetArm.srv" NAME_WE)
add_custom_target(_web_teleop_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "web_teleop" "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/SetArm.srv" ""
)

get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/OpenGripper.srv" NAME_WE)
add_custom_target(_web_teleop_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "web_teleop" "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/OpenGripper.srv" ""
)

get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/MoveHead.srv" NAME_WE)
add_custom_target(_web_teleop_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "web_teleop" "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/MoveHead.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(web_teleop
  "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/SetTorso.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/web_teleop
)
_generate_srv_cpp(web_teleop
  "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/CloseGripper.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/web_teleop
)
_generate_srv_cpp(web_teleop
  "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/OpenGripper.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/web_teleop
)
_generate_srv_cpp(web_teleop
  "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/MoveHead.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/web_teleop
)
_generate_srv_cpp(web_teleop
  "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/SetArm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/web_teleop
)

### Generating Module File
_generate_module_cpp(web_teleop
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/web_teleop
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(web_teleop_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(web_teleop_generate_messages web_teleop_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/SetTorso.srv" NAME_WE)
add_dependencies(web_teleop_generate_messages_cpp _web_teleop_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/CloseGripper.srv" NAME_WE)
add_dependencies(web_teleop_generate_messages_cpp _web_teleop_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/SetArm.srv" NAME_WE)
add_dependencies(web_teleop_generate_messages_cpp _web_teleop_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/OpenGripper.srv" NAME_WE)
add_dependencies(web_teleop_generate_messages_cpp _web_teleop_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/MoveHead.srv" NAME_WE)
add_dependencies(web_teleop_generate_messages_cpp _web_teleop_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(web_teleop_gencpp)
add_dependencies(web_teleop_gencpp web_teleop_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS web_teleop_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(web_teleop
  "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/SetTorso.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/web_teleop
)
_generate_srv_lisp(web_teleop
  "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/CloseGripper.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/web_teleop
)
_generate_srv_lisp(web_teleop
  "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/OpenGripper.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/web_teleop
)
_generate_srv_lisp(web_teleop
  "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/MoveHead.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/web_teleop
)
_generate_srv_lisp(web_teleop
  "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/SetArm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/web_teleop
)

### Generating Module File
_generate_module_lisp(web_teleop
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/web_teleop
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(web_teleop_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(web_teleop_generate_messages web_teleop_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/SetTorso.srv" NAME_WE)
add_dependencies(web_teleop_generate_messages_lisp _web_teleop_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/CloseGripper.srv" NAME_WE)
add_dependencies(web_teleop_generate_messages_lisp _web_teleop_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/SetArm.srv" NAME_WE)
add_dependencies(web_teleop_generate_messages_lisp _web_teleop_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/OpenGripper.srv" NAME_WE)
add_dependencies(web_teleop_generate_messages_lisp _web_teleop_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/MoveHead.srv" NAME_WE)
add_dependencies(web_teleop_generate_messages_lisp _web_teleop_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(web_teleop_genlisp)
add_dependencies(web_teleop_genlisp web_teleop_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS web_teleop_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(web_teleop
  "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/SetTorso.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/web_teleop
)
_generate_srv_py(web_teleop
  "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/CloseGripper.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/web_teleop
)
_generate_srv_py(web_teleop
  "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/OpenGripper.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/web_teleop
)
_generate_srv_py(web_teleop
  "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/MoveHead.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/web_teleop
)
_generate_srv_py(web_teleop
  "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/SetArm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/web_teleop
)

### Generating Module File
_generate_module_py(web_teleop
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/web_teleop
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(web_teleop_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(web_teleop_generate_messages web_teleop_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/SetTorso.srv" NAME_WE)
add_dependencies(web_teleop_generate_messages_py _web_teleop_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/CloseGripper.srv" NAME_WE)
add_dependencies(web_teleop_generate_messages_py _web_teleop_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/SetArm.srv" NAME_WE)
add_dependencies(web_teleop_generate_messages_py _web_teleop_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/OpenGripper.srv" NAME_WE)
add_dependencies(web_teleop_generate_messages_py _web_teleop_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/team4/catkin_ws/src/cse481wi19/web_teleop/srv/MoveHead.srv" NAME_WE)
add_dependencies(web_teleop_generate_messages_py _web_teleop_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(web_teleop_genpy)
add_dependencies(web_teleop_genpy web_teleop_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS web_teleop_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/web_teleop)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/web_teleop
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/web_teleop)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/web_teleop
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/web_teleop)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/web_teleop\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/web_teleop
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
