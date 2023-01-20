cmake_minimum_required(VERSION 3.0.2)
project(l3_libs)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Compile as C++17, supported in ROS Melodic and newer
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  std_msgs
  sensor_msgs
  angles
  tf_conversions
  eigen_conversions
  kdl_conversions
  vigir_generic_params
  l3_msgs
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system serialization)
find_package(Eigen3 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Specify additional locations of header files
set(HEADERS
  include/${PROJECT_NAME}/conversions/l3_msg_conversions.h
  include/${PROJECT_NAME}/conversions/l3_msg_foothold_conversions.h
  include/${PROJECT_NAME}/conversions/l3_msg_floating_base_conversions.h
  include/${PROJECT_NAME}/conversions/l3_msg_std_conversions.h
  include/${PROJECT_NAME}/conversions/l3_msg_step_conversions.h
  include/${PROJECT_NAME}/conversions/l3_msg_step_feedback_conversions.h
  include/${PROJECT_NAME}/conversions/l3_kdl_conversions.h
  include/${PROJECT_NAME}/conversions/l3_tf_conversions.h
  include/${PROJECT_NAME}/conversions/serialization.h
  include/${PROJECT_NAME}/robot_description/base_info.h
  include/${PROJECT_NAME}/robot_description/foot_info.h
  include/${PROJECT_NAME}/robot_description/leg_info.h
  include/${PROJECT_NAME}/robot_description/robot_description.h
  include/${PROJECT_NAME}/robot_description/robot_description_server.h
  include/${PROJECT_NAME}/types/abstract_step.h
  include/${PROJECT_NAME}/types/array_2d.h
  include/${PROJECT_NAME}/types/base_link.h
  include/${PROJECT_NAME}/types/base_queue.h
  include/${PROJECT_NAME}/types/boost_types.h
  include/${PROJECT_NAME}/types/eigen_types.h
  include/${PROJECT_NAME}/types/floating_base.h
  include/${PROJECT_NAME}/types/foothold.h
  include/${PROJECT_NAME}/types/joint_states.h
  include/${PROJECT_NAME}/types/memory.h
  include/${PROJECT_NAME}/types/stdlib_types.h
  include/${PROJECT_NAME}/types/variant_data_macros.h
  include/${PROJECT_NAME}/types/step.h
  include/${PROJECT_NAME}/types/base_step_data.h
  include/${PROJECT_NAME}/types/foot_step_data.h
  include/${PROJECT_NAME}/types/step_feedback.h
  include/${PROJECT_NAME}/types/step_feedback_data.h
  include/${PROJECT_NAME}/types/step_feedback_queue.h
  include/${PROJECT_NAME}/types/step_queue.h
  include/${PROJECT_NAME}/types/typedefs.h
  include/${PROJECT_NAME}/types/types.h
  include/${PROJECT_NAME}/types/type_traits.h
  include/${PROJECT_NAME}/types/variant_data.h
  include/${PROJECT_NAME}/helper.h
  include/${PROJECT_NAME}/macros.h
  include/${PROJECT_NAME}/singleton.h
  include/${PROJECT_NAME}/yaml_parser.h
)

set(SOURCES
  src/conversions/l3_msg_std_conversions.cpp
  src/conversions/l3_msg_foothold_conversions.cpp
  src/conversions/l3_msg_floating_base_conversions.cpp
  src/conversions/l3_msg_step_conversions.cpp
  src/conversions/l3_msg_step_feedback_conversions.cpp
  src/conversions/l3_kdl_conversions.cpp
  src/conversions/l3_tf_conversions.cpp
  src/conversions/serialization.cpp
  src/robot_description/base_info.cpp
  src/robot_description/foot_info.cpp
  src/robot_description/leg_info.cpp
  src/robot_description/robot_description.cpp
  src/types/base_link.cpp
  src/types/floating_base.cpp
  src/types/foothold.cpp
  src/types/joint_states.cpp
  src/types/step.cpp
  src/types/base_step_data.cpp
  src/types/foot_step_data.cpp
  src/types/step_queue.cpp
  src/types/step_feedback.cpp
  src/types/step_feedback_data.cpp
  src/types/step_feedback_queue.cpp
  src/types/variant_data.cpp
  src/helper.cpp
  src/yaml_parser.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

## Declare a cpp executable
add_executable(robot_description_server src/robot_description/robot_description_server.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(robot_description_server ${PROJECT_NAME})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

## Specify additional locations of test files
set(TEST_HEADERS
  test/include/${PROJECT_NAME}/test_macros.h
)

set(TEST_SOURCES
  test/src/typedefs_utest.cpp
  test/src/step_data_utest.cpp
  test/src/variant_data_utest.cpp
)

set_source_files_properties(test/src/step_data_utest.cpp PROPERTIES COMPILE_FLAGS "--param=max-vartrack-size=200000000")

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_library(${PROJECT_NAME}-test ${TEST_SOURCES} ${TEST_HEADERS})
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})

  catkin_add_gtest(utest test/src/utest.cpp ${TEST_SOURCES})

  if(TARGET ${PROJECT_NAME}-test)
    target_include_directories(${PROJECT_NAME}-test PUBLIC test/include ${GMOCK_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})
    target_link_libraries(utest ${PROJECT_NAME}-test ${PROJECT_NAME})
  endif()
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
