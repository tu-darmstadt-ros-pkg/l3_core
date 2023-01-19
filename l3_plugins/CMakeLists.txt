cmake_minimum_required(VERSION 3.0.2)
project(l3_plugins)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Compile as C++17, supported in ROS Melodic and newer
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  geometry_msgs
  sensor_msgs
  kdl_parser
  vigir_pluginlib
  l3_libs
  l3_math
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

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
  INCLUDE_DIRS include
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
  include/${PROJECT_NAME}/aggregator/joint_cmd_interface_aggregator.h
  include/${PROJECT_NAME}/base/dynamics_plugin.h
  include/${PROJECT_NAME}/base/ft_sensor_plugin.h
  include/${PROJECT_NAME}/base/gait_generator_plugin.h
  include/${PROJECT_NAME}/base/imu_sensor_plugin.h
  include/${PROJECT_NAME}/base/interface_plugin.h
  include/${PROJECT_NAME}/base/joint_cmd_interface_plugin.h
  include/${PROJECT_NAME}/base/joint_states_plugin.h
  include/${PROJECT_NAME}/base/kinematics_plugin.h
  include/${PROJECT_NAME}/std/bipedal_gait_generator.h
  include/${PROJECT_NAME}/std/cyclic_gait_generator.h
  include/${PROJECT_NAME}/std/quadrupedal_gait_generator.h
  include/${PROJECT_NAME}/std/ft_sensor_topic.h
  include/${PROJECT_NAME}/std/imu_sensor_topic.h
  include/${PROJECT_NAME}/std/joint_cmd_interface_topic.h
  include/${PROJECT_NAME}/std/joint_states_topic.h
  include/${PROJECT_NAME}/std/kdl_kinematics.h
  include/${PROJECT_NAME}/robot_model.h
  include/${PROJECT_NAME}/robot_model_server.h
)

set(SOURCES
  src/aggregator/joint_cmd_interface_aggregator.cpp
  src/base/dynamics_plugin.cpp
  src/base/ft_sensor_plugin.cpp
  src/base/gait_generator_plugin.cpp
  src/base/imu_sensor_plugin.cpp
  src/base/interface_plugin.cpp
  src/base/joint_cmd_interface_plugin.cpp
  src/base/joint_states_plugin.cpp
  src/base/kinematics_plugin.cpp
  src/std/bipedal_gait_generator.cpp
  src/std/cyclic_gait_generator.cpp
  src/std/quadrupedal_gait_generator.cpp
  src/std/ft_sensor_topic.cpp
  src/std/imu_sensor_topic.cpp
  src/std/joint_cmd_interface_topic.cpp
  src/std/joint_states_topic.cpp
  src/std/kdl_kinematics.cpp
  src/robot_model.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

## Declare a cpp executable
add_executable(robot_model_server src/robot_model_server.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${kdl_parser_LIBRARIES} ${orocos_kdl_LIBRARIES})
target_link_libraries(robot_model_server ${PROJECT_NAME})

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

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_l3_plugins.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
