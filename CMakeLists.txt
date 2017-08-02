cmake_minimum_required(VERSION 2.8.3)
project(aerial_autonomy)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "-g -Werror -Wall -std=c++11 --coverage ${CMAKE_CXX_FLAGS}")
add_definitions(-DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS -DBOOST_MPL_LIMIT_VECTOR_SIZE=30)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  parsernode
  class_loader
  pluginlib
  rostest
  tf_conversions
  quad_simulator_parser
  tf
  cv_bridge
  image_transport
  arm_plugins
  ar_track_alvar_msgs
)

include(cmake/add_event_function.cmake)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Protobuf 3.1 REQUIRED) 
find_package(Glog REQUIRED)

# Quad simulator parser dependencies
find_package(GCOP REQUIRED systems)
include_directories(${GCOP_INCLUDE_DIRS})
set(QUAD_SIM_PARSER_LIBS ${QUAD_SIM_PARSER_LIBS} ${GCOP_LIBRARIES})

find_package(TinyXML REQUIRED)
include_directories(${TINYXML_INCLUDE_DIR})
set(QUAD_SIM_PARSER_LIBS ${QUAD_SIM_PARSER_LIBS} ${TINYXML_LIBRARIES})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
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
  LIBRARIES aerial_autonomy
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

######################
## Event Generation ##
######################
## Generates event header files
## ADD ${${PROJECT_NAME}_EXPORTED_TARGETS} as a dependency to any
## targets that use the event header files
## This target is also common to messages, cfg files etc
## Add ${EVENT_INCLUDE_DIR} to include_directories
## To use the event header files in source code, include
## them as #include <${PROJECT_NAME}/event_file_name.h>
## The header file contains the structs to define events
## It also adds an event manager class templated on StateMachine
## to trigger events based on name:
##  void triggerEvent(event_name, state_machine)
## It also lets you print all the event names created:
##  void printEventList()
generate_event_targets(
uav_basic_events
visual_servoing_events
pick_place_events
arm_events
)

##############
## Protobuf ##
##############
PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER 
  proto/uav_system_config.proto
  proto/uav_vision_system_config.proto
  proto/uav_arm_system_config.proto
  proto/common_system_handler_config.proto
  proto/uav_system_handler_config.proto
  proto/uav_arm_system_handler_config.proto
  proto/velocity_based_position_controller_config.proto
  proto/constant_heading_depth_controller_config.proto
  proto/position_controller_config.proto
  proto/pose_controller_config.proto
  proto/velocity_controller_config.proto
)
add_library(proto ${PROTO_HEADER} ${PROTO_SRC})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${GLOG_INCLUDE_DIR}
)

## Declare a C++ library
add_library(aerial_autonomy
  src/common/async_timer.cpp
  src/common/math.cpp
  src/common/conversions.cpp
  src/trackers/roi_to_position_converter.cpp
  src/trackers/simple_tracker.cpp
  src/trackers/alvar_tracker.cpp
  src/trackers/base_tracker.cpp
  src/trackers/simple_multi_tracker.cpp
  src/trackers/closest_tracking_strategy.cpp
  src/trackers/simple_tracking_strategy.cpp
  src/controllers/manual_rpyt_controller.cpp
  src/controllers/velocity_based_position_controller.cpp
  src/controllers/constant_heading_depth_controller.cpp
  src/controllers/relative_pose_controller.cpp
  src/controller_hardware_connectors/visual_servoing_controller_drone_connector.cpp
  src/controller_hardware_connectors/visual_servoing_controller_arm_connector.cpp
  src/controller_hardware_connectors/position_controller_drone_connector.cpp
  src/controller_hardware_connectors/builtin_velocity_controller_drone_connector.cpp
  src/controller_hardware_connectors/manual_rpyt_controller_drone_connector.cpp
)
add_dependencies(aerial_autonomy proto)
target_link_libraries(aerial_autonomy ${catkin_LIBRARIES})


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(aerial_autonomy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(uav_system_node src/system_handler_nodes/uav_system_node.cpp)
add_executable(uav_vision_system_node src/system_handler_nodes/uav_vision_system_node.cpp)
add_executable(uav_arm_system_node src/system_handler_nodes/uav_arm_system_node.cpp)
add_executable(event_publish_node src/tests/event_publish_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(aerial_autonomy_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(uav_system_node aerial_autonomy proto ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
target_link_libraries(uav_vision_system_node aerial_autonomy proto ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
target_link_libraries(uav_arm_system_node aerial_autonomy proto ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
target_link_libraries(event_publish_node ${catkin_LIBRARIES})

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
# install(TARGETS aerial_autonomy aerial_autonomy_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-logic-states-test tests/logic_states/base_state_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-ctrlr-hrdwr-cnctr-test tests/controller_hardware_connectors/controller_hardware_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-evnt-mngr-test tests/event_manager_tests/event_manager_tests.cpp)
add_dependencies(${PROJECT_NAME}-evnt-mngr-test ${${PROJECT_NAME}_EXPORTED_TARGETS})
catkin_add_gtest(${PROJECT_NAME}-type-map-test tests/common/type_map_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-pos-ctrlr-drone-cnctr-test tests/controller_hardware_connectors/position_controller_drone_connector_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-builtin-vel-ctrlr-drone-cnctr-test tests/controller_hardware_connectors/builtin_velocity_controller_drone_connector_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-manual-rpyt-ctrlr-drone-cnctr-test tests/controller_hardware_connectors/manual_rpyt_controller_drone_connector_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-uav-system-test tests/robot_systems/uav_system_tests.cpp)
add_rostest_gtest(${PROJECT_NAME}-uav-arm-system-test tests/robot_systems/uav_arm_system_tests.test tests/robot_systems/uav_arm_system_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-uav-basic-functor-tests tests/actions_guards/uav_basic_functor_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-visual-servoing-functor-tests tests/actions_guards/visual_servoing_functor_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-pick-place-functor-tests tests/actions_guards/pick_place_functor_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-uav-basic-state-machine-test tests/state_machines/uav_state_machine_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-visual-servoing-state-machine-test tests/state_machines/visual_servoing_state_machine_tests.cpp)
add_dependencies(${PROJECT_NAME}-visual-servoing-state-machine-test ${${PROJECT_NAME}_EXPORTED_TARGETS})
catkin_add_gtest(${PROJECT_NAME}-pick-place-state-machine-test tests/state_machines/pick_place_state_machine_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-arm-system-test tests/robot_systems/arm_system_tests.cpp)
add_rostest_gtest(${PROJECT_NAME}-uav-vision-system-test tests/robot_systems/uav_vision_system_tests.test tests/robot_systems/uav_vision_system_tests.cpp)
add_rostest_gtest(${PROJECT_NAME}-state-machine-gui-connector-event-test tests/state_machines/state_machine_gui_connector_event_tests.test tests/state_machines/state_machine_gui_connector_event_tests.cpp)
add_dependencies(${PROJECT_NAME}-state-machine-gui-connector-event-test ${${PROJECT_NAME}_EXPORTED_TARGETS} event_publish_node)
add_rostest_gtest(${PROJECT_NAME}-state-machine-gui-connector-pose-test tests/state_machines/state_machine_gui_connector_pose_tests.test tests/state_machines/state_machine_gui_connector_pose_tests.cpp)
add_dependencies(${PROJECT_NAME}-state-machine-gui-connector-pose-test ${${PROJECT_NAME}_EXPORTED_TARGETS} event_publish_node)
catkin_add_gtest(${PROJECT_NAME}-async-timer-test tests/common/async_timer_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-atomic-test tests/common/atomic_tests.cpp)
add_rostest_gtest(${PROJECT_NAME}-uav-system-handler-test tests/system_handlers/uav_system_handler_tests.test tests/system_handlers/uav_system_handler_tests.cpp)
add_dependencies(${PROJECT_NAME}-uav-system-handler-test ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_rostest_gtest(${PROJECT_NAME}-uav-vision-system-handler-test tests/system_handlers/uav_vision_system_handler_tests.test tests/system_handlers/uav_vision_system_handler_tests.cpp)
add_dependencies(${PROJECT_NAME}-uav-vision-system-handler-test ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_rostest_gtest(${PROJECT_NAME}-uav-arm-system-handler-test tests/system_handlers/uav_arm_system_handler_tests.test tests/system_handlers/uav_arm_system_handler_tests.cpp)
add_dependencies(${PROJECT_NAME}-uav-arm-system-handler-test ${${PROJECT_NAME}_EXPORTED_TARGETS})
catkin_add_gtest(${PROJECT_NAME}-thread-safe-state-machine-test tests/common/thread_safe_state_machine_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-iterable-enum-test tests/common/iterable_enum_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-velocity-based-position-controller-test tests/controllers/velocity_based_position_controller_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-relative-pose-controller-test tests/controllers/relative_pose_controller_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-constant-heading-depth-controller-test tests/controllers/constant_heading_depth_controller_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-math-test tests/common/math_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-simple-tracker-test tests/trackers/simple_tracker_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-simple-multi-tracker-test tests/trackers/simple_multi_tracker_tests.cpp)
add_rostest_gtest(${PROJECT_NAME}-roi-to-position-converter-test tests/trackers/roi_to_position_converter_tests.test tests/trackers/roi_to_position_converter_tests.cpp)
add_rostest_gtest(${PROJECT_NAME}-alvar-tracker-test tests/trackers/alvar_tracker_tests.test tests/trackers/alvar_tracker_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-visual-servoing-drone-connector-test tests/controller_hardware_connectors/visual_servoing_drone_connector_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}-visual-servoing-arm-connector-test tests/controller_hardware_connectors/visual_servoing_arm_connector_tests.cpp)
if(TARGET ${PROJECT_NAME}-uav-basic-state-machine-test)
  target_link_libraries(${PROJECT_NAME}-uav-basic-state-machine-test aerial_autonomy proto ${catkin_LIBRARIES} ${QUAD_SIM_PARSER_LIBS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-visual-servoing-state-machine-test)
  target_link_libraries(${PROJECT_NAME}-visual-servoing-state-machine-test aerial_autonomy proto ${catkin_LIBRARIES} ${QUAD_SIM_PARSER_LIBS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-pick-place-state-machine-test)
  target_link_libraries(${PROJECT_NAME}-pick-place-state-machine-test aerial_autonomy proto ${catkin_LIBRARIES} ${QUAD_SIM_PARSER_LIBS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-pos-ctrlr-drone-cnctr-test)
  target_link_libraries(${PROJECT_NAME}-pos-ctrlr-drone-cnctr-test aerial_autonomy proto ${catkin_LIBRARIES} ${QUAD_SIM_PARSER_LIBS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-builtin-vel-ctrlr-drone-cnctr-test)
  target_link_libraries(${PROJECT_NAME}-builtin-vel-ctrlr-drone-cnctr-test aerial_autonomy proto ${catkin_LIBRARIES} ${QUAD_SIM_PARSER_LIBS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-manual-rpyt-ctrlr-drone-cnctr-test)
  target_link_libraries(${PROJECT_NAME}-manual-rpyt-ctrlr-drone-cnctr-test aerial_autonomy proto ${catkin_LIBRARIES} ${QUAD_SIM_PARSER_LIBS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-logic-states-test)
  target_link_libraries(${PROJECT_NAME}-logic-states-test proto ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-evnt-mngr-test)
  target_link_libraries(${PROJECT_NAME}-evnt-mngr-test proto ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-uav-system-test)
  target_link_libraries(${PROJECT_NAME}-uav-system-test aerial_autonomy proto ${catkin_LIBRARIES} ${QUAD_SIM_PARSER_LIBS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-uav-arm-system-test)
  target_link_libraries(${PROJECT_NAME}-uav-arm-system-test aerial_autonomy proto ${catkin_LIBRARIES} ${QUAD_SIM_PARSER_LIBS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-arm-system-test)
  target_link_libraries(${PROJECT_NAME}-arm-system-test ${catkin_LIBRARIES} ${GCOP_LIBRARIES} ${TINYXML_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-uav-basic-functor-tests)
  target_link_libraries(${PROJECT_NAME}-uav-basic-functor-tests aerial_autonomy proto ${catkin_LIBRARIES} ${QUAD_SIM_PARSER_LIBS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-pick-place-functor-tests)
  target_link_libraries(${PROJECT_NAME}-pick-place-functor-tests aerial_autonomy proto ${catkin_LIBRARIES} ${QUAD_SIM_PARSER_LIBS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-visual-servoing-functor-tests)
  target_link_libraries(${PROJECT_NAME}-visual-servoing-functor-tests aerial_autonomy proto ${catkin_LIBRARIES} ${QUAD_SIM_PARSER_LIBS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-ctrlr-hrdwr-cnctr-test)
  target_link_libraries(${PROJECT_NAME}-ctrlr-hrdwr-cnctr-test ${Boost_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-state-machine-gui-connector-event-test)
  target_link_libraries(${PROJECT_NAME}-state-machine-gui-connector-event-test aerial_autonomy proto ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-state-machine-gui-connector-pose-test)
  target_link_libraries(${PROJECT_NAME}-state-machine-gui-connector-pose-test aerial_autonomy proto ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-async-timer-test)
  target_link_libraries(${PROJECT_NAME}-async-timer-test aerial_autonomy proto ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-atomic-test)
  target_link_libraries(${PROJECT_NAME}-atomic-test ${Boost_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-thread-safe-state-machine-test)
  target_link_libraries(${PROJECT_NAME}-thread-safe-state-machine-test ${Boost_LIBRARIES} ${catkin_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-uav-system-handler-test)
  target_link_libraries(${PROJECT_NAME}-uav-system-handler-test aerial_autonomy proto ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-uav-vision-system-handler-test)
  target_link_libraries(${PROJECT_NAME}-uav-vision-system-handler-test aerial_autonomy proto ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-uav-arm-system-handler-test)
  target_link_libraries(${PROJECT_NAME}-uav-arm-system-handler-test aerial_autonomy proto ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-uav-vision-system-test)
  target_link_libraries(${PROJECT_NAME}-uav-vision-system-test aerial_autonomy proto ${catkin_LIBRARIES} ${QUAD_SIM_PARSER_LIBS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-velocity-based-position-controller-test)
  target_link_libraries(${PROJECT_NAME}-velocity-based-position-controller-test aerial_autonomy proto ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-relative-pose-controller-test)
  target_link_libraries(${PROJECT_NAME}-relative-pose-controller-test aerial_autonomy proto ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-constant-heading-depth-controller-test)
  target_link_libraries(${PROJECT_NAME}-constant-heading-depth-controller-test aerial_autonomy proto ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-visual-servoing-drone-connector-test)
  target_link_libraries(${PROJECT_NAME}-visual-servoing-drone-connector-test aerial_autonomy proto ${catkin_LIBRARIES} ${QUAD_SIM_PARSER_LIBS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-visual-servoing-arm-connector-test)
  target_link_libraries(${PROJECT_NAME}-visual-servoing-arm-connector-test aerial_autonomy proto ${catkin_LIBRARIES} ${QUAD_SIM_PARSER_LIBS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-roi-to-position-converter-test)
  target_link_libraries(${PROJECT_NAME}-roi-to-position-converter-test aerial_autonomy proto ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-alvar-tracker-test)
  target_link_libraries(${PROJECT_NAME}-alvar-tracker-test aerial_autonomy proto ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-simple-tracker-test)
  target_link_libraries(${PROJECT_NAME}-simple-tracker-test aerial_autonomy proto ${catkin_LIBRARIES} ${QUAD_SIM_PARSER_LIBS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-simple-multi-tracker-test)
  target_link_libraries(${PROJECT_NAME}-simple-multi-tracker-test aerial_autonomy proto ${catkin_LIBRARIES} ${QUAD_SIM_PARSER_LIBS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()
if(TARGET ${PROJECT_NAME}-math-test)
  target_link_libraries(${PROJECT_NAME}-math-test aerial_autonomy proto ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

## Configure script to generate test coverage
configure_file(scripts/generate_test_coverage.bash.in ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_test_coverage.bash)
## Configure script to generate dot files for state machines
configure_file(scripts/generate_dot_files.py.in ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_dot_files.py)
