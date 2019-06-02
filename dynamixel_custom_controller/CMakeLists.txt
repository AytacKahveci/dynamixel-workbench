cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_custom_controller)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  control_msgs
  actionlib
  dynamixel_sdk
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  tf
  rosconsole
  urdf
  xacro
  sensor_msgs
  controller_manager
  hardware_interface
  control_toolbox
)

catkin_package(
 INCLUDE_DIRS include
 #LIBRARIES dynamixel_workbench_controllers dynamixel_workbench_toolbox hardware_interface
 CATKIN_DEPENDS roscpp std_msgs dynamixel_workbench_msgs tf dynamixel_workbench_toolbox
  controller_manager
  hardware_interface
  control_toolbox
  sensor_msgs
  control_msgs
  actionlib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(keyboardControl src/keyboardControl.cpp)
add_dependencies(keyboardControl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(keyboardControl ${catkin_LIBRARIES})

add_library(dynamixelCommonInterface src/dynamixelCommonInterface.cpp)
target_link_libraries(dynamixelCommonInterface ${catkin_LIBRARIES})
add_executable(dynamixelCommonController src/dynamixelCommonController.cpp src/dynamixelCommonInterface.cpp)
target_link_libraries(dynamixelCommonController ${catkin_LIBRARIES})

add_executable(dynamixel_limits_node src/dynamixel_limits_node.cpp)
target_link_libraries(dynamixel_limits_node ${catkin_LIBRARIES})

add_executable(dynamixel_limits_controller src/dynamixelCommonController.cpp src/dynamixel_limits_interface.cpp)
target_link_libraries(dynamixel_limits_controller ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
