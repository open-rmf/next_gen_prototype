cmake_minimum_required(VERSION 3.5)

project(rmf_prototype_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/ControlPoint.msg"
  "msg/Curve.msg"
  "msg/Destination.msg"
  "msg/DestinationConstraints.msg"
  "msg/DestinationError.msg"
  "msg/DestinationGoal.msg"
  "msg/NodeKey.msg"
  "msg/Participant.msg"
  "msg/Plan.msg"
  "msg/PlanError.msg"
  "msg/PlanId.msg"
  "msg/PlanRelease.msg"
  "msg/Progress.msg"
  "msg/Region.msg"
  "msg/SafeZone.msg"
  "msg/SafeZoneId.msg"
  "msg/SessionRefresh.msg"
  "msg/TargetNode.msg"
  "msg/TargetOrientation.msg"
  "msg/TargetRegion.msg"
  "msg/TrafficDependency.msg"
  "msg/Trajectory.msg"
  "msg/Waypoint.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces nav2_msgs unique_identifier_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
