cmake_minimum_required(VERSION 3.10)
project(kinetics_observer_ros2 LANGUAGES C CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(state-observation 1.3 REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KineticsConfiguration.msg"
  "msg/KineticsContactInput.msg"
  "msg/KineticsContactState.msg"
  "msg/KineticsGyroBias.msg"
  "msg/KineticsImuInput.msg"
  "msg/KineticsInput.msg"
  "msg/KineticsKinematics.msg"
  "msg/KineticsState.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_library(kinetics_observer_bridge src/kinetics_observer_bridge.cpp)
target_include_directories(kinetics_observer_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${state-observation_INCLUDE_DIRS}
)
ament_target_dependencies(kinetics_observer_bridge geometry_msgs rclcpp std_msgs)
target_link_libraries(kinetics_observer_bridge state-observation::state-observation)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(kinetics_observer_bridge ${cpp_typesupport_target})

add_executable(kinetics_observer_node src/kinetics_observer_node.cpp)
target_link_libraries(kinetics_observer_node kinetics_observer_bridge)
ament_target_dependencies(kinetics_observer_node rclcpp)

install(TARGETS kinetics_observer_bridge kinetics_observer_node
  EXPORT export_kinetics_observer_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinetics_observer_bridge test/test_kinetics_observer_bridge.cpp)
  target_link_libraries(test_kinetics_observer_bridge kinetics_observer_bridge state-observation::state-observation)
  target_include_directories(test_kinetics_observer_bridge PRIVATE ${state-observation_INCLUDE_DIRS})
endif()

ament_export_targets(export_kinetics_observer_bridge HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs rclcpp rosidl_default_runtime state-observation std_msgs)
ament_package()
