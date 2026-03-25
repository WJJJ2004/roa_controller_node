cmake_minimum_required(VERSION 3.8)
project(roa_controller_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(roa_interfaces REQUIRED)
find_package(roa_policy_driver REQUIRED)

add_executable(roa_controller_node
  src/roa_controller_node.cpp
)

ament_target_dependencies(roa_controller_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  roa_interfaces
)

target_link_libraries(roa_controller_node
  roa::roa_policy_driver
)

target_include_directories(roa_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# add_executable(system_manager_node
#   src/system_manager_node.cpp
# )

# ament_target_dependencies(system_manager_node
#   rclcpp
#   lifecycle_msgs
#   roa_interfaces
# )

# target_include_directories(system_manager_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

install(TARGETS
  roa_controller_node
  # system_manager_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()