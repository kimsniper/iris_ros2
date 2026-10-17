cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(balance_controller
  src/balance_controller.cpp
)

add_executable(balance_node
  src/balance_node.cpp
)

add_executable(observation_node
  src/observation_node.cpp
)

target_link_libraries(balance_node balance_controller)

ament_target_dependencies(balance_node
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(observation_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  balance_node
  observation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_balance_controller
    test/test_balance_controller.cpp
  )

  target_link_libraries(
    test_balance_controller
    balance_controller
  )
endif()

ament_package()
