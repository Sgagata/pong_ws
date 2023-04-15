cmake_minimum_required(VERSION 3.8)
project(pong_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_messages REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(right_bar_position src/right_bar_position.cpp)
ament_target_dependencies(right_bar_position rclcpp custom_messages geometry_msgs)

add_executable(left_bar_position src/left_bar_position.cpp)
ament_target_dependencies(left_bar_position rclcpp custom_messages geometry_msgs)

add_executable(game_state_publisher src/game_state_publisher.cpp)
ament_target_dependencies(game_state_publisher rclcpp custom_messages)

add_executable(terminal_input src/terminal_input.cpp)
ament_target_dependencies(terminal_input rclcpp custom_messages geometry_msgs)

add_executable(window_server src/window_size_server.cpp)
ament_target_dependencies(window_server rclcpp custom_messages)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Install executable
install(TARGETS
  right_bar_position
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  left_bar_position
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  game_state_publisher
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  terminal_input
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  window_server
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rclcpp custom_messages geometry_msgs)


ament_package()
