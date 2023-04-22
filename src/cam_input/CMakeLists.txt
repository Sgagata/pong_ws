cmake_minimum_required(VERSION 3.8)
project(cam_input)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

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


add_executable(light_position src/light_position.cpp)
target_include_directories(light_position  PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(light_position rclcpp sensor_msgs std_msgs cv_bridge image_transport geometry_msgs)
target_link_libraries(light_position ${OpenCV_LIBS} cv_bridge::cv_bridge)


add_executable(light_position_subscriber src/light_position_subscriber.cpp)
target_include_directories(light_position_subscriber  PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(light_position_subscriber rclcpp sensor_msgs geometry_msgs)
target_link_libraries(light_position_subscriber ${OpenCV_LIBS})


add_executable(light_plotter src/light_plotter.cpp)
# target_include_directories(light_position_subscriber  PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(light_plotter rclcpp geometry_msgs)

# Install executable
install(TARGETS
  light_position
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  light_position_subscriber
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  light_plotter
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})



ament_export_dependencies(rclcpp sensor_msgs cv_bridge image_transport geometry_msgs)
ament_export_include_directories(${OpenCV_INCLUDE_DIRS})
ament_export_libraries(${OpenCV_LIBS})

ament_package()
