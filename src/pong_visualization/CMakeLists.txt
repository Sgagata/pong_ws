cmake_minimum_required(VERSION 3.8)
project(pong_visualization)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Discover where SDL2 is
find_package(ament_cmake REQUIRED)
find_package(SDL2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_messages REQUIRED)
find_package(geometry_msgs REQUIRED)
string(STRIP ${SDL2_LIBRARIES} SDL2_LIBRARIES)
set(SDL2_TTF_LIBRARIES SDL2_ttf) # Officially this should be done with find_package, but due to a bug that does not work

# Tell which files to compile for the executable
add_executable(${PROJECT_NAME} 
    # src/example.cpp 
    src/pong_visualization.cpp
    src/SDL2_UI.cpp 
    src/Pong_field.cpp
)


# Include the SDL libraries (as found above) for this executable
# target_link_directories(${PROJECT_NAME} PUBLIC ${SDL2_LINK_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${SDL2_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} rclcpp custom_messages geometry_msgs)

target_link_libraries(${PROJECT_NAME} ${SDL2_LIBRARIES} ${SDL2_TTF_LIBRARIES})
# target_link_libraries(${PROJECT_NAME} PUBLIC SDL2::SDL2 SDL2::SDL2_ttf)

install(TARGETS
  pong_visualization
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()