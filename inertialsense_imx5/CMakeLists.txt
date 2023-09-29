cmake_minimum_required(VERSION 3.8)
project(inertialsense_imx5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(inertialsense_msgs REQUIRED)

# setup targets
include_directories(include
  lib/inertial-sense-sdk/src #This line of CMakeList.txt stays in .external file to reference submodule
)

add_subdirectory(lib/inertial-sense-sdk)

add_library(inertialsense_imx5_lib 
        src/inertialsense_imx5.cpp
)

target_link_libraries(inertialsense_imx5_lib InertialSense)
ament_target_dependencies(inertialsense_imx5_lib rclcpp std_msgs geometry_msgs inertialsense_msgs sensor_msgs)

add_executable(inertialsense_imx5_node src/inertialsense_imx5_node.cpp)
target_link_libraries(inertialsense_imx5_node inertialsense_imx5_lib)

ament_target_dependencies(inertialsense_imx5_node inertialsense_msgs rclcpp std_msgs sensor_msgs)

install(TARGETS
inertialsense_imx5_node
inertialsense_imx5_lib
DESTINATION lib/${PROJECT_NAME})

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

ament_package()
