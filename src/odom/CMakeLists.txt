cmake_minimum_required(VERSION 3.8)
project(odom)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(auna_common REQUIRED)

find_package(auna_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(auna_its_msgs REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)

# Include Cpp "include" directory
include_directories(include)

add_library(odom_node SHARED src/odom_node/odom_node.cpp)

# Create Cpp executable
add_executable(odom_node_main src/odom_node/odom_node_main.cpp)

# Target dependencies
set(odom_node_dependencies rclcpp std_msgs geometry_msgs nav_msgs auna_its_msgs)
ament_target_dependencies(odom_node ${odom_node_dependencies})
ament_target_dependencies(odom_node_main ${odom_node_dependencies})

# Link libraries
target_link_libraries(odom_node_main odom_node)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Cpp executables
install(TARGETS
  odom_node_main
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  odom_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
