cmake_minimum_required(VERSION 3.8)
project(ekf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(auna_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(auna_its_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(autodiff REQUIRED)

# Include Cpp "include" directory
include_directories(include)

add_library(ekf_node SHARED src/ekf_node.cpp)

# Create Cpp executable
add_executable(ekf_node_main src/ekf_node_main.cpp)

# Target dependencies
set(ekf_node_dependencies rclcpp std_msgs geometry_msgs nav_msgs auna_its_msgs sensor_msgs Eigen3)
ament_target_dependencies(ekf_node ${ekf_node_dependencies})
ament_target_dependencies(ekf_node_main ${ekf_node_dependencies})

# Link libraries
target_link_libraries(ekf_node_main ekf_node)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Cpp executables
install(TARGETS
  ekf_node_main
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  ekf_node
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