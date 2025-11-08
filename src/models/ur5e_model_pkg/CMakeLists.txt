cmake_minimum_required(VERSION 3.8)
project(ur5e_model_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # ament_add_pytest_test(ur_urdf_xacro test/test_ur_urdf_xacro.py)
  # ament_add_pytest_test(view_ur_launch test/test_view_ur_launch.py)
endif()

install(
  DIRECTORY launch urdf config meshes worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
