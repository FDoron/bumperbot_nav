cmake_minimum_required(VERSION 3.5)
project(bumperbot_description)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(urdf REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch meshes models photos urdf rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  "${PROJECT_NAME}/range_to_float32.py"
  DESTINATION lib/${PROJECT_NAME}
  RENAME range_to_float32
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()