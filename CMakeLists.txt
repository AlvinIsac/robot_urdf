cmake_minimum_required(VERSION 3.5)
project(robot_urdf)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


install(DIRECTORY
  launch urdf config worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(
    PROGRAMS src/rotate_robot.py  # Correctly specify the Python script
    DESTINATION lib/${PROJECT_NAME}  # Install the script as an executable
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
