cmake_minimum_required(VERSION 3.8)
project(patrol_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Dependencies the OQ-8 CheckpointCapture message will need (04-perception, M6).
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Interface landing slot (C8). M2 ships zero interfaces — rosidl_generate_interfaces
# errors on an empty list, so it is called only once 04-perception lands a .msg/.srv file.
# When that happens the package generates types with no structural change here.
file(GLOB _patrol_interface_files
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "msg/*.msg"
  "srv/*.srv"
  "action/*.action"
)

if(_patrol_interface_files)
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${_patrol_interface_files}
    DEPENDENCIES std_msgs geometry_msgs sensor_msgs diagnostic_msgs
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
