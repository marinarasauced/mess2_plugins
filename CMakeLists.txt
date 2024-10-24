cmake_minimum_required(VERSION 3.8)
project(mess2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mess2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(_calibration SHARED src/calibration.cpp)
add_library(_rotation SHARED src/rotation.cpp)
add_library(_threat SHARED src/threat.cpp)
add_library(_ugv SHARED src/ugv.cpp)
add_library(_utils SHARED src/utils.cpp)

ament_target_dependencies(
  _calibration 
  "rclcpp" 
  "geometry_msgs" 
  "mess2_msgs"
)

ament_target_dependencies(
  _rotation 
  "rclcpp" 
  "geometry_msgs" 
  "mess2_msgs"
)

ament_target_dependencies(
  _threat 
  "rclcpp" 
  "geometry_msgs"
  "mess2_msgs"
  "sensor_msgs"
)

ament_target_dependencies(
  _ugv 
  "rclcpp" 
  "geometry_msgs" 
  "mess2_msgs"
)

ament_target_dependencies(
  _utils
  "rclcpp" 
  "geometry_msgs" 
  "mess2_msgs"
)

install(
  TARGETS _calibration _rotation _threat _ugv _utils
  EXPORT export_mess2_plugins
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_targets(
  export_mess2_plugins
)

ament_package()