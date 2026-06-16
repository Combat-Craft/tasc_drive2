cmake_minimum_required(VERSION 3.8)
project(phidgets_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
include_directories(include)

add_library(phidgets_hardware SHARED
  src/phidgets_bldc_hardware.cpp
)

ament_target_dependencies(phidgets_hardware
  rclcpp
  hardware_interface
  pluginlib
  std_msgs
  std_srvs
)

add_executable(ps4_teleop src/ps4_teleop.cpp)

ament_target_dependencies(ps4_teleop
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS
  ps4_teleop
  DESTINATION lib/${PROJECT_NAME}
)

# Phidgets (Phidget22) C library + headers
# On Ubuntu, this usually works if you installed libphidget22 and dev headers.
target_link_libraries(phidgets_hardware
  phidget22
)

pluginlib_export_plugin_description_file(hardware_interface phidgets_hardware_plugin.xml)

install(TARGETS phidgets_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES phidgets_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
