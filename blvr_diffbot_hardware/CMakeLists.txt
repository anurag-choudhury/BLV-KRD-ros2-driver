cmake_minimum_required(VERSION 3.5)
project(blvr_diffbot_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(std_srvs REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

include_directories(${LIBMODBUS_INCLUDE_DIRS})
link_directories(${LIBMODBUS_LIBRARY_DIRS})

## COMPILE
add_library(${PROJECT_NAME} SHARED
  src/blvr_diffbot_system.cpp
  src/blvr_comunicator.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_srvs
)

pluginlib_export_plugin_description_file(hardware_interface blvr_diffbot_hardware.xml)

target_link_libraries(${PROJECT_NAME}
  ${LIBMODBUS_LIBRARIES}
)

## STOP SERVER EXECUTABLE
add_executable(blvr_stop_server src/blvr_stop_server.cpp)
ament_target_dependencies(blvr_stop_server
  rclcpp
  std_srvs
)
target_link_libraries(blvr_stop_server
  ${PROJECT_NAME}   # link against the library target defined above
)

## INSTALL
install(TARGETS
  ${PROJECT_NAME}
  blvr_stop_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

## EXPORTS
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()
