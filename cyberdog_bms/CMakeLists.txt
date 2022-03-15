cmake_minimum_required(VERSION 3.8)
project(cyberdog_bms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(protocol REQUIRED)
find_package(cyberdog_embed_protocol REQUIRED)
find_package(cyberdog_system REQUIRED) 
find_package(cyberdog_common REQUIRED)
find_package(params REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/bms_plugin.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  ${PROJECT_NAME}
  pluginlib
  protocol
  cyberdog_embed_protocol
  cyberdog_system
  cyberdog_common
  params
  ament_index_cpp
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "BMS_PLUGIN_BUILDING_LIBRARY")
pluginlib_export_plugin_description_file( ${PROJECT_NAME} plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  #DESTINATION lib/${PROJECT_NAME})
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()

