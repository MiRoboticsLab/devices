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

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   find_package(ament_cmake_gtest REQUIRED)
#   include_directories(include)
#   ament_add_gtest(configuration_toml_test test/configuration_toml_test.cpp)
#   target_compile_definitions(configuration_toml_test PRIVATE BenchmarkPath="${CMAKE_INSTALL_PREFIX}/lib/cyberdog_bms")
#   ament_target_dependencies(configuration_toml_test rclcpp toml)
#   install(TARGETS
#     configuration_toml_test
#     DESTINATION lib/${PROJECT_NAME})
#   install(FILES
#     test/toml_config.toml
#     DESTINATION lib/${PROJECT_NAME})
#   ament_lint_auto_find_test_dependencies()
# endif()


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

