cmake_minimum_required(VERSION 3.8)
project(device_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cyberdog_system REQUIRED)
find_package(protocol REQUIRED)

set(GTEST_ROOT /opt/google/gtest)
find_package(GTest REQUIRED)

add_executable(led_test src/led_test.cpp)
target_link_libraries(led_test gtest gtest_main)

ament_target_dependencies(led_test
  rclcpp
  protocol
  cyberdog_system
  )


install(TARGETS
  led_test
  DESTINATION lib/${PROJECT_NAME})

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





ament_package()
