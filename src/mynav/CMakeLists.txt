cmake_minimum_required(VERSION 3.8)
project(mynav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(waypoint_config src/waypoint_config.cpp)
ament_auto_add_executable(waypoint_runner src/waypoint_runner.cpp)
target_link_libraries(waypoint_runner waypoint_config)
find_package(yaml-cpp REQUIRED)
target_link_libraries(waypoint_config ${YAML_CPP_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_waypoint_config test/test_waypoint_config.cpp)
  target_link_libraries(test_waypoint_config waypoint_config)
  install(TARGETS test_waypoint_config RUNTIME DESTINATION lib/${PROJECT_NAME})
  ament_lint_auto_find_test_dependencies()
endif()

set(USE_SCOPED_HEADER_INSTALL_DIR TRUE)
ament_auto_package(INSTALL_TO_SHARE maps config launch)
