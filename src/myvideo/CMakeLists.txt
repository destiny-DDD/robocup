cmake_minimum_required(VERSION 3.8)
project(myvideo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# OpenCV 的 rosdep key 是 libopencv-dev，但 CMake 包名是 OpenCV，
# 两者不一致，ament_auto_find_build_dependencies 不会自动找到它，
# 所以这里手动 find_package 并链接
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include/${PROJECT_NAME})
file(GLOB_RECURSE SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
ament_auto_add_executable(${PROJECT_NAME} ${SRC})

# target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS}) 不能记录依赖关系，而且include是隐式添加，我不喜欢
ament_target_dependencies(${PROJECT_NAME} OpenCV)
# 链接 myserial 串口驱动库
ament_target_dependencies(${PROJECT_NAME} myserial)
ament_target_dependencies(${PROJECT_NAME} ament_index_cpp)

install(DIRECTORY image
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(USE_SCOPED_HEADER_INSTALL_DIR TRUE)
ament_auto_package(INSTALL_TO_SHARE image)
