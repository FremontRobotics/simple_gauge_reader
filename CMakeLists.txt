cmake_minimum_required(VERSION 3.22)
project(simple_gauge_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(simple_gauge_reader_interfaces REQUIRED)

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

add_executable(simple_gauge_reader src/gauge.cpp src/simple_gauge_reader.cpp)
add_executable(simple_gauge_reader_from_image src/gauge.cpp  src/simple_gauge_reader_from_image.cpp)
add_executable(simple_gauge_reader_from_compressed_image src/gauge.cpp  src/simple_gauge_reader_from_compressed_image.cpp)

ament_target_dependencies(simple_gauge_reader rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV simple_gauge_reader_interfaces)
ament_target_dependencies(simple_gauge_reader_from_image rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV simple_gauge_reader_interfaces)
ament_target_dependencies(simple_gauge_reader_from_compressed_image rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV simple_gauge_reader_interfaces)

target_include_directories(simple_gauge_reader PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(simple_gauge_reader_from_image PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(simple_gauge_reader_from_compressed_image PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  simple_gauge_reader
  simple_gauge_reader_from_image
  simple_gauge_reader_from_compressed_image
  DESTINATION lib/${PROJECT_NAME})


ament_package()
