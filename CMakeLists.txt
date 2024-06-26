cmake_minimum_required(VERSION 3.8)
project(cobot_corrector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(chess_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(OpenCV REQUIRED)

generate_parameter_library(
  cobot_corrector_params
  src/parameters.yaml
)

add_executable(cobot_corrector src/cobot_corrector.cpp src/cobot_corrector_node.cpp)
target_include_directories(cobot_corrector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cobot_corrector PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20

ament_target_dependencies(cobot_corrector PUBLIC
  rclcpp
  urdf
  tf2
  tf2_ros
  tf2_kdl
  orocos_kdl
  kdl_parser
  tf2_geometry_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  chess_msgs
  cv_bridge
  image_transport
)

target_include_directories(cobot_corrector PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(cobot_corrector PRIVATE
  cobot_corrector_params
)

target_link_libraries(cobot_corrector PUBLIC
  ${OpenCV_LIBS}
)

install(TARGETS cobot_corrector
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
