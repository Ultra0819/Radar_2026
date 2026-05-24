cmake_minimum_required(VERSION 3.8)
project(video_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# --- 新增的编译规则 ---
add_executable(video_publisher_node src/video_publisher_node.cpp)
target_include_directories(video_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(video_publisher_node
  rclcpp
  sensor_msgs
  cv_bridge
  )

  target_link_libraries(video_publisher_node ${OpenCV_LIBRARIES})

install(TARGETS video_publisher_node
  DESTINATION lib/${PROJECT_NAME})
# --------------------

ament_package()
