cmake_minimum_required(VERSION 3.8)
project(rm_serial_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_interface REQUIRED) # 必须找到你的自定义消息包

# 编译节点可执行文件
add_executable(rm_serial_driver_node
  src/rm_serial_driver.cpp
  # 如果你的hpp放在include里，也要确保路径正确，比如 include/rm_serial_driver.hpp
)

add_executable(vulnerability_decision_node src/vulnerability_decision_node.cpp)

# 包含头文件路径
target_include_directories(rm_serial_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接 ROS 2 依赖 (非常重要，解决找不到头文件的问题)
ament_target_dependencies(rm_serial_driver_node
  rclcpp
  vision_interface
)
ament_target_dependencies(vulnerability_decision_node 
  rclcpp 
  vision_interface 
  std_msgs
)

# 安装可执行文件
install(TARGETS rm_serial_driver_node vulnerability_decision_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件夹
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)  

ament_package()