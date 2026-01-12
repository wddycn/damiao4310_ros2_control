cmake_minimum_required(VERSION 3.8)
project(damiao_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-sign-compare)  # 关闭符号比较警告
endif()

# 查找依赖（补充rclcpp/logging）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ros2_control REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)  # 补充生命周期依赖

# 添加头文件路径
include_directories(
  include
  include/damiao_ros2_control  # 驱动头文件所在目录
)

# 编译硬件接口库
add_library(
  damiao_ros2_control
  SHARED
  src/damiao_system.cpp
  # 如果有 damiao 或 SerialPort 的实现文件，需要添加到这里
  # 例如：src/damiao.cpp src/SerialPort.cpp
)

# 链接依赖库（补充rclcpp_lifecycle）
ament_target_dependencies(
  damiao_ros2_control
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  ros2_control
  lifecycle_msgs
  pluginlib
)

# 安装库文件
install(
  TARGETS damiao_ros2_control
  DESTINATION lib
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装插件描述文件
install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# 安装launch文件（关键：将launch目录下的所有文件安装到share目录）
install(
  DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME}
)


# 导出插件
pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

# 导出包含路径
ament_export_include_directories(
  include
)

# 导出库
ament_export_libraries(
  damiao_ros2_control
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  ros2_control
  pluginlib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
