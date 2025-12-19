cmake_minimum_required(VERSION 3.8)
project(ly_mags)

# 默认使用C++17
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Input.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 添加可执行文件
add_executable(transform_node src/transform_node.cpp)

ament_target_dependencies(transform_node 
  rclcpp 
  std_msgs
)

# 关键添加ROS接口依赖
rosidl_target_interfaces(transform_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
# 安装目标
install(TARGETS
  transform_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install message files
install(
  DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

ament_package()
