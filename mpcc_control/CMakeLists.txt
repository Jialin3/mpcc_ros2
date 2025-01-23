cmake_minimum_required(VERSION 3.5)
project(mpcc_control)

# 默认使用C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)  

# 添加头文件目录
include_directories(include)

# 添加可执行文件
add_executable(mpc_controller src/mpc_controller.cpp src/track.cpp)

# 添加依赖
ament_target_dependencies(mpc_controller
  rclcpp
  std_msgs
  geometry_msgs
  Eigen3  # 如果使用了Eigen库
)

# 安装可执行文件
install(TARGETS
  mpc_controller
  DESTINATION lib/${PROJECT_NAME})

# 导出依赖
ament_package()