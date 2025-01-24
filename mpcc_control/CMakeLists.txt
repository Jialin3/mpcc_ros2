cmake_minimum_required(VERSION 3.5)
project(mpcc_control)

# 默认使用C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 查找依赖
# 查找ROS 2的依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

# 添加头文件目录
include_directories(include)

# 添加可执行文件
add_executable(mpc_controller_node 
  src/mpc_controller.cpp 
  src/ArcLengthSpline.cpp 
  src/CubicSpline.cpp 
  src/mpcc_solver.cpp 
  src/track.cpp
)

# 链接库
target_link_libraries(mpc_controller_node
  ${rclcpp_LIBRARIES}
  ${ackermann_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  Eigen3::Eigen
)


# 安装目标
install(TARGETS
  mpc_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  ackermann_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  eigen3_cmake_module
)

# 安装配置
ament_package()