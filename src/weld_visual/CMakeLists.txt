cmake_minimum_required(VERSION 3.8)
project(weld_visual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# MoveIt2 相关依赖
find_package(moveit_visual_tools REQUIRED)

# 其他依赖
find_package(Eigen3 REQUIRED)

# 包含路径
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

# 可执行文件
add_executable(weld_visual_node src/weld_visual.cpp)

ament_target_dependencies(weld_visual_node
    rclcpp
    moveit_visual_tools
    Eigen3
)

# 安装目标与资源文件
install(TARGETS weld_visual_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()