cmake_minimum_required(VERSION 3.8)
project(weld_visual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)

# MoveIt2 相关依赖
find_package(moveit_visual_tools REQUIRED)
# find_package(moveit_core REQUIRED)
# find_package(moveit_ros_planning REQUIRED)
# find_package(moveit_ros_planning_interface REQUIRED)
# find_package(moveit_ros_perception)

# 其他依赖
find_package(Eigen3 REQUIRED)
# find_package(pluginlib REQUIRED)
# find_package(geometric_shapes REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_eigen REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(interactive_markers REQUIRED)
# find_package(rosbag2_cpp REQUIRED)
# find_package(PCL REQUIRED COMPONENTS common io)
# find_package(Boost REQUIRED COMPONENTS system filesystem date_time thread)


# 包含路径
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    # ${Boost_INCLUDE_DIRS}
    # ${PCL_INCLUDE_DIRS}
)

# 可执行文件
add_executable(weld_visual_node src/weld_visual.cpp)

ament_target_dependencies(weld_visual_node
    rclcpp
    moveit_visual_tools
    Eigen3
    # std_msgs
    # moveit_core
    # moveit_ros_planning
    # moveit_ros_planning_interface
    # moveit_ros_perception
    # pluginlib
    # geometric_shapes
    # PCL
    # tf2_ros
    # tf2_eigen
    # tf2_geometry_msgs
    # interactive_markers
    # rosbag2_cpp
)

# 安装目标与资源文件
install(TARGETS weld_visual_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()