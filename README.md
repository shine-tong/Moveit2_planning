# MR12机械臂运动规划与控制系统

## 项目概述

本项目是一个基于ROS 2的MR12六自由度机械臂运动规划与控制系统，集成了MoveIt 2运动规划框架，提供完整的机器人建模、运动规划、轨迹控制和点云处理功能。

## 项目结构

```
src/
├── moka_interface/          # 自定义消息接口包
│   ├── msg/                 # 消息定义文件
│   │   ├── JointTrajectoryEx.msg
│   │   └── JointTrajectoryPointEx.msg
│   └── package.xml
├── moka_planning/           # 运动规划功能包
│   ├── config/              # 配置文件
│   │   └── ompl_planning.yaml
│   ├── scripts/             # Python脚本
│   │   ├── moveit_control.py
│   │   └── pointcloud_publisher.py
│   ├── utils/             # 工具目录
│   │   ├── __init__.py
│   │   └── redis_param.py
│   └── package.xml
├── mr12_moveit_config/      # MoveIt配置包
│   ├── config/              # MoveIt配置文件
│   │   ├── chomp_planning.yaml
│   │   ├── initial_positions.yaml
│   │   ├── joint_limits.yaml
│   │   ├── kinematics.yaml
│   │   ├── moveit.rviz
│   │   ├── moveit_controllers.yaml
│   │   ├── mr12urdf20240605.ros2_control.xacro
│   │   ├── mr12urdf20240605.srdf
│   │   ├── mr12urdf20240605.urdf.xacro
│   │   ├── ompl_planning.yaml
│   │   ├── pilz_cartesian_limits.yaml
│   │   ├── ros2_controllers.yaml
│   │   └── sensors_3d.yaml
│   ├── launch/              # 启动文件
│   │   ├── demo.launch.py
│   │   ├── move_group.launch.py
│   │   ├── moveit_rviz.launch.py
│   │   ├── rsp.launch.py
│   │   ├── setup_assistant.launch.py
│   │   ├── spawn_controllers.launch.py
│   │   ├── static_virtual_joint_tfs.launch.py
│   │   └── warehouse_db.launch.py
│   └── package.xml
└── mr12urdf20240605/        # 机器人描述包
    ├── config/              # 配置文件
    ├── launch/              # 启动文件
    ├── meshes/              # 3D网格文件
    │   ├── Link1.STL
    │   ├── Link2.STL
    │   ├── Link3.STL
    │   ├── Link4.STL
    │   ├── Link5.STL
    │   ├── Link6.STL
    │   └── base_link.STL
    ├── rviz/                # RViz配置文件
    └── urdf/                # URDF机器人描述文件
        ├── mr12urdf20240605.csv
        └── mr12urdf20240605.urdf
```

## 主要功能

### 1. 机器人建模与描述
- **URDF模型**: 完整的MR12六自由度机械臂URDF描述文件
- **3D网格**: 高精度STL格式的机器人连杆网格文件
- **运动学参数**: 详细的关节限制、惯性参数和碰撞检测配置
- **语义描述**: SRDF文件定义机器人组、预设姿态和碰撞禁用规则

### 2. 运动规划与控制
- **MoveIt 2集成**: 基于MoveIt 2框架的运动规划系统
- **多种规划算法**: 支持OMPL、CHOMP、Pilz等多种路径规划算法
- **轨迹执行**: 完整的轨迹生成和执行功能
- **关节空间规划**: 支持关节角度目标的运动规划
- **笛卡尔空间规划**: 支持末端执行器位姿目标的运动规划
- **随机运动**: 生成随机有效配置的运动功能

### 3. 点云处理与感知
- **点云发布**: 实时点云数据发布功能
- **点云降采样**: 基于体素网格的点云降采样处理
- **焊缝检测**: 专门的焊缝起点终点检测和向量计算
- **3D感知**: 集成Open3D库进行高级点云处理

### 4. 自定义消息接口
- **扩展轨迹消息**: 增强的关节轨迹消息类型
- **轨迹点扩展**: 包含类型和序列信息的轨迹点消息
- **灵活数据结构**: 支持位置、速度、加速度和力矩数据

## 技术栈

### 核心框架
- **ROS 2 (Robot Operating System 2)**: 机器人操作系统框架
- **MoveIt 2**: 运动规划框架
- **Ament CMake**: ROS 2构建系统

### 编程语言
- **Python 3**: 主要编程语言
- **C++**: 部分底层功能实现
- **XML/YAML**: 配置文件格式

### 关键依赖库

#### 运动规划相关
- `moveit_ros_move_group`: MoveIt核心规划组件
- `moveit_kinematics`: 运动学求解器
- `moveit_planners`: 路径规划算法集合
- `moveit_simple_controller_manager`: 控制器管理
- `moveit_configs_utils`: MoveIt配置工具

#### 机器人控制
- `controller_manager`: ROS 2控制器管理器
- `joint_state_publisher`: 关节状态发布器
- `robot_state_publisher`: 机器人状态发布器
- `tf2_ros`: 坐标变换库

#### 可视化与仿真
- `rviz2`: 3D可视化工具
- `rviz_common`: RViz通用组件
- `rviz_default_plugins`: RViz默认插件

#### 点云处理
- `Open3D`: 3D数据处理库
- `NumPy`: 数值计算库
- `SciPy`: 科学计算库
- `sensor_msgs`: 传感器消息类型

#### 其他工具
- `xacro`: XML宏处理器
- `warehouse_ros_mongo`: 运动规划数据库存储
- `rclpy`: ROS 2 Python客户端库
- `rclcpp`: ROS 2 C++客户端库
- `redis`: redis通信

## 安装与使用

### 环境要求
- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Python 3.12+
- MoveIt2 Jazzy

### 安装步骤

1. **安装ROS2 Jazzy**
```bash
# 按照官方文档安装ROS 2 Jazzy
sudo apt update
sudo apt install ros-Jazzy-desktop
```

2. **安装MoveIt 2**
```bash
sudo apt install ros-Jazzy-moveit*
```

3. **安装依赖包**
```bash
sudo apt install ros-Jazzy-joint-state-publisher-gui \
                 ros-Jazzy-xacro \
                 ros-Jazzy-controller-manager \
                 ros-Jazzy-warehouse-ros-mongo
```

4. **安装Python依赖**
```bash
pip3 install open3d redis
```

5. **配置redis**
```bash
sudo apt install redis-server
sudo systemctl enable redis-server
sudo systemctl start redis-server
```

6. **构建工作空间**
```bash
cd ~/your_workspace
colcon build
source install/setup.bash
```

### 使用方法

#### 1. 启动MoveIt规划环境
```bash
ros2 launch mr12_moveit_config demo.launch.py
```

#### 2. 启动运动控制节点
```bash
ros2 run moka_planning moveit_control.py
```

#### 3. 启动点云发布器
```bash
ros2 run moka_planning pointcloud_publisher.py
```

#### 4. 可视化机器人模型
```bash
ros2 launch mr12urdf20240605 display.launch.py
```

## 机器人规格

### MR12机械臂参数
- **自由度**: 6DOF
- **关节类型**: 全旋转关节
- **工作空间**: 基于关节限制的球形工作空间
- **关节限制**:
  - Joint 1: -2.878 ~ 2.878 rad
  - Joint 2-6: 具体限制见配置文件
- **最大速度**: 3.541 rad/s (Joint 1)
- **控制精度**: 高精度位置控制

### 预设姿态
- **Home位置**: 所有关节角度为0的初始姿态
- **Test位置**: Joint1=0.2rad的测试姿态

## 开发与扩展

### 添加新的运动规划功能
1. 在`moka_planning/scripts/moveit_control.py`中添加新的运动函数
2. 配置相应的规划参数
3. 测试和验证功能

### 自定义消息类型
1. 在`moka_interface/msg/`目录下添加新的.msg文件
2. 更新`package.xml`和`CMakeLists.txt`
3. 重新构建工作空间

### 修改机器人模型
1. 更新URDF文件中的几何和物理参数
2. 重新生成MoveIt配置
3. 更新碰撞检测和运动学配置

## 许可证

- `moka_interface`: Apache-2.0
- `moka_planning`: Apache-2.0
- `mr12_moveit_config`: BSD
- `mr12urdf20240605`: 待定义

## 贡献者

- **维护者**: tong (tongry@123.com)
- **作者**: try (try@123.com)
- **原始开发**: ragesh (ragesh.ramachandran@ipa.fraunhofer.de)

## 支持与反馈

如有问题或建议，请通过以下方式联系：
- 邮箱: tongry@123.com

---

*本项目基于ROS 2和MoveIt 2框架开发，旨在提供完整的机械臂运动规划与控制解决方案。*