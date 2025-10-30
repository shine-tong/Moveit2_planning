# MR12机械臂运动规划与控制系统

## 项目概述

**写在前面：项目正在开发中...**  
本项目是一个基于 ROS2 的 MR12 六自由度机械臂运动规划与控制系统，集成了 MoveIt2 运动规划框架，提供完整的机器人建模、运动规划、轨迹控制和点云处理功能。

## 项目结构

```
src/
├── moka_interface/      # 自定义消息接口包
│   ├── msg/                    # 消息定义文件
│   │   ├── JointTrajectoryEx.msg       # trajectory_msgs.msg：JointTrajectory 类型消息
│   │   └── JointTrajectoryPointEx.msg  # trajectory_msgs.msg：JointTrajectoryPoint 类型消息
│   └── package.xml
├── moka_plan/           # 运动规划功能包
│   ├── config/                 # 配置文件
│   │   └── ompl_planning.yaml
│   ├── scripts/                # Python 模块
│   │   ├── moveit_control.py       # MoveItPy 路径规划模块
│   │   ├── welding_pose.py         # 姿态计算模块
│   │   ├── welding_sequence.py     # 规划顺序计算模块
│   │   ├── command.py              # 指令模块
│   │   ├── launch_plan.py          # 启动模块
│   │   ├── check_joint_limits.py   # 检查关节翻转模块
│   │   └── pointcloud_publisher.py # 点云信息发布模块
│   ├── utils/           # 工具包
│   │   ├── __init__.py
│   │   ├── execution_timer.py      # 计算函数执行时间工具
│   │   └── redis_param.py          # Redis 全局参数服务器工具
│   └── package.xml
├── mr12_moveit_config/  # MoveIt2 配置包
│   ├── config/                 # MoveIt2 配置文件
│   │   ├── chomp_planning.yaml     # chomp 配置文件
│   │   ├── initial_positions.yaml  # 初始点配置文件
│   │   ├── joint_limits.yaml       # 机器人关节限位文件
│   │   ├── kinematics.yaml         # 逆运动学求解器配置文件
│   │   ├── moveit.rviz             # rviz 可视化配置文件
│   │   ├── moveit_controllers.yaml # moveit2 控制器配置文件
│   │   ├── mr12urdf20240605.ros2_control.xacro
│   │   ├── mr12urdf20240605.srdf   # 机器人模型配置文件
│   │   ├── mr12urdf20240605.urdf.xacro
│   │   ├── ompl_planning.yaml      # ompl 配置文件
│   │   ├── pilz_cartesian_limits.yaml
│   │   ├── ros2_controllers.yaml   # ros2 控制器配置文件
│   │   └── sensors_3d.yaml         # octomap 配置文件
│   ├── launch/           # ROS2 启动文件
│   │   ├── demo.launch.py          # 启动 MoveIt2 规划环境 
│   │   ├── move_group.launch.py
│   │   ├── moveit_rviz.launch.py
│   │   ├── rsp.launch.py
│   │   ├── setup_assistant.launch.py
│   │   ├── spawn_controllers.launch.py
│   │   ├── static_virtual_joint_tfs.launch.py
│   │   └── warehouse_db.launch.py
│   └── package.xml
└── mr12urdf20240605/     # 机器人描述包
    ├── config/              # 配置文件
    ├── launch/              # 启动文件
    ├── meshes/              # 3D 网格文件
    │   ├── Link1.STL
    │   ├── Link2.STL
    │   ├── Link3.STL
    │   ├── Link4.STL
    │   ├── Link5.STL
    │   ├── Link6.STL
    │   └── base_link.STL
    ├── rviz/                # RViz 配置文件
    └── urdf/                # URDF 机器人描述文件
        ├── mr12urdf20240605.csv
        └── mr12urdf20240605.urdf
```

## 主要功能

### 1. 机器人建模与描述
- **URDF 模型**: 完整的 MR12 六自由度机械臂 URDF 描述文件
- **3D 网格**: 高精度 STL 格式的机器人连杆网格文件
- **运动学参数**: 详细的关节限制、惯性参数和碰撞检测配置
- **语义描述**: SRDF 文件定义机器人组、预设姿态和碰撞禁用规则

### 2. 运动规划与控制
- **MoveIt2 集成**: 基于MoveIt2 框架的运动规划系统
- **规划算法**: 目前只支持 OMPL 路径规划算法
- **轨迹执行**: 完整的轨迹生成和执行功能
- **关节空间规划**: 支持关节角度目标的运动规划
- **笛卡尔空间规划**: 支持末端执行器位姿目标的运动规划
- **随机关节运动**: 支持生成随机有效配置的运动功能
- **指定配置点运动**: 支持运动到 srdf 配置点的运动功能

### 3. 点云处理与感知
- **点云发布**: 实时点云数据发布功能
- **点云降采样**: 基于体素网格的点云降采样处理
- **焊缝检测**: 专门的焊缝起点终点检测和向量计算
- **3D 感知**: 集成 Open3D 库进行高级点云处理

### 4. 自定义消息接口
- **扩展轨迹消息**: 增强的关节轨迹消息类型
- **轨迹点扩展**: 包含类型和序列信息的轨迹点消息
- **灵活数据结构**: 支持位置、速度、加速度和力矩数据

## 技术栈

### 核心框架
- **ROS2 (Robot Operating System 2)**: 机器人操作系统框架
- **MoveIt2**: 运动规划框架
- **Ament Python**: ROS2 构建系统

### 编程语言
- **Python3**: 主要编程语言
- **C++**: 部分底层功能实现
- **XML/YAML**: 配置文件格式

### 关键依赖库

#### 运动规划相关
- `moveit_ros_move_group`: MoveIt2 核心规划组件
- `moveit_kinematics`: 运动学求解器
- `moveit_planners`: 路径规划算法集合
- `moveit_simple_controller_manager`: 控制器管理
- `moveit_configs_utils`: MoveIt2 配置工具

#### 机器人控制
- `controller_manager`: ROS2 控制器管理器
- `joint_state_publisher`: 关节状态发布器
- `robot_state_publisher`: 机器人状态发布器
- `tf2_ros`: 坐标变换库
- `tf_transformations`: 三维空间坐标变换和四元数运算

#### 可视化与仿真
- `rviz2`: 3D 可视化工具
- `rviz_common`: RViz 通用组件
- `rviz_default_plugins`: RViz 默认插件

#### 点云处理
- `Open3D`: 3D 数据处理库
- `NumPy`: 数值计算库
- `SciPy`: 科学计算库
- `sensor_msgs`: 传感器消息类型

#### 其他工具
- `xacro`: XML 宏处理器
- `warehouse_ros_mongo`: 运动规划数据库存储
- `rclpy`: ROS2 Python 客户端库
- `rclcpp`: ROS2 C++ 客户端库
- `redis`: redis 通信服务

## 安装与使用

### 环境要求
- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Python 3.12+
- MoveIt2 Jazzy

### 安装步骤

1. **安装 ROS2 Jazzy**
```bash
# 按照官方文档安装ROS2 Jazzy
sudo apt install ros-Jazzy-desktop
```

2. **安装 MoveIt2**
```bash
sudo apt install ros-Jazzy-moveit*
```

3. **安装依赖包**
```bash
sudo apt install ros-Jazzy-joint-state-publisher-gui \
                 ros-Jazzy-xacro \
                 ros-Jazzy-controller-manager \
                 ros-Jazzy-warehouse-ros-mongo \
                 ros-jazzy-tf-transformations 
```

4. **安装 Python3 依赖**
```bash
pip3 install open3d redis -i https://pypi.tuna.tsinghua.edu.cn/simple
```

5. **配置 redis**
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

```bash
ros2 launch moka_plan plan.launch.py
```

## 许可证

- `moka_interface`: Apache-2.0
- `moka_plan`: Apache-2.0
- `moka_utils`: Apache-2.0
- `mr12_moveit_config`: BSD
- `mr12urdf20240605`: BSD

## 贡献者

- **维护者**: shine-tong ([shine-tong](https://github.com/shine-tong))
- **作者**: shine-tong ([shine-tong](https://github.com/shine-tong))
- **原始开发**: ragesh (ragesh.ramachandran@ipa.fraunhofer.de)

## 支持与反馈

如有问题或建议，请通过以下方式联系：
- Github: https://github.com/shine-tong

---

*本项目基于 ROS2 和 MoveIt2 框架开发.*