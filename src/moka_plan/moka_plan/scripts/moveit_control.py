#!/usr/bin/env python3
import rclpy
import yaml
import tf_transformations
import numpy as np

from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningComponent
from moveit.planning import PlanRequestParameters
from moveit.core.robot_state import RobotState  # type: ignore
from moveit_configs_utils import MoveItConfigsBuilder
from geometry_msgs.msg import PoseStamped
from moveit.core.kinematic_constraints import construct_joint_constraint # type: ignore
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from moka_utils.redis_param import RedisParam as rdsp

#------------------------------ 重构内容 ---------------------------#
# 1.重构MoveItPy规划类，继承Node，不在类中初始化rclpy
# 2.重构读取moveit_config方法，使用更主流稳定的方法
# 3.重构plan_and_execute方法
# 4.重构move_joint方法，取消使用关节约束来运动到给定关节位置，同时支持传入给定关节列表参数
#-----------------------------------------------------------------#


class MoveIt2Py(Node):
    # 初始化 ros 和 moveit
    def __init__(self):
        super().__init__("moveit_control")
        self.get_logger().info("moveit_py.pose_goal")
        
        # 初始化相关参数
        self.pi = np.pi
        self.home_pose = []
        self.weld_num = 0
        self.weld_fail = []
        # self.h = rdsp.get_param('workpiece_hight')

        # ompl配置文件路径
        self.path = __file__.split('scripts/moveit_control.py')[0]+'config/ompl_planning.yaml'
        print(f"ompl config path: {self.path}")
        
        self.moveit_config = (
                MoveItConfigsBuilder(robot_name="mr12urdf20240605", package_name="mr12_moveit_config")
                .robot_description(file_path="config/mr12urdf20240605.urdf.xacro")
                .robot_description_semantic(file_path="config/mr12urdf20240605.srdf")
                .trajectory_execution(file_path="config/moveit_controllers.yaml")
                .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
                .to_moveit_configs()
                )

        # MoveIt参数转换为字典
        self.params = self.moveit_config.to_dict()
        
        with open(self.path, 'r') as f:
            self.ompl_params = yaml.safe_load(f)
            
        self.params.update(self.ompl_params)

        # 实例化 MoveItPy 并获取规划组件自动补全
        self.robot = MoveItPy(node_name="moveit_py_robot", config_dict=self.params)
        self.arm: PlanningComponent = self.robot.get_planning_component("manipulator")
        self.get_logger().info("MoveItPy instance created")
        
        # 规划相关参数设置
        self.end_effector_link = 'link_end'
    
    # 规划和执行辅助函数    
    def plan_and_execute(self):
        """A helper function to plan and execute a motion."""
        plan_result = self.arm.plan()
        if plan_result:
            self.get_logger().info("Executing plan")
            robot_trajectory = plan_result.trajectory
            self.robot.execute(robot_trajectory, controllers=[])
        else:
            self.get_logger().error("Planning failed")
                
    # 回HOME点              
    def go_home(self):
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name = "home")
        
        self.plan_and_execute()
        
    # 运动到随机位姿
    def move_to_random(self):
        # 使用当前机器人模型实例化 RobotState 实例
        self.robot_model = self.robot.get_robot_model()
        self.robot_state = RobotState(self.robot_model)
        
        # 生成随机位姿
        self.robot_state.set_to_random_positions()
        
        # 设置起始状态为当前状态
        self.arm.set_start_state_to_current_state()

        # 设置目标状态为随机位姿
        self.get_logger().info(f"Moving to random pose: {self.robot_state}")
        self.arm.set_goal_state(robot_state = self.robot_state)
        
        # 规划和执行运动
        self.plan_and_execute()

    def move_pose(self, pose): 
        # self.q = quaternion_from_euler(3.14, 0.0, 0.0)    # 将欧拉角转换为四元数
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.28
        target_pose.pose.position.y = -0.2
        target_pose.pose.position.z = 0.5
        target_pose.pose.orientation.w = 1.0
        
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg = target_pose, pose_link = "link_end")
        
        self.plan_and_execute()
                
    def move_joint(self, joint_values):
        # if joint_values is None or len(joint_values) != 6:
        #     return
        robot_state = RobotState(self.robot.get_robot_model())
        # 设置关节角度值
        joint_values = {
                "joint1": -1.0,
                "joint2": 0.7,
                "joint3": 0.7,
                "joint4": -1.5,
                "joint5": -0.7,
                "joint6": 2.0,
        } 
        
        robot_state.joint_positions = joint_values
        
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(robot_state = robot_state)
        
        self.plan_and_execute()     

        
if __name__ == "__main__":
    rclpy.init()
    moveit2py = MoveIt2Py()
#     moveit2py.go_home()
#     moveit2py.move_to_random()
#     moveit2py.move_to_pose(pose=None)
    moveit2py.move_joint(joint_values=None)