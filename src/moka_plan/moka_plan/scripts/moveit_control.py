#!/usr/bin/env python3
import os
import rclpy
import yaml
import json
import tf_transformations
import numpy as np
import check_joint_limits

from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit.planning import MoveItPy, PlanningComponent, PlanningSceneMonitor
from moveit.core.planning_interface import MotionPlanResponse   # type: ignore
from moveit.core.robot_state import RobotState  # type: ignore
from moveit.core.robot_model import RobotModel, JointModelGroup  # type: ignore
from moveit.core.robot_trajectory import RobotTrajectory  # type: ignore
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_msgs.msg._move_it_error_codes import MoveItErrorCodes
from moveit_msgs.msg._robot_trajectory import RobotTrajectory as RobotTrajectoryMsg
from moveit_msgs.msg import Constraints, PositionConstraint
from geometry_msgs.msg import PoseStamped, Vector3, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive
from builtin_interfaces.msg import Duration
from tf_transformations import quaternion_from_euler
from copy import deepcopy# type: ignore
from moka_utils.redis_param import RedisParam as rdsp
from moka_interface.msg import JointTrajectoryEx, JointTrajectoryPointEx
from py_msg import PyMsgs


# --------------------------- MoveItPy ---------------------------------------- #
class MoveIt2Py(Node):
    def __init__(self):
        """
        ROS2 和 MoveItPy 以及相关规划参数初始化
        """
        super().__init__("moveit_control")
        self.get_logger().info("moveit_py.pose_goal")
        
        # 初始化相关参数
        self.pi = np.pi
        self.home_pose = []
        self.weld_num = 0
        self.weld_fail = []
        self.h = rdsp.get_param('workpiece_hight')

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
        self.planning_scene_monitor: PlanningSceneMonitor = self.robot.get_planning_scene_monitor()
        self.get_logger().info("MoveItPy instance created")
        self.end_effector_link = 'link_end'
        
        # 确保机器人初始姿态为 home 姿态
        self.go_home()
    
    def get_eef_link(self):
        """
        获取当前规划组的末端执行器名称
        :return eef_link: 末端执行器名称
        """
        robot_model: RobotModel = self.robot.get_robot_model()
        group: JointModelGroup = robot_model.get_joint_model_group("manipulator")
        eef_link = group.eef_name
        
        return eef_link
         
    def clear_path_constraints(self):
        """
        清除所有路径约束
        """
        self.move_action_goal = MoveGroup.Goal()
        self.move_action_goal.request.path_constraints = Constraints()
     
    def plan_and_execute(self):
        """
        规划和执行辅助函数
        """
        plan_result: MotionPlanResponse = self.arm.plan()
        if plan_result:
            self.get_logger().info("Executing plan")
            robot_trajectory = plan_result.trajectory
            self.robot.execute(robot_trajectory, controllers=[])    # 注意执行是 robot
        else:
            self.get_logger().error("Planning failed")
    
    def execute_trajectory(self, robot_traj: RobotTrajectory):
        """执行轨迹，并默认使用 manipulator_controller"""
        return self.robot.execute(robot_traj, controllers=[])  
            
    def get_now_pose(self):
        """
        获取末端执行器的当前位姿
        :return: 当前位姿
        """ 
        current_state: RobotState = self.arm.get_start_state()
        current_pose: Pose = current_state.get_pose(self.end_effector_link)
        pose = []
        pose.append(current_pose.position.x)
        pose.append(current_pose.position.y)
        pose.append(current_pose.position.z)
        
        pose.append(current_pose.orientation.x)
        pose.append(current_pose.orientation.y)
        pose.append(current_pose.orientation.z)
        pose.append(current_pose.orientation.w)
        
        return pose
        
    def get_current_state(self):
        """
        获取机械臂当前状态
        :return: 六个轴的关节位置列表
        """
        with self.planning_scene_monitor.read_only() as sence:
            robot_state: RobotState = sence.current_state   # RobotState()
            
        return robot_state
                             
    def go_home(self):
        """
        控制机器人回到 home 姿态
        """
        self.get_logger().info("go_home start")
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name = "home")
        self.plan_and_execute()
        self.get_logger().info("go_home end")
        
    def go_home_justplan(self, trajectory, trajectory_with_type):
        """
        仅规划机器人回到home点的轨迹
        :param trajectory: 轨迹列表
        :param trajectory_with_type: 带有轨迹类型的轨迹列表
        :return: 包含规划信息的元组
        """
        if trajectory:
            state: RobotState = self.arm.get_start_state()
            state.joint_positions = trajectory[-1].joint_trajectory.points[-1].positions
            self.arm.set_start_state(state)
        else:
            self.arm.set_start_state_to_current_state()
        
        self.arm.set_goal_state(configuration_name = "home")
        plan_result: MotionPlanResponse = self.arm.plan()
        traj: RobotTrajectory = plan_result.trajectory
        trajj: RobotTrajectoryMsg = traj.get_robot_trajectory_msg()
        traj_with_type = mark_the_traj(trajj, "go-home", welding_sequence)
        trajectory.append(trajj)
        trajectory_with_type.append(traj_with_type)
        
        return trajectory, trajectory_with_type

    def move_to_random(self):
        """
        控制机器人运动到随机位姿
        """
        robot_model = self.robot.get_robot_model()
        robot_state = RobotState(robot_model)
        
        # 生成随机位姿
        robot_state.set_to_random_positions()
        
        # 设置目标状态为随机位姿
        self.get_logger().info(f"Moving to random pose: {robot_state}")
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(robot_state = robot_state)
        self.plan_and_execute()

    def move_pose(self, pose): 
        """
        控制机器人到达指定姿态
        :param pose: [x, y, z, roll, pitch, yaw]
        """
        q = quaternion_from_euler(pose[3], pose[4], pose[5])    # 将欧拉角转换为四元数
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = pose[0]
        target_pose.pose.position.y = pose[1]
        target_pose.pose.position.z = pose[2]
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]
        
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg = target_pose, pose_link = self.end_effector_link)
        self.plan_and_execute()
                
    def move_joint(self, joints):
        """
        控制机器人到达制定关节位置
        :param joint_values: 机器人六轴关节值列表
        """
        if joints is None or len(joints) != 6:
            self.get_logger().info("请检查 joints 是否正确！")
            return
        
        robot_state = RobotState(self.robot.get_robot_model())
        # 设置关节角度值
        joint_values = {
                "joint1": joints[0],
                "joint2": joints[1],
                "joint3": joints[2],
                "joint4": joints[3],
                "joint5": joints[4],
                "joint6": joints[5],
        } 
        
        robot_state.joint_positions = joint_values
        
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(robot_state = robot_state)
        self.plan_and_execute()     
    
    def move_p_path_constraints(self, start_point, end_point, r):
        """
        创建move_p路径约束
        :param start_point: 起点坐标
        :param end_point: 终点坐标
        :param r: 圆柱半径
        :return: 路径约束
        """
        #计算起点指向终点的向量
        vector = np.array([end_point[0]- start_point[0], end_point[1]- start_point[1], end_point[2]- start_point[2]])
        height = np.linalg.norm(vector) + 0.16  #高度延长16cm
        radius = r
        
        # 创建圆柱路径约束
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'base_link'
        position_constraint.link_name = self.end_effector_link
        position_constraint.target_point_offset = Vector3(0, 0, 0)
        position_constraint.weight = 1.0
        
        # 构建 shape_msgs/SolidPrimitive 消息
        bounding_volume = SolidPrimitive()
        bounding_volume.type = SolidPrimitive.CYLINDER
        bounding_volume.dimensions = [height, radius]
        position_constraint.constraint_region.primitives.append(bounding_volume)
        
        # 构建 geometry_msgs/Pose 消息,用于指定圆柱体在空间中的位置和姿态
        pose = Pose()
        pose.position.x = start_point[0] + vector[0] / 2
        pose.position.y = start_point[1] + vector[1] / 2
        pose.position.z = start_point[2] + vector[2] / 2
        
        # 计算圆柱体的姿态
        z_axis = np.array([0, 0, 1])
        if np.linalg.norm(vector) < 1e-6:
            angle = 0.0
            axis = np.array([1, 0, 0])
            q = np.array([0, 0, 0, 1])
        else:
            axis = np.cross(z_axis, vector)
            if np.linalg.norm(axis) < 1e-6:
                axis = np.array([1, 0, 0])
            axis = axis / np.linalg.norm(axis)
            
            cos_theta = np.clip(np.dot(z_axis, vector) / np.linalg.norm(vector), -1.0, 1.0)
            angle = np.arccos(cos_theta)
            q = tf_transformations.quaternion_about_axis(angle, axis)
            
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        position_constraint.constraint_region.primitive_poses.append(pose)
        
        constraint = Constraints()
        constraint.position_constraints.append(position_constraint)
        
        return constraint
        
    def move_p(self, point, points: list, trajectory: list, trajectory_with_type: list):
        """
        转点规划
        :param point: 目标点
        :param points: 路径点列表
        :param trajectory: 轨迹列表
        :param trajectory_with_type: 带标记的轨迹列表
        :return: 包含规划信息的元组
        """
        r = 0.1
        er = 0
        attempts = 10
        
        if trajectory:
            state: RobotState = self.arm.get_start_state()  # 起点位置设置为规划组最后一个点，即上一次移动的终点
            state.joint_positions = trajectory[-1].joint_trajectory.points[-1].positions
            path_constraints = self.move_p_path_constraints(points[-1], point, r)
        else:
            self.go_home()  # 刚开始规划 起点位置设定为当前状态  按理来说是home点 
            self.home_pose = self.get_now_pose()
            state: RobotState = self.arm.get_start_state()
            path_constraints = self.move_p_path_constraints(self.home_pose, point, r)
            
        self.arm.set_path_constraints(path_constraints)
        self.arm.set_goal_state(pose_stamped_msg = point, pose_link = self.end_effector_link)
        self.arm.set_start_state(state)   # 起点位置设置为规划组最后一个点 或者当前状态（第一个点时）
        
        # 尝试规划 10 次
        for i in range(attempts):
            plan_result: MotionPlanResponse = self.arm.plan()
            error: MoveItErrorCodes = plan_result.error_code
            if error.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info("MOVP规划完成! 正在检查移动轨迹有效性...")
                traj: RobotTrajectory = plan_result.trajectory
                trajj: RobotTrajectoryMsg = traj.get_robot_trajectory_msg()
                error_c, limit_margin = check_joint_limits.check_joint_limits(trajj)
                if not error_c:
                    self.get_logger().info("本次移动OK")
                    self.get_logger().info("*******************")
                    traj_with_type = mark_the_traj(trajj, "during-p", welding_sequence)
                    trajectory_with_type.append(traj_with_type)
                    points.append(points)
                    trajectory.append(trajj)
                    break
                else:
                    self.get_logger().info("check failed! 移动轨迹无效")
                    self.get_logger().info("移动轨迹检查失败-开始第{}次重新规划".format(i+1))
                    r += 0.2
                    self.get_logger().info("R值: {}".format(r))
                    if trajectory:
                        path_constraints = self.move_p_path_constraints(points[-1], point, r)
                    else:
                        path_constraints = self.move_p_path_constraints(self.home_pose, point, r)
                    self.arm.set_path_constraints(path_constraints)
                    
                    if (i == (attempts - 1)):
                        er = 1
                        self.get_logger().info("所有移动规划尝试失败,焊缝起点不可达!")
                        break
            else:
                er = 1
                self.get_logger().info("移动规划失败，焊缝起点不可达！")
                break    
        return points, trajectory, traj_with_type, er
    
    def move_l_path_constraints(self, start_point, end_point):
        """
        创建move_l路径约束
        :param start_point: 起点坐标
        :param end_point: 终点坐标
        :return: 路径约束
        """
        #计算起点指向终点的向量
        vector = np.array([end_point[0]- start_point[0], end_point[1]- start_point[1], end_point[2]- start_point[2]])
        height = np.linalg.norm(vector) + 0.002
        radius = 0.001
        
        constraint = Constraints()
        
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "base_link"
        position_constraint.link_name = self.end_effector_link
        position_constraint.target_point_offset = Vector3(0, 0, 0)  # 不做偏移
        
        bounding_volume = SolidPrimitive()
        bounding_volume.type = SolidPrimitive.CYLINDER
        bounding_volume.dimensions = [height, radius]
        
        # 计算旋转矩阵
        z_axis = np.array([0, 0, 1])
        axis = np.cross(z_axis, vector)
        angle = np.arccos(np.dot(z_axis, vector) / np.linalg.norm(vector))
        q = tf_transformations.quaternion_about_axis(angle, axis)
        
        pose = Pose()
        pose.position.x = start_point[0] + vector[0] / 2
        pose.position.y = start_point[1] + vector[1] / 2
        pose.position.z = start_point[2] + vector[2] / 2
        
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        
        position_constraint.constraint_region.primitives.append(bounding_volume)
        position_constraint.constraint_region.primitive_poses.append(pose)
        position_constraint.weight = 1.0
        
        constraint.position_constraints.append(position_constraint)
        
        return constraint

    def move_l(self, point, points: list, trajectory: list, trajectory_with_type: list):
        """
        焊缝规划，规划当前点(焊缝起点)和焊缝终点之间的路径
        :param point: 目标点(焊缝起点)
        :param points: 路径点列表
        :param trajectory: 轨迹列表
        :param trajectory_with_type: 轨迹类型列表
        :return: 包含规划信息的元组
        """
        er = 0
        attempts = 10
        
        if trajectory:  # move_p规划成功，则move_p规划路径中的最后一个关节轨迹点为move_l的起点
            state: RobotState = self.arm.get_start_state()
            state.joint_positions = trajectory[-1].joint_trajectory.points[-1].positions
            self.arm.set_start_state(state)
        else:
            """
            第一次规划超时(Time_out),但并未返回Error_code;此时move_p并不会返回任何路径点信息
            到points列表中,此时机器人应还处在home点位置,在此将home点直接添加到points列表中,与
            焊缝终点做路径约束,并规划路径 (虽然不再报错继续执行，但该条焊缝仍规划失败)
            """
            points.append(self.home_pose)
            
        self.arm.set_goal_state(pose_stamped_msg = point, pose_link = self.end_effector_link)
        
        path_constraints = self.move_l_path_constraints(points[-1], point)
        self.arm.set_path_constraints(path_constraints)
        
        for i in range(attempts):
            plan_result: MotionPlanResponse = self.arm.plan()
            error: MoveItErrorCodes = plan_result.error_code
            if error.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info("MOVL规划完成! 正在检查焊缝轨迹有效性...")
                traj: RobotTrajectory = plan_result.trajectory
                trajj: RobotTrajectoryMsg = traj.get_robot_trajectory_msg()
                error_c, limit_margin = check_joint_limits.check_joint_limits(trajj)
                
                if not error_c:
                    self.get_logger().info("本次焊缝规划 OK")
                    traj_with_type = mark_the_traj(trajj, "during-l", welding_sequence)
                    traj_with_type.points[-len(trajj.joint_trajectory.points)].type = "start"
                    traj_with_type.points[-1].type = "end"
                    points.append(point)
                    trajectory.append(trajj)
                    trajectory_with_type.append(traj_with_type)
                    break
                else:
                    self.get_logger().info("焊缝轨迹检查失败-关节翻转-开始第{}次重新规划".format(i+1))
                    points.pop()
                    # 同样第一次move_p提示timeout, trajectory中同样不会有路径信息
                    if trajectory:
                        trajectory.pop()
                        trajectory_with_type.pop()
                        
                    self.weld_fail.append(welding_sequence[self.weld_num])
                    self.weld_num += 1
                    er = 1
                    break
        return points, trajectory, trajectory_with_type, er     
    
    def move_cartesian(self):
        pass
    
    def path_planning(self, folder_path, gohome=True):
        """
        读取文件中的焊缝数据,并调用move_p_flexiblee和move_pl函数进行路径规划
        :param folder_path: 焊缝数据文件路径
        :param gohome: 是否回到home点
        :return: 包含规划信息的元组
        """
        file_path_result = os.path.join(folder_path, 'result.txt')
        all_welds = process_welding_data(file_path_result)
        err = 0
        points, trajectory, trajectory_with_type = [],[],[]
        
        for i in range(len(all_welds)):
            self.get_logger().info("本次共读取到%d条焊缝,开始规划第%d条", len(all_welds, i+1))
            start_point = all_welds[i][0]
            end_point = all_welds[i][1]
            q1 = all_welds[i][2]
            q2 = all_welds[i][3]
            
            # 动态计算安全点：横缝±45度，竖缝xy平面法向±45度
            if i == 0:
                point_safe = compute_safe_point(start_point, end_point, q1)
                points, trajectory, trajectory_with_type, err = self.move_p(point_safe, points, trajectory, trajectory_with_type)
            else:
                continue    
            
            point_start = [start_point[0]/1000, start_point[1]/1000, start_point[2]/1000, q1[0], q1[1], q1[2], q1[3]]
            points, trajectory, trajectory_with_type, err = self.move_p(point_start, points, trajectory, trajectory_with_type)                   
            if err == 1:
                self.weld_fail.append(welding_sequence[self.weld_num])
                self.weld_num = self.weld_num + 1
                continue
            
            point_end = [end_point[0]/1000, end_point[1]/1000, end_point[2]/1000, q2[0], q2[1], q2[2], q2[3]]
            points, trajectory, trajectory_with_type, err = self.move_l(point_end, points, trajectory, trajectory_with_type)
            
            self.get_logger().info("第%d条焊缝规划完毕", i+1)
            self.get_logger().info("*******************")
        
        if gohome:
            points, trajectory, trajectory_with_type, err = self.move_p(self.home_pose, points, trajectory, trajectory_with_type)
            trajectory,trajectory_with_type = self.go_home_justplan(trajectory,trajectory_with_type)
        
        traj_merge = merge_robot_trajectories(trajectory)
        trajectory_with_type_merge = merge_trajectories_with_type(trajectory_with_type)
        self.get_logger().info("全部焊缝规划完毕!")
        
        return trajectory, traj_merge, trajectory_with_type_merge
        
    # def test(self):
    #     state = self.arm.get_start_state()
    #     print(f'state: {state}')
    #     joints = state.joint_positions
    #     print(f'joints: {joints}')
    #     pose = state.get_pose(self.end_effector_link)
    #     print(f'pose: {pose}')           

# --------------------------- MoveItPy end ---------------------------------------- #     
            
def compute_safe_point(hight, start_point, end_point, q):
    """
    计算安全点, 目前只针对横缝
    :param hight: 工件高度
    :param start_point: 起点坐标
    :param end_point: 终点坐标
    :return: 安全点坐标
    """
    dir_vec = np.array([end_point[0] - start_point[0], end_point[1] - start_point[1], end_point[2] - start_point[2]])
    seam_xy = np.array([dir_vec[0], dir_vec[1]])
    
    if np.linalg.norm(seam_xy) < 1e-6:
        offset_dir = np.array([1.0, 0.0])
    else:
        if start_point[1] < end_point[1]:
            theta = -np.deg2rad(45)
        else:
            theta = np.deg2rad(45)
        rot_mat = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        offset_dir = rot_mat @ (seam_xy / np.linalg.norm(seam_xy))
        
    offset_dist = 0.05  # 50mm
    safe_x = start_point[0]/1000 + offset_dir[0] * offset_dist
    safe_y = start_point[1]/1000 + offset_dir[1] * offset_dist
    safe_z = start_point[2]/1000 + hight
    
    safe_point = [safe_x, safe_y, safe_z, q[0], q[1], q[2], q[3]]
    return safe_point

def process_welding_data(filename):
    """
    处理焊缝数据文件，返回包含所有焊缝数据的列表
    :param filename: 焊缝数据文件路径
    :return: 包含所有焊缝数据的列表
    """
    all_welds = []
    with open(filename, 'r') as file:
        for line in file:
            parts = line.strip().split('/')
            coordinates_and_poses = [part.split(',') for part in parts[1:]]
            
            start_point = tuple(map(float, coordinates_and_poses[0][:3]))
            end_point = tuple(map(float, coordinates_and_poses[1][:3]))
            q1 = tuple(map(float, coordinates_and_poses[2][:4]))
            q2 = tuple(map(float, coordinates_and_poses[3][:4]))
            
            all_welds.append((start_point, end_point, q1, q2))
    return all_welds

def mark_the_traj(trajj: RobotTrajectoryMsg, TYPE, SEQUENCE):
    """
    标记轨迹类型
    :param traj: 原始轨迹
    :param TYPE: 轨迹类型(during-p, during-l, go-home)
    :param SEQUENCE: 轨迹序列
    :return: 标记后的轨迹
    """
    traj_with_type = JointTrajectoryEx()
    traj_with_type.header = trajj.joint_trajectory.header
    traj_with_type.joint_names = trajj.joint_trajectory.joint_names
    traj_with_type.points = [
        JointTrajectoryPointEx(
            positions = point.positions,
            velocities = point.velocities,
            accelerations = point.accelerations,
            effort = point.effort,
            type = TYPE,
            sequence = SEQUENCE
        ) for point in trajj.joint_trajectory.points
    ]
    
    return traj_with_type

def merge_robot_trajectories(self, trajectories: list):
    """
    合并所有轨迹消息
    :param trajectories: 包含所有轨迹消息的列表
    :return merged_traj: 合并后的轨迹消息
    """
    # 过滤空轨迹
    valid_trajs = [traj for traj in trajectories if traj and traj.joint_trajectory and traj.joint_trajectory.points]
    if not valid_trajs:
        self.get_logger().info("所有轨迹均为空，返回空的 RobotTrajectory")
        return RobotTrajectory(self.robot.get_robot_model())

    merged_traj = RobotTrajectoryMsg()
    merged_traj.joint_trajectory.header = valid_trajs[0].joint_trajectory.header
    merged_traj.joint_trajectory.joint_names = valid_trajs[0].joint_trajectory.joint_names

    last_time_from_start = 0.0  # 以秒为单位
    inter_traj_delay = 0.05     # 轨迹间最小延迟，保证严格递增且平滑

    for traj in valid_trajs:
        trajj: JointTrajectory = traj.joint_trajectory
        first_point_time = trajj.points[0].time_from_start.sec + trajj.points[0].time_from_start.nanosec * 1e-9

        for point in trajj.points:
            new_point: JointTrajectoryPoint = deepcopy(point)
            t_point = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            # 保持轨迹内部时间间隔
            t_point_shifted = last_time_from_start + (t_point - first_point_time)
            new_point.time_from_start.sec = int(t_point_shifted)
            new_point.time_from_start.nanosec = int((t_point_shifted - int(t_point_shifted)) * 1e9)
            merged_traj.joint_trajectory.points.append(new_point)

        # 更新 last_time_from_start 为最后一点时间 + 延迟
        last_point = trajj.points[-1]
        last_point_time = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9
        last_time_from_start += (last_point_time - first_point_time) + inter_traj_delay

    return merged_traj

def convert_to_robottraj(self, trajectories_msg):
    """
    将 RobotTrajectoryMsg 轨迹消息转换为机器人可执行的轨迹类型 RobotTrajectory
    :param trajectories_msg: 合并后的轨迹消息
    :return robot_traj: 机器人可执行的轨迹 
    """
    robot_trajectory = RobotTrajectory(self.robot.get_robot_model())
    robot_trajectory.joint_model_group_name = "manipulator"
    merged_traj = self.merge_robot_trajectories(trajectories_msg)
    # start_state: RobotState = self.arm.get_start_state()
    start_state: RobotState = self.get_current_state()
    if trajectories_msg:
        first_traj: RobotTrajectoryMsg = trajectories_msg[0]
        if first_traj.joint_trajectory.points:
            first_point = first_traj.joint_trajectory.points[0]
            # 转换成 numpy array
            positions_array = np.array(first_point.positions, dtype=np.float64)
            # 设置起始关节位置
            start_state.set_joint_group_positions("manipulator", positions_array)
    robot_traj = robot_trajectory.set_robot_trajectory_msg(start_state, merged_traj)
    
    return robot_traj

def merge_trajectories_with_type(trajectory_with_type):
    """
    合并所有带标记信息的关节轨迹点为单条轨迹，因为不需要执行故不需要转换为 RobotTrajectory
    :param trajectory_with_type: 包含所有带标记的轨迹信息列表
    :return: 合并后的带标记的关节轨迹
    """
    if not trajectory_with_type:
        return
    
    merged_trajectory_with_type = JointTrajectoryEx()
    merged_trajectory_with_type.header = trajectory_with_type[0].header
    merged_trajectory_with_type.joint_names = trajectory_with_type[0].joint_names
    
    last_time_from_start = Duration(0)  # 初始化时间累加器
    
    # 合并所有带标记的 trajectories 的 points
    for traj in trajectory_with_type:
        for point in traj.points:
            new_point = deepcopy(point)
            new_point.time_from_start += last_time_from_start   # 累加时间
            merged_trajectory_with_type.points.append(new_point)
        
        # 更新时间累加器为当前轨迹的最后一个点的时间
        if traj.points:
            last_time_from_start = traj.points[-1].time_from_start + last_time_from_start
    return merged_trajectory_with_type
    
def ros2py_msgs(msgs, moveit_obj):
    """
    将规划后的轨迹信息打包成json格式发送到redis
    :param msgs: 规划后的轨迹信息
    :param moveit_obj: 参数对象
    """
    for i in range(len(msgs.points)):
        PyMsgs.Point.xyz_list.append(msgs.points[i].positions)
        PyMsgs.Point.type.append(msgs.points[i].type)
    PyMsgs.fail = moveit_obj.weld_fail.copy()
    PyMsgs.sequence = msgs.points[0].sequence.copy()
    
    message = {
        'positions': PyMsgs.Point.xyz_list,  # 规划路径点结果
        'flags': PyMsgs.Point.type,          # 规划路径点的类型，焊接点移动点
        'weld_order': PyMsgs.sequence,       # 规划路径顺序
        'failed': PyMsgs.fial,               # 规划路径失败的焊缝
    }
    
    file_path_message = os.path.join(folder_path, 'message.txt')
    with open(file_path_message, 'w') as file:
        json.dump(message, file, indent=4)
        
    file_path_json = os.path.join(folder_path, 'message.json')
    with open(file_path_json, 'w', encoding='utf-8') as json_file:
        json.dump(message, json_file, indent=4, ensure_ascii=False)
        
    return json.dumps(message)

def ros2msgs(msgs):
    """
    将规划信息保存为txt文件到指定文件夹
    :param msgs: 规划后的轨迹信息
    """
    for i in range(len(msgs.points)):
        PyMsgs.Point.xyz_list.append(msgs.points[i].positions)
    
    file_path_msgs_result = os.path.join(folder_path, 'msgs_result.txt')
    with open(file_path_msgs_result, 'w') as file:
        for point in PyMsgs.Point.xyz_list:
            file.write(' '.join(str(value) for value in point) + "\n")
    
if __name__ == "__main__":
    rclpy.init()
    folder_path = rdsp.get_param("folder_path")
    welding_sequence = rdsp.get_param('welding_sequence')
    moveit_server = MoveIt2Py()
    
    trajectory, trajectory_merge, trajectory_with_type_merge = moveit_server.path_planning(folder_path)
    moveit_server.execute_trajectory(trajectory_merge)
    print("-------------------------------------")
    print(f"本次规划失败焊缝序号:{moveit_server.weld_fail}")
    
    message = ros2py_msgs(trajectory_with_type_merge, moveit_server)
    # rdsp.pub_plan_result(message)   # 发布路径规划结果到 redis
    rdsp.set_param('sign_control', 0)
    
    # -------------------------- test ------------------------------ #
    # moveit_server.go_home()
    # moveit_server.move_to_random()
    # moveit_server.move_to_pose(pose=None)
    # moveit_server.test()
    # moveit_server.move_joint(joint_values=None)
    # moveit_server.test()