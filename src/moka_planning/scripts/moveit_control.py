#!/usr/bin/env python3
import rclpy
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from rclpy.logging import get_logger
from moveit.planning import MultiPipelinePlanRequestParameters
from moveit_configs_utils import MoveItConfigsBuilder
import yaml
from geometry_msgs.msg import PoseStamped
from moveit.core.kinematic_constraints import construct_joint_constraint, construct_link_constraint

# MoveitPy规划类
class MoveIt2Py:
    # 初始化 ros 和 moveit
    def __init__(self):
        rclpy.init()
        self.logger = rclpy.logging.get_logger("moveit_py.pose_goal")
        
        # ompl配置文件路径
        self.path = __file__.split('scripts/moveit_control.py')[0]+'config/ompl_planning.yaml'
        print(f"ompl config path: {self.path}")
        
        # MoveIt参数配置路径
        # self.moveit_config = (
        #         MoveItConfigsBuilder(
        #                 robot_name="mr12urdf20240605",
        #                 package_name="mr12_moveit_config"
        #         )
        #         .moveit_cpp(self.path)
        #         .to_moveit_configs()
        # )
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

        # instantiate MoveItPy instance and get planning component
        self.moka = MoveItPy(node_name="moveit_py_moka", config_dict=self.params)
        self.arm = self.moka.get_planning_component("manipulator")
        self.logger.info("MoveItPy instance created")
    
    # 规划和执行辅助函数    
    def plan_and_execute(self,
        robot,
        planning_component,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        ):
        """A helper function to plan and execute a motion."""
        # plan to goal
        logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
                self.plan_result = planning_component.plan(
                        multi_plan_parameters=multi_plan_parameters
                )
        elif single_plan_parameters is not None:
                self.plan_result = planning_component.plan(
                        single_plan_parameters=single_plan_parameters
                )
        else:
                self.plan_result = planning_component.plan()

        # execute the plan
        if self.plan_result:
                logger.info("Executing plan")
                self.robot_trajectory = self.plan_result.trajectory
                robot.execute(self.robot_trajectory, controllers=[])
        else:
                logger.error("Planning failed")
                
    def go_home(self):
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name = "home")
        
        self.plan_and_execute(self.moka, self.arm, self.logger)
  
    # 运动到随机位姿
    def move_to_random(self):
        # 使用当前机器人模型实例化 RobotState 实例
        self.robot_model = self.moka.get_robot_model()
        self.robot_state = RobotState(self.robot_model)
        
        # 生成随机位姿
        self.robot_state.set_to_random_positions()
        
        # 设置起始状态为当前状态
        self.arm.set_start_state_to_current_state()

        # 设置目标状态为随机位姿
        self.logger.info(f"Moving to random pose: {self.robot_state}")
        self.arm.set_goal_state(robot_state = self.robot_state)
        
        # 规划和执行运动
        self.plan_and_execute(self.moka, self.arm, self.logger)

    def move_pose(self, pose):
        self.arm.set_start_state_to_current_state()
        
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "base_link"
        self.target_pose.pose.position.x = 0.28
        self.target_pose.pose.position.y = -0.2
        self.target_pose.pose.position.z = 0.5
        self.target_pose.pose.orientation.w = 1.0
        
        self.arm.set_goal_state(pose_stamped_msg = self.target_pose, pose_link = "link_end")
        
        self.plan_and_execute(self.moka, self.arm, self.logger)
        
    def move_joint(self, joint_values):
        self.arm.set_start_state_to_current_state()
        
        # 设置关节角度值
        self.joint_values = {
                "joint1": -1.0,
                "joint2": 0.7,
                "joint3": 0.7,
                "joint4": -1.5,
                "joint5": -0.7,
                "joint6": 2.0,
        } 
        
        self.robot_model = self.moka.get_robot_model()
        self.robot_state = RobotState(self.robot_model)
        self.robot_state.joint_positions = self.joint_values
        
        # 创建关节组约束
        self.joint_constraint = construct_joint_constraint(
                robot_state = self.robot_state,
                joint_model_group = self.moka.get_robot_model().get_joint_model_group("manipulator")
        )
        
        # 设置关节约束
        self.arm.set_goal_state(motion_plan_constraints = [self.joint_constraint])  
        
        self.plan_and_execute(self.moka, self.arm, self.logger)      
        
if __name__ == "__main__":
    moveit2py = MoveIt2Py()
    moveit2py.go_home()
#     moveit2py.move_to_random()
#     moveit2py.move_to_pose(pose=None)
#     moveit2py.move_joint(joint_values=None)


