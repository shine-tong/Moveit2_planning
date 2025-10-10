from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("mr12urdf20240605", package_name="mr12_moveit_config").sensors_3d(file_path="config/sensors_3d.yaml").to_moveit_configs()
    return generate_demo_launch(moveit_config)

# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch.conditions import IfCondition
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from ament_index_python.packages import get_package_share_directory
# from moveit_configs_utils import MoveItConfigsBuilder
# from launch_param_builder import ParameterBuilder


# def generate_launch_description():

#     # Command-line arguments
#     rviz_config_arg = DeclareLaunchArgument(
#         "rviz_config",
#         default_value="moveit.rviz",
#         description="RViz configuration file",
#     )

#     moveit_config = (
#         MoveItConfigsBuilder("mr12")
#         .robot_description(
#             file_path="config/mr12urdf20240605.urdf.xacro"
#         )
#         .robot_description_semantic(file_path="config/mr12urdf20240605.srdf")
#         .robot_description_kinematics(file_path="config/kinematics.yaml")
#         .planning_scene_monitor(
#             publish_robot_description=True, publish_robot_description_semantic=True
#         )
#         .trajectory_execution(file_path="config/moveit_controllers.yaml")
#         .planning_pipelines(
#             pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
#         )
#         .sensors_3d(
#             file_path="config/sensors_3d.yaml"
#         )
#         .to_moveit_configs()
#     )

#     # Start the actual move_group node/action server
#     move_group_node = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[moveit_config.to_dict()],
#         arguments=["--ros-args", "--log-level", "info"],
#     )

#     # RViz
#     rviz_base = LaunchConfiguration("rviz_config")
#     rviz_config = PathJoinSubstitution(
#         [FindPackageShare("mr12_moveit_config"), "launch", rviz_base]
#     )
#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         output="log",
#         arguments=["-d", rviz_config],
#         parameters=[
#             moveit_config.robot_description,
#             moveit_config.robot_description_semantic,
#             moveit_config.planning_pipelines,
#             moveit_config.robot_description_kinematics,
#             moveit_config.joint_limits,
#         ],
#     )

#     # Publish TF
#     robot_state_publisher = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         name="robot_state_publisher",
#         output="both",
#         parameters=[moveit_config.robot_description],
#     )

#     # ros2_control using FakeSystem as hardware
#     ros2_controllers_path = os.path.join(
#         get_package_share_directory("mr12_moveit_config"),
#         "config",
#         "ros2_controllers.yaml",
#     )
#     ros2_control_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[ros2_controllers_path],
#         remappings=[
#             ("/controller_manager/robot_description", "/robot_description"),
#         ],
#         output="screen",
#     )

#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=[
#             "joint_state_broadcaster",
#             "--controller-manager",
#             "/controller_manager",
#         ],
#     )

#     return LaunchDescription(
#         [
#             rviz_config_arg,
#             rviz_node,
#             robot_state_publisher,
#             move_group_node,
#             ros2_control_node,
#             joint_state_broadcaster_spawner,
#         ]
#     )