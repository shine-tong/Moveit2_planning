#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 启动脚本的绝对路径
    script_path = '/home/tong/colcon_ws/src/moka_planning/scripts/launch_plan.py'
    
    return LaunchDescription([
        Node(
            package='moka_planning',
            executable=script_path,
            name='launch_plan_node',
            output='screen',
            emulate_tty=True
        )
    ])