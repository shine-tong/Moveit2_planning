#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moka_plan',      # 包名
            executable='launch_plan_node',   # setup.py 中 console_scripts 注册的名字
            name='launch_plan_node',         # 节点名
            output='screen',
            emulate_tty=True,         # 让彩色日志正常显示（可选）
        )
    ])