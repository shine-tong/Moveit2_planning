#!/usr/bin/env python3
import os
import subprocess
import datetime
import signal

script_directory = os.path.join(os.path.abspath(__file__))  # 当前脚本所在的目录

def launch_rviz():
    """
    启动MoveIt2仿真环境和Rviz可视化界面
    """
    cmd = "ros2 launch mr12_moveit_config demo.launch.py"
    
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', cmd])
    
def launch_rviz_background():
    """
    在后台启动MoveIt2仿真环境和Rviz可视化界面，并将日志信息保存在项目logs文件夹下
    """
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    
    with open(f'/home/tong/colcon_ws/moveit_logs/moveit_log_{timestamp}.txt', 'w') as log_file:
        process = subprocess.Popen(
            ['ros2', 'launch', 'mr12_moveit_config', 'demo.launch.py'],
            stdout=log_file, stderr=subprocess.STDOUT
        )
    
    return process

def close_rviz():
    """
    关闭Rviz
    """
    process_name = "/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch mr12_moveit_config demo.launch.py"
    
    try:
        # 使用pgrep查找终端进程
        pgrep_cmd = "pgrep -f '{}'".format(process_name)
        pids = subprocess.check_output(pgrep_cmd, shell=True).strip().split()
        
        # 如果找到了匹配的进程ID，发送SIGTERM信号
        for pid in pids:
            try:
                os.kill(int(pid), signal.SIGTERM)
            except ProcessLookupError:
                print('Process does not exist!')
            except Exception as e:
                print('Error killing process{pid}: {e}!')
    except subprocess.CalledProcessError:
        print(f"No processes found for terminal name '{process_name}'.")
        
def load_visual():
    pass

def launch_tf2_web():
    pass

def launch_rosbridge():
    pass

def launch_publish_pointcloud_background():
    """
    后台静默启动点云发布节点
    """
    file_name = "pointcloud_publisher.py"
    absolute_path = os.path.join(script_directory, file_name)
    cmd = ['python3', absolute_path]
    
    subprocess.call(cmd)
    
def launch_moveit_control_server_background():
    """
    后台静默启动MoveIt2控制节点
    """
    file_name = "moveit_control.py"
    absolute_path = os.path.join(script_directory, file_name)
    cmd = ['python3', absolute_path]
    
    subprocess.call(cmd)