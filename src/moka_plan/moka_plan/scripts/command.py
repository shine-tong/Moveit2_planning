#!/usr/bin/env python3
import os
import subprocess
import datetime
import signal

ws_path = os.getenv("COLCON_WS", None)
if ws_path is None:
    ws_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
    
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
    
    with open(f'{ws_path}/selflogs/moveit_log/moveit_log_{timestamp}.txt', 'w') as log_file:
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
        
def load_weld_visual_background():
    """
    后台启动规划路径可视化节点
    """
    command = ["ros2", "run", "weld_visual", "weld_visual_node"]
    subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def launch_tf2_web_background():
    """后台运行 tf2_web_republisher 节点"""
    command = ["ros2", "run", "tf2_web_republisher", "tf2_web_republisher_node"]
    subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def launch_rosbridge_background():
    """后台运行 rosbridge_server 节点"""
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    with open(f"{ws_path}/selflogs/rosbridge_log/rosbridge_log_{timestamp}.txt", "w") as logfile:
        process = subprocess.Popen(
                    ["ros2", "run", 'rosbridge_server', 'rosbridge_websocket'],
                    stdout=logfile, stderr=subprocess.STDOUT
            )
         
    return process

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