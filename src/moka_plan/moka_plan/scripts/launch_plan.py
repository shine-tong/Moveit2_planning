#!/usr/bin/env python3
import os
import time
import rclpy
import multiprocessing
import numpy as np
import actionlib_msgs.msg._goal_status_array
import command
import welding_sequence

from moveit_msgs.msg import PlanningScene
from rclpy.exceptions import ROSInterruptException
from std_srvs.srv import Empty
from rclpy.node import Node
from moka_utils.redis_param import RedisParam as rdsp
from welding_pose import GetQuaternion


class LaunchPlan(Node):
    def __init__(self):
        super().__init__('launch_plan_node')
        self.get_logger().info('LaunchPlan node started.')
        
        # 启动MoveIt2仿真环境和Rviz可视化界面
        command.launch_rviz()
        
        # 初始化相关参数
        self.waited_once = False
        self.tim_list = []
        self.num = 0
        
        # 初始化redis全局服务器参数
        rdsp.set_param('sign_control', 0)
        rdsp.set_param('sign_pointcloud', 0)
        rdsp.set_param('sign_traj_accepted', 0)
        rdsp.set_param('sign_error', 0)
        
        rdsp.set_param('yaw', np.pi/3)  # 内收角，即前进方向与竖直方向的夹角
        rdsp.set_param('yaw_rate', 100) # 偏航角过渡频率
        rdsp.set_param('pitch_of_Horizontalweld', np.pi*5/18)   # 平缝与底面夹角
        rdsp.set_param('pitch_of_Verticalweld', np.pi/5)        # 竖缝与底面夹角
        rdsp.set_param('culling_radius', 12)    # 焊缝剔除半径
        rdsp.set_param('thresholds', 10)        # 起点和终点扩大剔除半径的范围
        rdsp.set_param('workspace_hight', 0.103)    # 工件高度
        rdsp.set_param('folder_path', '/home/tong/colcon_ws/data/6_Welds')
        
        # 控制台显示当前参数值
        self.get_logger().info('当前参数值：')
        self.get_logger().info("sign_pointcloud = %s", rdsp.get_param("sign_pointcloud"))
        self.get_logger().info("culling_radius = %s", rdsp.get_param("culling_radius"))
        self.get_logger().info("thresholds = %s", rdsp.get_param("thresholds"))
        self.get_logger().info("yaw = %s",rdsp.get_param("yaw")/np.pi*180)
        self.get_logger().info("yaw_rate = %s",rdsp.get_param("yaw_rate"))
        self.get_logger().info("pitch_of_Horizontalweld = %s",rdsp.get_param("pitch_of_Horizontalweld")/np.pi*180)
        self.get_logger().info("pitch_of_Verticalweld = %s",rdsp.get_param("pitch_of_Verticalweld")/np.pi*180)
        self.get_logger().info("workpiece_hight = %s", rdsp.get_param("workpiece_hight"))
        
        

    def wait_for_topic(self, message_type, node_name, topic_name):
        """
        等待指定话题的消息,如果超时则返回None
        :param topic_name: 话题名称
        :param message_type: 消息类型
        :return: message或None
        """
        ok, message = rclpy.wait_for_message(message_type, node_name, topic_name, time_to_wait = 5.0)
        try:
            if ok:
                return message
            else:
                self.get_logger().info('超时内未收到消息！')
                return None
        except ROSInterruptException as e:
            self.get_logger().info(f'等待话题 {topic_name} 时程序被中断: {e}.')
            return None
        
    def clear_octomap(self):
        """
        清除octomap
        """
        # 创建服务
        clear_octomap_client = self.create_client(Empty, '/clear_octomap')
        
        # 等待服务可用
        if not clear_octomap_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('clear_octomap 服务不可用, 跳过操作.')
            return False
        
        req = Empty.Request()
        try:
            future = clear_octomap_client.call_async(req)   # 使用异步调用请求服务
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info('Octomap has been cleared.')
                return True
            else:
                self.get_logger().info('Service returned None result.')
                return False
        except Exception as e:
            self.get_logger().info(f'Unexpected error while calling /clear_octomap: {e}.')
            return False
        
    def check_file(self):
        """
        检查特定文件是否存在
        :return: True如果文件存在,否则返回False
        """
        file_path = rdsp.get_param('folder_path')
        file_path_points = os.path.join(file_path, 'points.txt')
        if os.path.exists(file_path_points):
            result = True
        else:
            self.get_logger().info('焊缝数据不存在，请选择焊缝...')
            result = False
            
        return result
    
    def wait_rviz(self):
        """
        等待rviz启动
        """
        self.get_logger().info("正在等待rviz启动...")
        self.wait_for_topic(actionlib_msgs.msg.GoalStatusArray, )    
    
    def launch_plan(self):
        """
        启动规划
        """
        while rclpy.ok():
            sign_control = str(rdsp.get_param('sign_control'))
            if sign_control == '0':
                if not self.waited_once:
                    self.get_logger().info('正在等待点云数据准备完成...')
                    self.waited_once = True
            elif sign_control == '1':
                flag = self.check_file()
                
                if flag:
                    tim1 = time.time()
                    rdsp.set_param('sign_traj_accepted', 0)
                    
                    self.clear_octomap()
                    
                    # 点云计算与发布
                    process = multiprocessing.Process(target=command.launch_publish_pointcloud_background)
                    process.start()
                    
                    # 计算焊接顺序和姿态
                    welding_sequence.run()
                    GetQuaternion.run()
                    
                    # 等待场景加载
                    self.get_logger().info('正在等待场景地图加载完毕...')
                    self.wait_for_topic(PlanningScene, self, '/move_group/monitored_planning_scene')
                    self.get_logger().info('场景地图已加载完毕,点云已发布.')
                    
                    rdsp.set_param('sign_pointcloud', 1)
                    
                    self.get_logger().info('正在运行规划程序...')
                    process = multiprocessing.Process(target=command.launch_moveit_control_server_background)
                    process.start()
                
                    while rdsp.get_param('sign_control'):
                        pass
                    
                    tim2 = time.time() - tim1
                    print(f'本次规划用时：{round(tim2,2)}s')
                    print("-------------------------------------")
                    
                    self.waited_once = False
                else:
                    rdsp.set_param('sign_control', 0)
                    self.waited_once = False
            elif sign_control == '2':
                flag = self.check_file()
                if flag:
                    tim3 = time.time()
                    rdsp.set_param('sign_traj_accepted', 0)
                    
                    # 计算焊接顺序和姿态
                    welding_sequence.run()
                    GetQuaternion.run()
                    
                    self.get_logger().info('正在运行规划程序...')
                    process = multiprocessing.Process(target=command.launch_moveit_control_server_background)
                    process.start()
                    
                    while rdsp.get_param('sign_control'):
                        pass
                    
                    tim4 = time.time() - tim3
                    self.tim_list.append(round(tim4, 2))
                    self.num += 1
                    print('-------------------------------------')
                    print(f'第{self.num}次重规划,用时：{round(tim4,2)}s')
                    print(self.tim_list)
                    print("-------------------------------------")
                    
                    self.waited_once = False
                else:
                    rdsp.set_param('sign_control', 0)
                    self.waited_once = False
            else:
                self.get_logger().info('正在关闭规划程序...')  
                command.close_rviz()
                exit(0)
                
def run():
    rclpy.init(args=None)
    node = LaunchPlan()
    
    try:
        node.launch_plan()
    except KeyboardInterrupt:
        node.get_logger().info('正在关闭规划节点...')
    except Exception as e:
        node.get_logger().info(f'规划节点异常退出：{e}!')
    finally:
        # 确保资源释放
        node.destroy_node()
        rclpy.shutdown()
        print('LaunchPlan 节点已安全退出.')
        
if __name__ == "__mian__":
    run()