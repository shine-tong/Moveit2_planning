# /home/tong/colcon_ws/src/moka_planning/utils/execution_timer.py
import time
import atexit
import rclpy
from rclpy.node import Node

# 全局节点单例，用于日志
_timer_node = None
_node_initialized_here = False

def _get_timer_node():
    """
    获取全局 Timer Node，保证只初始化一次
    """
    global _timer_node, _node_initialized_here
    
    if _timer_node is None:
        # 防止ROS2重复初始化
        if not rclpy.ok():
            rclpy.init()
            _node_initialized_here = True
        _timer_node = Node('execution_timer_node')
        
        atexit.register(_cleanup_timer_node)    # 程序退出时自动销毁节点和 shutdown（如果是这里初始化的）
        
    return _timer_node

def _cleanup_timer_node():
    """
    清理全局 Timer Node，保证不干扰其他节点
    """
    global _timer_node, _node_initialized_here
    
    if _timer_node is not None:
        _timer_node.destroy_node()
        _timer_node = None
    if _node_initialized_here:
        rclpy.shutdown()
        _node_initialized_here = False

class ExecutionTimer:
    """
    装饰器类：用于计算函数/方法运行时间并打印 ROS2 日志
    使用方法，在函数上方添加 @ExecutionTimer.time
    """
    @staticmethod
    def time(func):
        """
        计算函数运行时间的装饰器
        :param func: 需要计算运行时间的函数
        :return: 装饰后的函数
        """
        def wrapper(*args, **kwargs):
            start_time = time.time()
            result = func(*args, **kwargs)
            end_time = time.time()
            elapsed_time = end_time - start_time
            node = _get_timer_node()
            node.get_logger().info(f"Time taken by {func.__name__}: {elapsed_time:.4f} seconds")
            return result
        return wrapper