# /home/tong/colcon_ws/src/moka_planning/utils/global_param.py
"""
全局参数共享模块 (基于 Redis)
允许不同 Python 文件 / ROS2 节点 共享参数。

使用示例:
    from moka_planning.utils.global_param import set_param, get_param

    set_param('robot_speed', 1.5)
    print(get_param('robot_speed'))
"""

import redis
import json
import threading

class rds:
    def __init__(self):
        # 默认 Redis 连接配置（可根据需要修改）
        REDIS_HOST = 'localhost'
        REDIS_PORT = 6379
        REDIS_DB = 0

        # 初始化连接池
        _pool = redis.ConnectionPool(
            host=REDIS_HOST,
            port=REDIS_PORT,
            db=REDIS_DB,
            decode_responses=True  # 让返回值自动解码为 str
        )
        _r = redis.Redis(connection_pool=_pool)
        _lock = threading.Lock()


    def set_param(self, key: str, value):
        """
        设置全局参数（自动 JSON 序列化）
        """
        with _lock:
            try:
                _r.set(key, json.dumps(value))
            except Exception as e:
                print(f"[global_param] 设置参数失败: {e}")

    def get_param(self, key: str, default=None):
        """
        获取全局参数（自动反序列化为 Python 类型）
        """
        try:
            val = _r.get(key)
            return json.loads(val) if val is not None else default
        except Exception as e:
            print(f"[global_param] 获取参数失败: {e}")
            return default

    def delete_param(self, key: str):
        """删除参数"""
        try:
            _r.delete(key)
        except Exception as e:
            print(f"[global_param] 删除参数失败: {e}")

    def list_params(self, prefix: str = ""):
        """
        列出所有参数（可按前缀过滤）
        例如: list_params('robot_') → {'robot_speed': 1.5, 'robot_mode': 'auto'}
        """
        try:
            keys = _r.keys(f"{prefix}*")
            params = {k: json.loads(_r.get(k)) for k in keys}
            return params
        except Exception as e:
            print(f"[global_param] 列出参数失败: {e}")
            return {}