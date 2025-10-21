# /home/tong/colcon_ws/src/moka_planning/utils/redis_param.py
"""
RedisParam — 全局参数共享模块 (基于 Redis)
-----------------------------------------
特性:
- 支持直接通过类方法调用，无需实例化
- 自动初始化连接（仅一次）
- 自动检测 Redis 连接状态并重连
- 支持多线程安全访问
- 适用于多节点全局参数共享
"""

import redis
import json
import threading
import time


class RedisParam:
    _r = None
    _lock = threading.Lock()
    _initialized = False

    # Redis 连接配置
    _REDIS_HOST = 'localhost'
    _REDIS_PORT = 6379
    _REDIS_DB = 0

    @classmethod
    def _init_connection(cls, force_reconnect=False):
        """
        初始化或重连 Redis。
        - force_reconnect=True 时强制重新连接
        """
        if cls._initialized and not force_reconnect:
            return

        max_retries = 5
        for attempt in range(1, max_retries + 1):
            try:
                pool = redis.ConnectionPool(
                    host=cls._REDIS_HOST,
                    port=cls._REDIS_PORT,
                    db=cls._REDIS_DB,
                    decode_responses=True
                )
                cls._r = redis.Redis(connection_pool=pool)
                cls._r.ping()  # 测试连接
                cls._initialized = True
                print(f"Redis 已成功连接: (host={cls._REDIS_HOST}, port={cls._REDIS_PORT}, db={cls._REDIS_DB})")
                return
            except Exception as e:
                print(f"Redis 连接失败{e}, 正在尝试重新连接...")
                if attempt < max_retries:
                    time.sleep(1)
                else:
                    print("无法连接 Redis, 检查 Redis 服务是否开启!")
                    cls._r = None
                    cls._initialized = False

    @classmethod
    def _ensure_connection(cls):
        """确保连接可用，若断开则自动重连"""
        if not cls._initialized or cls._r is None:
            cls._init_connection(force_reconnect=True)
        else:
            try:
                cls._r.ping()
            except redis.ConnectionError:
                print("Redis 连接中断, 正尝试重新连接...")
                cls._init_connection(force_reconnect=True)

    # -------------------------------
    # 公共方法 (均为类方法)
    # -------------------------------

    @classmethod
    def set_param(cls, key: str, value):
        """设置全局参数"""
        cls._ensure_connection()
        if not cls._r:
            print("Redis 未连接, 无法设置参数!")
            return
        with cls._lock:
            try:
                cls._r.set(key, json.dumps(value))
            except Exception as e:
                print(f"Redis 参数设置失败: {e}")

    @classmethod
    def get_param(cls, key: str, default=None):
        """获取全局参数"""
        cls._ensure_connection()
        if not cls._r:
            print("Redis 未连接, 返回默认值!")
            return default
        try:
            val = cls._r.get(key)
            return json.loads(val) if val is not None else default
        except Exception as e:
            print(f"Redis 参数获取失败: {e}")
            return default

    @classmethod
    def delete_param(cls, key: str):
        """删除参数"""
        cls._ensure_connection()
        if not cls._r:
            print("Redis 未连接, 无法删除参数!")
            return
        try:
            cls._r.delete(key)
        except Exception as e:
            print(f"Redis 参数删除失败: {e}")

    @classmethod
    def list_params(cls, prefix: str = ""):
        """列出所有参数（可按前缀过滤）"""
        cls._ensure_connection()
        if not cls._r:
            print("Redis 未连接, 无法列出参数!")
            return {}
        try:
            keys = cls._r.keys(f"{prefix}*")
            return {k: json.loads(cls._r.get(k)) for k in keys}
        except Exception as e:
            print(f"Redis 参数列出失败: {e}")
            return {}
    
    @classmethod    
    def pub_plan_result(cls, result_dict):
        """发布路径规划结果"""
        cls._ensure_connection()
        
        if not cls._r:
            print('Redis 未连接，无法发布规划结果!')
            return
        try:
            message = json.dumps(result_dict, ensure_ascii=False)
            cls._r.publish('ros_plan_result', message)
            print('规划结果已成功发布!')
        except Exception as e:
            print(f'规划结果发布失败：{e}!')