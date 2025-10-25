#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField


# 点云文件所在位置
folder_path = "/home/tong/colcon_ws/data/6_Welds"
file_path_pointcloud = os.path.join(folder_path, 'pointcloud.txt')
file_path_points = os.path.join(folder_path, 'points.txt')

#======================== 点云降采样 =========================#

# 计算焊缝起点和终点构成的向量
def read_and_caculate_vectors(file_path):
    """
    从文件中读取焊缝起点和终点，并计算两点构成的向量
    :param file_path: 文件路径
    :return: 焊缝起点列表，焊缝终点列表，向量列表
    """
    start_points, end_points, vectors = [], [], []
    with open(file_path, 'r') as file:
        # 逐行读取文件内容
        lines = file.readlines()
    for line in lines:
        # 去除行尾的换行符并按'/'分割每一行
        points_str = line.strip().split('/')

        # 确保每一行被分割为两部分
        if len(points_str) == 2:
            point1_str = points_str[0].split(',')
            point2_str = points_str[1].split(',')

            # 转换字符串为浮点数列表，构造三维点
            point1 = [float(coord) for coord in point1_str]
            point2 = [float(coord) for coord in point2_str]

            start_points.append(point1)
            end_points.append(point2)

            # 计算向量：向量 = 点2 - 点1
            vector = [p2 - p1 for p1, p2 in zip(point1, point2)]

            vectors.append(vector)

    return start_points, end_points, vectors

# 读取并加载进制点云文件
def load_pointcloud_from_binary_txt(file_path):
    """
    使用内存映射方式高效读取大规模点云数据，减少所需时间
    :param file_path: 文件路径
    :return: 点云数据
    """
    s_time = time.time()
    with open(file_path, 'rb') as file:
        point_cloud = np.memmap(file, dtype=np.float64, mode='r').reshape(-1, 3)
        print(f'点云数据读取完成，用时 {time.time() - s_time:.3f}s')

    return np.array(point_cloud, copy=True)

# 点云将采样
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(load_pointcloud_from_binary_txt(file_path_pointcloud))
pcd = pcd.voxel_down_sample(voxel_size = 4)


#============================ 剔除焊缝周围的点云 ============================#

# def dynamic_cull_pointcloud_fast(data, culling_radius):
#     """
#     横缝：本体用圆柱体剔除+首尾球体剔除
#     竖缝：终点用圆柱剔除，不再进行球体剔除和半径放大，防止点云剔除过多产生碰撞
#     """
#     s_time = time.time()
#     data = np.asarray(data)

#     # 单位向量 (M, 3)
#     directions = end_points - start_points
#     norms = np.linalg.norm(directions, axis=1, keepdims=True)
#     unit_vectors = directions / norms  # 焊缝方向向量单位化

#     # 判断横缝/竖缝（与Z轴夹角小于30度为竖缝，否则为横缝）
#     z_axis = np.array([0, 0, 1])
#     angles = np.arccos(np.clip(np.dot(unit_vectors, z_axis), -1.0, 1.0)) * 180 / np.pi
#     is_vertical = (angles < 30) | (angles > 150)    # 竖缝mask

#     mask_sphere = np.zeros(len(data), dtype=bool)
#     mask_cylinder = np.zeros(len(data), dtype=bool)

#     for i in range(len(start_points)):
#         A = start_points[i]
#         B = end_points[i]
#         V = unit_vectors[i]
#         AB_length = norms[i][0]

#         AP = data - A  # (N, 3)
#         t = np.dot(AP, V)  # (N,)
#         within_segment = (t >= 0) & (t <= AB_length)
#         proj = A + np.outer(t, V)
#         dist_to_axis = np.linalg.norm(data - proj, axis=1)
#         within_radius = dist_to_axis <= culling_radius

#         if not is_vertical[i]:
#             # 横缝：本体圆柱剔除+首尾球体剔除
#             ball_centers = np.stack([A, B], axis=0)
#             ball_radii = 3 * culling_radius
#             dist_to_balls = np.linalg.norm(data[:, None, :] - ball_centers[None, :, :], axis=2)
#             mask_sphere |= np.any(dist_to_balls <= ball_radii, axis=1)
#             mask_cylinder |= within_segment & within_radius
#         else:
#             # 竖缝：Z值小的端点球体剔除+整条缝圆柱剔除
#             if A[2] < B[2]:
#                 ball_center = A
#             else:
#                 ball_center = B
#             ball_radius = 3 * culling_radius
#             dist_to_ball = np.linalg.norm(data - ball_center, axis=1)
#             mask_sphere |= dist_to_ball <= ball_radius
#             mask_cylinder |= within_segment & within_radius

#     mask_culled = mask_sphere | mask_cylinder
#     data_retained = data[~mask_culled]
#     data_culled = data[mask_culled]

#     rclpy.logging.get_logger().info(f'点云剔除完成，用时 {time.time() - s_time:.3f}s，剔除点数: {np.sum(mask_culled)}')

#     return data_retained, data_culled

start_points, end_points, vectors = np.array(read_and_caculate_vectors(file_path_points))   # 获取焊缝信息

pcd_data = np.array(pcd.points)

# 点云剔除
# data_retained2, data_culled2 = dynamic_cull_pointcloud(pcd_data, cull_radius) 
# data_scaled2 = np.array(data_retained2) / 1000

# 原始点云
data_scaled2 = np.array(pcd_data) / 1000

ptCloud_scaled1 = o3d.geometry.PointCloud()
ptCloud_scaled2 = o3d.geometry.PointCloud()

ptCloud_scaled2.points = o3d.utility.Vector3dVector(data_scaled2)

#============================== 构建ROS2消息和话题 ==========================#

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, '/pointcloud/output', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('点云发布节点启动...')


    def timer_callback(self):
        s_time = time.time()
        
        points = np.asarray(ptCloud_scaled2.points)
        msg = self.build_pointcloud2_msg(points)

        msg.header.stamp = self.get_clock().now().to_msg()  # 修正时间戳
        msg.header.frame_id = 'base_link'

        self.publisher_.publish(msg)
        self.get_logger().info(f'点云发布成功，用时 {time.time() - s_time:.3f}s')

    def build_pointcloud2_msg(self, points):
        """
        将点云数据转换为PointCloud2消息
        :param points: 点云数据
        :return: PointCloud2消息
        """
        msg = PointCloud2()

        if len(points.shape) == 3:
            msg.height = points.shape[1]
            msg.width = points.shape[0]
        else:
            msg.height = 1
            msg.width = len(points)

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = False
        msg.data = np.asarray(points, np.float32).tobytes()

        return msg

def main():
    rclpy.init()
    node = PointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()