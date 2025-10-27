import os
import numpy as np

from scipy.spatial.transform import Rotation as R
from scipy.spatial import KDTree
from math import acos, tan, atan, degrees, sqrt
from moka_utils.redis_param import RedisParam as rdsp

def corner_angle_discrimination(point, data, direction):
    """
    判断点是否为角点
    :param point: 待判断的点
    :param data: 数据集
    :param direction: 方向向量
    :return: 是否为角点
    """
    # 构建KDTree索引
    tree = KDTree(data)
    indices = np.array(tree.query_ball_point(point, r=30, p=2))  # 返回符合要求点的索引
    
    # 计算与点point的距离
    distances = np.array([np.linalg.norm(point - data[index]) for index in indices])
    distances_filter = distances >= 10
    indices_filtered = indices[distances_filter]
    data_ball = data[indices_filtered]

    # 获取与方向向量相似的点云
    data_similar = []
    for i in range(len(data_ball)):
        vec = np.array(data_ball[i]) - point
        norm_vec = np.linalg.norm(vec)
        norm_dir = np.linalg.norm(direction)
        
        # 防止零向量
        if norm_vec == 0 or norm_dir == 0:
            continue

        # 计算余弦值
        cos_angle = np.dot(vec, direction) / (norm_vec * norm_dir)
        
        # 将 cos_angle 限制在 [-1, 1] 范围内
        cos_angle = np.clip(cos_angle, -1.0, 1.0)

        angle = acos(cos_angle)  # 现在 safe
        angle2deg = degrees(angle)
        
        if angle2deg <= 15:
            data_similar.append(data_ball[i])

    return len(data_similar) >= 1

def check_x_axis_direction(rotation_matrix):
    """
    根据旋转矩阵检查 x 轴方向
    :param rotation_matrix: 旋转矩阵
    :return: 1 表示 x 轴方向向右，-1 表示 x 轴方向向左，0 表示 x 轴方向垂直于 z 轴
    """
    # 提取旋转后的 x 轴方向向量
    x_prime = rotation_matrix[:, 0]

    # 检查 x 轴的 z 分量
    if x_prime[2] > 0:
        return 1
    elif x_prime[2] < 0:
        return -1
    else:
        return 0

def set_yaw_angle(R_mat, yaw):
    """
    根据给定的yaw角和旋转矩阵设置新的yaw角,绕X轴旋转
    :param R_mat: 旋转矩阵 (3x3)
    :param yaw: yaw角（弧度）
    :return: 四元数列表
    """
    Rx_yaw = np.array([
        [1, 0, 0],
        [0, np.cos(yaw), -np.sin(yaw)],
        [0, np.sin(yaw), np.cos(yaw)]
    ])

    Rz_pi = np.array([
        [np.cos(np.pi), -np.sin(np.pi), 0],
        [np.sin(np.pi), np.cos(np.pi), 0],
        [0, 0, 1]
    ])

    # 计算旋转矩阵
    R1_mat = np.matmul(R_mat, Rx_yaw)

    sign = check_x_axis_direction(R1_mat)
    if sign == -1:  # 注意：你原逻辑是 -1 表示向左，需要绕Z轴180°翻转
        R1_mat = np.matmul(R1_mat, Rz_pi)
    # 如果 sign == 1，保持不变；如果是 0，也可以考虑处理

    # 转为四元数
    q = R.from_matrix(R1_mat).as_quat()
    return q.tolist()  # 返回 list

def calculate_angle_with_xy_plane(point1, point2):
    """
    计算两个点构成的向量与xy平面的夹角（单位：度）
    :param point1: 起点 [x, y, z]
    :param point2: 终点 [x, y, z]
    :return: 夹角（度）
    """
    # 计算方向向量
    dir_vec = np.array([
        point2[0] - point1[0],
        point2[1] - point1[1],
        point2[2] - point1[2]
    ])

    # 计算在 xy 平面上的投影（z=0）
    proj_vec = np.array([
        point2[0] - point1[0],
        point2[1] - point1[1],
        0
    ])

    # 防止零向量导致除以0
    if np.linalg.norm(dir_vec) == 0 or np.linalg.norm(proj_vec) == 0:
        return 0.0

    # 计算夹角：cosθ = (a·b) / (|a||b|)
    cos_angle = np.dot(dir_vec, proj_vec) / (np.linalg.norm(dir_vec) * np.linalg.norm(proj_vec))

    # 防止浮点误差导致 cos_angle 超出 [-1, 1]
    cos_angle = np.clip(cos_angle, -1.0, 1.0)

    angle_rad = np.arccos(cos_angle)
    angle_deg = np.degrees(angle_rad)

    return angle_deg

def load_pointcloud_from_txt(file_path):
    """
    导入点云数据
    :param file_path: 点云数据文件路径
    :return: NumPy数组
    """
    points = []
    with open(file_path, 'r') as file:
        for line in file:
            # 要求每行格式为"x, y, z"，且没有多余的空格
            coordinates = line.strip().split()
            if len(coordinates) == 3:
                point = [float(coor) for coor in coordinates]
                points.append(point)
    
    return np.array(points)

def load_point_cloud_from_binary_txt(file_path):
    """
    从二进制文件中导入点云数据
    :param file_path: 点云数据文件路径
    :return: NumPy数组
    """
    with open(file_path, 'rb') as f:
        binary_data = f.read()

        # 将二进制数据转换为 NumPy 数组，(固定3列x,y,z，行数自动计算)
        point_cloud = np.frombuffer(binary_data, dtype=np.float64).reshape(-1, 3)

    return point_cloud

def get_weld(file_path):
    """
    从文件中读取焊缝起点、终点和中间点，并计算两点构成的向量
    :param file_path: 文件路径
    :return: 焊缝起点列表、焊缝终点列表、焊缝中间点列表、焊缝向量列表
    """
    with open(file_path, 'r') as file:
        # 逐行读取文件内容
        lines = file.readlines()

    start_points, end_points, midpoints, welds = [], [], [], []

    for line in lines:
        # 去除行尾的换行符并按'/'分割每一行
        points_str = line.strip().split('/')
        
        # 确保每一行都正确地分为两部分
        if len(points_str) == 2:
            point1_str = points_str[0].split(',')
            point2_str = points_str[1].split(',')
            
            # 转换字符串为浮点数列表，构造三维点
            point1 = [float(coord) for coord in point1_str]
            point2 = [float(coord) for coord in point2_str]

            midpoint = [(p2 + p1)/2 for p1, p2 in zip(point1, point2)]  #焊缝的中间点
            vector = [p2 - p1 for p1, p2 in zip(point1, point2)]        #焊缝的方向向量
            
            start_points.append(point1)
            end_points.append(point2)
            midpoints.append(midpoint)  
            welds.append(vector)   

    return start_points, end_points, midpoints, welds

def compute_Rc2w_tc2w(file_path_Tw2c):
    """
    相机和基底坐标系的转换关系
    :param file_path_Tw2c: Tw2c文件路径
    :return: Rc2w, tc2w
    """
    with open(file_path_Tw2c, 'r') as file:
        lines = file.readlines()

    lines = [line.strip().replace('[', '').replace(']', '').split() for line in lines]
    data = [[float(num) for num in line] for line in lines] #将每行的字符串转换为浮点数
    data = np.array(data)
    # 提取Rw2c和tw2c
    Rw2c = data[:3,:3]  #前3行的前3列
    tw2c = data[:3,-1]  #前3行的最后一列

    Rc2w = Rw2c.T   # 旋转矩阵的转置
    tc2w = -np.matmul(Rc2w, tw2c)   #Rc2w乘tw2c，负号表示平移方向是从相机指向基底

    return Rc2w, tc2w

def compute_pose_R(vector, weld, start_point, end_point):
    """
    根据计算出的焊接时z轴负方向的向量和由起点(vector)指向终点的向量(weld)计算旋转矩阵
    :param vector: 方向向量
    :param weld: 焊缝辅助向量
    :param start_point: 焊缝起点
    :param end_point: 焊缝终点
    :return: 旋转矩阵 是否为平缝
    """
    x, y, z = vector

    # 计算旋转角度x轴和y轴
    rotx = -np.arctan2(y, z)                    # 计算绕x轴的旋转角度，负号表示顺时针方向测量角度(绕x轴旋转影响y和z)
    roty = np.arctan2(x, np.sqrt(y**2 + z**2))

    # 构造旋转矩阵，A为绕x轴的旋转矩阵，B为绕Y轴的旋转矩阵
    # 将末端坐标系旋转到焊缝方向
    Rx_rotx = np.array([[1, 0, 0],
                [0, np.cos(rotx), -np.sin(rotx)],
                [0, np.sin(rotx), np.cos(rotx)]])
    Ry_roty = np.array([[np.cos(roty), 0, np.sin(roty)],
                [0, 1, 0],
                [-np.sin(roty), 0, np.cos(roty)]])
    Rz = np.matmul(weld, np.matmul(Rx_rotx, Ry_roty))    # 将辅助向量同样旋转A和B，并确定绕z轴的旋转角度

    # 区分平缝竖缝
    angle = calculate_angle_with_xy_plane(start_point, end_point)
    flat_weld = abs(angle) < 30

    if flat_weld: 
        rotz = np.arctan2(Rz[0], -Rz[1])
    else:
        rotz = np.arctan2(Rz[1], Rz[0])

    Rz_rotz = np.array([[np.cos(rotz), -np.sin(rotz), 0],
                [np.sin(rotz), np.cos(rotz), 0],
                [0, 0, 1]])

    Rx_pi = np.array([[1, 0, 0],
                    [0, np.cos(np.pi), -np.sin(np.pi)],
                    [0, np.sin(np.pi), np.cos(np.pi)]])

    R_mat = np.matmul(Rx_rotx, np.matmul(Ry_roty, np.matmul(Rz_rotz, Rx_pi)))

    return R_mat, flat_weld

class GetQuaternion:
    def __init__(self):
        # 参数初始化
        self.yaw = np.pi/2 - rdsp.get_param("yaw")
        self.pitch_of_Verticalweld = rdsp.get_param("pitch_of_Verticalweld")
        self.pitch_of_Horizontalweld = rdsp.get_param("pitch_of_Horizontalweld")
        self.yaw_rate = rdsp.get_param("yaw_rate")
        self.file_path = rdsp.get_param("folder_path")

        # 获取文件路径
        self.file_path_pointcloud = os.path.join(self.file_path, 'pointcloud.txt')
        self.file_path_points = os.path.join(self.file_path, 'points_plan.txt')
        self.file_path_result = os.path.join(self.file_path, 'result.txt')
        self.file_path_Tw2c = os.path.join(self.file_path, 'Tw2c.txt')

        # 获取点云和焊缝数据
        self.data = load_point_cloud_from_binary_txt(self.file_path_pointcloud)
        self.start_points, self.end_points, self.midpoints, self.welds = get_weld(self.file_path_points)
        self.Rc2w, self.tc2w = compute_Rc2w_tc2w(self.file_path_Tw2c)
        
        # 定义结果列表
        self.result_ping = []
        self.result_shu = []
        self.result_wai = []
        self.result = []

    def get_vertical_pointcloud(self, data, midpoint, weld):
        # 构建KDTree索引
        tree = KDTree(self.data)
        # 查询半径范围内的点,找出"data"中所有距离"midpoint"点在r以内的点，这里使用的是欧几里得距离（由p=2表示）
        indices = np.array(tree.query_ball_point(self.midpoint, r=30, p=2)) # 返回这些点的索引数组
        distances = np.array([np.linalg.norm(self.midpoint - self.data[index]) for index in indices])
        # 对numpy数组进行布尔索引
        # distance_filter = np.logical_and(distances >= 10, distances <= 50)
        distance_filter = distances >= 10
        # 使用布尔索引过滤出符合条件的索引
        filtered_indices = indices[distance_filter]
        # 提取符合条件的点
        data_circular = data[filtered_indices]

        # 获取竖直方向上的点云
        data_vertical = []
        for i in range(len(data_circular)):
            P = np.array(data_circular[i])
            v = P - self.midpoint
            anglecos = np.dot(v, self.weld) / (np.linalg.norm(v) * np.linalg.norm(self.weld))
            
            if abs(anglecos) <= 0.015:  #余弦值接近0，即角度接近90度
                # print(anglecos)
                data_vertical.append(P)
        data_vertical = np.array(data_vertical)

        return data_vertical

    def calculate_fangxiang(self, data, midpoint, weld):
        # 随机取竖向点云
        data_shu = self.get_vertical_pointcloud(self.data, self.midpoint, self.weld)
        random_point = data_shu[np.random.randint(0, len(data_shu))]

        # 随机点与 midpoint 定义直线向量
        unweld = random_point - self.midpoint

        distance_to_line = []

        # 遍历每个点
        for i in range(data_shu.shape[0]):
            P = data_shu[i]
            A = self.midpoint
            u = unweld
            v = P - A   

            # 计算点p到unweld的距离
            distance = np.linalg.norm(np.cross(u, v) / np.linalg.norm(u))
            distance_to_line.append(distance)

        distance_to_line = np.array(distance_to_line)

        # 都小于5为true，即data_shu只有一列，侧着拍的外缝
        all_distance_less_than_5 = np.all(distance_to_line < 5)

        if all_distance_less_than_5:
            random_index = np.random.randint(0, data_shu.shape[0])
            vector1 = data_shu[random_index] - self.midpoint
            vector1 /= np.linalg.norm(vector1)    # 单位化vector1，向量除以其模长
            fa_xian = np.cross(vector1, self.weld)  # 计算叉积，得到法向量
            fa_xian /= np.linalg.norm(fa_xian)    # 单位化法向量

            point = []
            point1 = midpoint + 50 * fa_xian    #沿法线方向偏移50个单位
            point2 = midpoint - 50 * fa_xian

            point1_c = np.matmul(self.Rc2w, point1) + self.tc2w
            point2_c = np.matmul(self.Rc2w, point2) + self.tc2w
            a = np.linalg.norm(point1_c)    #到原点的距离
            b = np.linalg.norm(point2_c) 

            if a < b:
                point = point1
            else:
                point = point2

            faxian = point - self.midpoint
            vector2 =  faxian / np.linalg.norm(faxian)  # 单位化vector2

            self.fangxiang = vector1 + vector2*1.2

            self.R_mat, self.ispingfeng = compute_pose_R(self.fangxiang, self.weld, self.start_point, self.end_point)

            if self.ispingfeng:
                ##绕X轴偏转##
                pz1 = np.array([[1, 0, 0],
                            [0, np.cos(self.yaw), -np.sin(self.yaw)],
                            [0, np.sin(self.yaw),  np.cos(self.yaw)]])
                pz2 = np.array([[1, 0, 0],
                            [0, np.cos(-self.yaw), -np.sin(-self.yaw)],
                            [0, np.sin(-self.yaw),  np.cos(-self.yaw)]])
                Z_PI = np.array([[np.cos(np.pi), -np.sin(np.pi), 0],
                        [np.sin(np.pi), np.cos(np.pi), 0],
                        [0, 0, 1]])
                R1_mat = np.matmul(self.R_mat, pz1)
                R2_mat = np.matmul(self.R_mat, pz2)

                sign = check_x_axis_direction(R1_mat)
                if sign == 1:
                    pass
                elif sign == -1:
                    R1_mat = np.matmul(R1_mat, Z_PI)
                    R2_mat = np.matmul(R2_mat, Z_PI)

                q1 = R.from_matrix(R1_mat).as_quat()
                q2 = R.from_matrix(R2_mat).as_quat()
                q1 = q1.tolist()  # 将NumPy数组转换为列表
                q2 = q2.tolist()
                # print("一个点云面的平外缝")
                # print(q1,q2)
                self.result_ping.append(("f", self.start_point, self.end_point, q1, q2))  #平外缝
                
            else:
                ##偏转##
                E = np.array([[np.cos(-self.pitch_of_Verticalweld), 0, np.sin(-self.pitch_of_Verticalweld)],
                            [0, 1, 0],
                            [-np.sin(-self.pitch_of_Verticalweld), 0, np.cos(-self.pitch_of_Verticalweld)]])

                R_mat1 = np.matmul(self.R_mat,E)
                
                q1 = R.from_matrix(R_mat1).as_quat()
                q2 = R.from_matrix(R_mat1).as_quat()
                q1 = q1.tolist()  # 将NumPy数组转换为列表
                q2 = q2.tolist()
                # print("即一个点云面的竖外缝")
                # print(q1,q2)
                self.result_wai.append(("o", self.start_point, self.end_point, q1, q2))   #竖外缝

        else:
            # 选择位于平面两侧的点
            # 随机选取data_shu中的第一个点
            random_index = np.random.randint(0, data_shu.shape[0])
            vector1 = data_shu[random_index] - self.midpoint
            vector1 /= np.linalg.norm(vector1)  # 单位化vector1

            # 随机选取下一个点，直到找到与vector1夹角大于10度的向量
            found = False
            while not found:
                random_index = np.random.randint(0, data_shu.shape[0])
                random_point = data_shu[random_index] - self.midpoint
                vector2 = random_point / np.linalg.norm(random_point)  # 单位化vector2
                
                # 计算两个向量之间的夹角（弧度）
                dot_product = np.dot(vector1, vector2)
                dot_product = np.clip(dot_product, -1.0, 1.0)  # 防止浮点数精度问题导致超出[-1, 1]范围
                angle_rad = np.arccos(dot_product)
                
                # 判断角度是否大于10度（转换为弧度）
                if angle_rad > np.deg2rad(20) and angle_rad < np.deg2rad(180):
                    found = True
                    # 如果找到了，则保存vector2
                else:
                    continue

            # 计算并保存最终的fangxiang向量, vector1与vector2均为单位向量
            vector1_sim_to_z = np.dot(vector1, [0,0,1]) / (np.linalg.norm(vector1))
            vector2_sim_to_z = np.dot(vector2, [0,0,1]) / (np.linalg.norm(vector2))
            if vector1_sim_to_z > vector2_sim_to_z:
                vector_shu = vector1
                vector_ping = vector2
            else:
                vector_shu = vector2
                vector_ping = vector1
            self.fangxiang = vector_ping + vector_shu*tan(self.pitch_of_Horizontalweld)
            # fangxiang = vector1 + vector2
            self.fangxiang /= np.linalg.norm(self.fangxiang)  # 单位化fangxiang

            self.R_mat, self.ispingfeng = compute_pose_R(self.fangxiang, self.weld, self.start_point, self.end_point)
        
        return self.fangxiang

    def judge_inner_or_outer(self, data, midpoint, weld, start_point, end_point):
        # 共享参数
        self.data = data
        self.midpoint = midpoint
        self.weld = weld
        self.start_point = start_point
        self.end_point = end_point

        self.fangxiang = self.calculate_fangxiang(self.data, self.midpoint, self.weld)

        point1 = self.midpoint + 70 * self.fangxiang 
        point2 = self.midpoint - 70 * self.fangxiang 
        
        point1_c = np.matmul(self.Rc2w, point1) + self.tc2w
        point2_c = np.matmul(self.Rc2w, point2) + self.tc2w
        a = np.linalg.norm(point1_c)
        b = np.linalg.norm(point2_c) 

        if a < b: # 内缝
            # print("两个点云面的内缝")
            self.fangxiang = self.fangxiang
        else: # 外缝
            # print("两个点云面的外缝")
            self.fangxiang = -self.fangxiang

        if self.ispingfeng:
            ##偏转##
            corner_of_start_point = corner_angle_discrimination(self.start_point, self.data, self.fangxiang)
            corner_of_end_point = corner_angle_discrimination(self.end_point, self.data, self.fangxiang)
            # pi4_pz = atan(1 / (sqrt(2) * tan(np.pi/4))) # CL公式 54.74deg 
            pi6_pz = atan(1 / (sqrt(2) * tan(np.pi/3))) # CL公式 67.5deg
            yaw_rate1 = 10
            len_hanfeng = np.linalg.norm(self.weld)
            transition_length = 1.5 * len_hanfeng
            # pi6_pz = np.pi/3
            # yaw_pz = atan(1 / (sqrt(2) * tan(self.yaw)))
            if corner_of_start_point:
                q1 = set_yaw_angle(self.R_mat, pi6_pz)
                if not corner_of_end_point:
                    angle = pi6_pz-(pi6_pz)*transition_length/yaw_rate1
                    if angle < -pi6_pz:
                        angle =-pi6_pz
                    q2 = set_yaw_angle(self.R_mat, angle)
            if corner_of_end_point:
                q2 = set_yaw_angle(self.R_mat,-pi6_pz)
                if not corner_of_start_point:
                    angle = -pi6_pz+(pi6_pz)*transition_length/yaw_rate1
                    if angle > pi6_pz:
                        angle = pi6_pz
                    q1 = set_yaw_angle(self.R_mat, angle)
            if not corner_of_start_point and not corner_of_end_point:     
                q1 = set_yaw_angle(self.R_mat, pi6_pz)
                angle = pi6_pz-(pi6_pz)*transition_length/yaw_rate1
                if angle < -pi6_pz:
                    angle =-pi6_pz
                q2 = set_yaw_angle(self.R_mat, angle)
            
            self.result_ping.append(("p", self.start_point, self.end_point, q1, q2))

            return self.result_ping, self.result_shu, self.result_wai
        else:
            ##偏转##
            E = np.array([[np.cos(-self.pitch_of_Verticalweld), 0, np.sin(-self.pitch_of_Verticalweld)],
                        [0, 1, 0],
                        [-np.sin(-self.pitch_of_Verticalweld), 0, np.cos(-self.pitch_of_Verticalweld)]])

            R_mat2 = np.matmul(self.R_mat, E)
            
            q1 = R.from_matrix(R_mat2).as_quat()
            q2 = R.from_matrix(R_mat2).as_quat()
            q1 = q1.tolist()  # 将NumPy数组转换为列表
            q2 = q2.tolist()
            # print(q1,q2)
            self.result_wai.append(("s", self.start_point, self.end_point, q1, q2))

            return self.result_ping, self.result_shu, self.result_wai

    def run(self):
        for i in range(len(self.midpoints)):
            self.result_ping, self.result_shu, self.result_wai = self.judge_inner_or_outer(
                self.data,
                self.midpoints[i],
                self.welds[i],
                self.start_points[i],
                self.end_points[i],
            )

        self.result = self.result_ping + self.result_shu + self.result_wai

        # 将结果写入文件
        with open(self.file_path_result, 'w') as file:
            for group in self.result:
                # 对于每个数据组，将每个部分用逗号连接，然后在各部分间插入斜杠
                line = "/".join([",".join(map(str, sublist)) for sublist in group]) + "\n"
                file.write(line)

        print("Welding Angle calculation completed.")