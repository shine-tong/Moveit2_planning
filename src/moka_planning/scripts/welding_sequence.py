import os
import numpy as np

from math import acos, degrees
from moka_planning.utils.redis_param import RedisParam as rdsp

def calculate_distance(point1, point2):
    """
    计算两点之间的欧氏距离
    :param point1: 第一个点的坐标，格式为[x, y, z]
    :param point2: 第二个点的坐标，格式为[x, y, z]
    :return: 两点之间的欧氏距离
    """
    distance = np.linalg.norm(np.array(point1) - np.array(point2))

    return distance

def calculate_angle_with_xy_plane(point1, point2):
    """
    计算两点构成的向量与 xy 平面的夹角
    :param point1: 第一个点的坐标，格式为[x, y, z]
    :param point2: 第二个点的坐标，格式为[x, y, z]
    :return: 与 xy 平面的夹角（单位：度）
    """
    # 计算方向向量和其在xy平面上的投影
    dir_vec = np.array([point2[0] - point1[0], point2[1] - point1[1], point2[2] - point1[2]])
    proj_vec = np.array([point2[0] - point1[0], point2[1] - point1[1], 0])

    # 计算夹角
    angle = acos(np.dot(dir_vec, proj_vec) / (np.linalg.norm(dir_vec) * np.linalg.norm(proj_vec)))
    angle_deg = degrees(angle)

    return angle_deg

def read_points_from_txt(file_path):
    """
    从文件中读取焊缝数据
    :param file_path: 文件路径
    :return: 焊缝数据列表
    """
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    deta = [line.strip().split('/') for line in lines]
    points_list = [[tuple(map(float, pair.split(','))) for pair in line] for line in deta]

    return points_list

def get_weld(file_path):
    """
    从文件中读取焊缝起点和终点，并计算两点构成的向量
    :param file_path: 文件路径
    :return: 焊缝信息列表
    """
    data_horizontal, data_vertical = [], []
    flag_sequence = 0

    with open(file_path, 'r') as file:
        lines = file.readlines()

    for line in lines:
        flag_sequence += 1

        points_str = line.strip().split('/')    # 去除行尾换行符并按‘/’分割每一行

        point1_str = points_str[0].split(',')
        point2_str = points_str[1].split(',')

        point1 = [float(coord) for coord in point1_str]
        point2 = [float(coord) for coord in point2_str]

        angle = calculate_angle_with_xy_plane(point1, point2)

        if abs(angle) < 30:
            data_horizontal.append([point1, point2, flag_sequence])
        else:
            data_vertical.append([point1, point2, flag_sequence])

        data = data_horizontal + data_vertical

    return data_horizontal, data_vertical, data

def sort_welds_horizontal(data_horizontal, reference_point):
    """
    根据与参考点的距离对平缝进行排序
    :param data_ping: 焊缝数据列表
    :param reference_point: 参考点
    :return: 排序后的焊缝数据列表
    """
    sorted_welds_horizontal = []
    
    for i in range(len(data_horizontal)):
        # 计算焊缝起点到参考点的距离，并根据距离进行排序
        data_with_distances = [[calculate_distance(start, reference_point), 
        calculate_distance(end, reference_point), 
        start, 
        end,
        flag_sequence] for start, end, flag_sequence in data_horizontal
        ]

        for i in range(len(data_with_distances)):
            # 如果起点到参考点的距离大于终点到参考点的距离，则交换其的起点和终点的位置
            if data_with_distances[i][0] > data_with_distances[i][0]:
                data_with_distances[i][0] = data_with_distances[i][1]
                
                tem = data_with_distances[i][2]
                data_with_distances[i][2] = data_with_distances[i][3]
                data_with_distances[i][3] = tem

        data_with_distances.sort(key=lambda x: x[0])    # 根据距离data_with_distances[i][0]进行排序

        # 初始化当前结果和起点终点
        sorted_welds_horizontal.append(data_with_distances.pop(0)[-3:])
        reference_point = sorted_welds_horizontal[-1][1]
        data_horizontal = [[start, end, flag_sequence] for distances1, distances2, start, end, flag_sequence in data_with_distances]

    return sorted_welds_horizontal

def sort_welds_vertical(data_vertical, reference_point):
    """
    根据与参考点的距离对竖缝进行排序
    :param data_shu: 焊缝数据列表
    :param reference_point: 参考点
    :return: 排序后的焊缝数据列表
    """
    sorted_welds_vertical = []

    for i in range(len(data_vertical)):
        # 计算焊缝起点到参考点的距离，并根据距离进行排序
        data_with_distances = [[calculate_distance(start, reference_point), 
        start, 
        end, 
        flag_sequence] for start, end ,flag_sequence in data_vertical
        ]

        data_with_distances.sort(key=lambda x: x[0])    # 根据距离data_with_distances[i][0]进行排序 

        # 初始化结果和当前终点
        sorted_welds_vertical.append(data_with_distances.pop(0)[-3:])
        reference_point = sorted_welds_vertical[-1][1]
        data_vertical = [[start, end, flag_sequence] for distances, start, end, flag_sequence in data_with_distances]

    return sorted_welds_vertical

def sort_welds(data_horizontal, data_vertical, reference_point):
    # 对所有焊缝进行排序
    sorted_welds_horizontal = sort_welds_horizontal(data_horizontal, reference_point)

    if len(data_horizontal) > 0:
        reference_point_new = tuple(sorted_welds_horizontal[-1][1])
    else:
        reference_point_new = reference_point
    
    sorted_welds_vertical = sort_welds_vertical(data_vertical, reference_point_new)

    sorted_welds = sorted_welds_horizontal + sorted_welds_vertical

    # 传递焊接顺序
    flag_sequence = [flag_sequence for start, end, flag_sequence in sorted_welds]
    rdsp.set_param('welding_sequence', flag_sequence)
    sorted_welds_noflag = [[start, end] for start, end, flag_sequence in sorted_welds]

    return sorted_welds_noflag


def run():
    # 定义参考点(当前所使用工具的末端位置
    reference_point = (1172.85077147, -1.50259925, 668.30298144)

    # 获取文件路径
    file_path = rdsp.get_param('folder_path')
    # file_path_pointcloud = os.path.join(file_path, 'pointcloud.txt')
    file_path_points = os.path.join(file_path, 'points.txt')
    # file_path_result = os.path.join(file_path, 'result.txt')
    file_path_points_plan = os.path.join(file_path, 'points_plan.txt')

    data_horizontal, data_vertical, data = get_weld(file_path_points)
    sorted_welds_noflag = sort_welds(data_horizontal, data_vertical, reference_point)

    # 将结果写入 points_plan.txt
    with open(file_path_points_plan, 'w') as file:
        for start, end in sorted_welds_noflag:
            # 将每一对起点和终点转换为字符串，并用逗号连接，最后以换行符分隔不同的焊缝对
            line = "{},{},{}".format(start[0], start[1], start[2]) + "/" + "{},{},{}".format(end[0], end[1], end[2]) + "\n"
            file.write(line)

    print("Welding sequence calculation completed.")

if __name__ == "__main__":
    run()