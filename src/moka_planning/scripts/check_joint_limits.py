#!/usr/bin/env python3

# 检查关节限位
def check_joint_limits(joint_trajectory):
    """
    检查关节轨迹点是否超出关节限位
    :param joint_trajectory: 关节轨迹
    :return: 是否超出关节限位
    """
    is_limited = False  # 初始化为关节未超出限位
    limit_margin = True # 限制边界，用来检查关节是否超出限制条件
    
    # 遍历除了最后一个关节轨迹点的所有点
    for i in range(len(joint_trajectory.joint_trajectory.points)-1):
        if not is_limited:
            # 限位，选择当前轨迹点和相邻的下一个轨迹点
            joint_positions1 = joint_trajectory.joint_trajectory.points[i].positions
            joint_positions2 = joint_trajectory.joint_trajectory.points[i + 1].positions
            
            # 遍历当前轨迹点的每个关节位置
            for j in range(len(joint_positions1)):
                positions_diff = abs(joint_positions1[j], joint_positions2[j])  # 计算绝对差值
                
                # 此处限位阈值为 6rad
                if positions_diff > 6:
                    print('发生大角度翻转：point{}-joint{}:{}'.format(i, j+1, joint_positions1))
                    
                    # 给定关节限位值
                    joint_limits = [[],[],[],[-1.6,1.6],[],[-3.838,3.838]]
                    
                    # 当前限位点为当前关节
                    limit_point = joint_positions1[j]
                    
                    # 计算当前限位点与起点位置偏移量的绝对值
                    distance_to_limit_point = abs(joint_trajectory.joint_trajectory.points[0].positions[j] - limit_point)
                    joint_range = abs(joint_limits[j][0] - joint_limits[j][1])  # 计算关节限位范围
                    
                    # 计算限位余量：偏移量的绝对值/关节限位范围
                    margin = distance_to_limit_point / joint_range
                    if margin > 0.3:
                        limit_margin = True # 超出限制范围
                    else:
                        limit_margin = False
                        
                    is_limited = True
                    break
                
                # 此处限位阈值为 6rad
                if positions_diff > 3:
                    print("point{}-joint{}:{}".format(i,j+1,joint_positions1))
                    
                    limit_margin = True
                    is_limited = True
                    break
        else:
            break
    if not is_limited:
        print('Check OK! 轨迹有效')
    
    return is_limited, limit_margin