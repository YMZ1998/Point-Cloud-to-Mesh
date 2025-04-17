import numpy as np
import matplotlib.pyplot as plt

from sorted_points import sort_points


def adjust_spacing(sorted_points):
    points = np.array(sorted_points)

    # 计算点集的总长度
    total_distance = 0
    distances = [0]  # 起始点的距离为 0
    for i in range(1, len(points)):
        distance = np.linalg.norm(points[i] - points[i - 1])
        total_distance += distance
        distances.append(total_distance)

    # 计算原始曲线的总长度
    distances = np.array(distances)

    # 生成均匀间隔的目标位置
    uniform_positions = np.linspace(0, total_distance, len(points))

    # 基于原始距离和目标距离，进行插值
    adjusted_points = np.zeros_like(points)
    for i in range(len(points)):
        # 对于每个点，找到最接近目标位置的插值位置
        target_pos = uniform_positions[i]
        insert_idx = np.searchsorted(distances, target_pos)  # 查找最接近的插值位置
        if insert_idx == len(distances) - 1:
            adjusted_points[i] = points[-1]
        else:
            # 使用线性插值调整
            t = (target_pos - distances[insert_idx - 1]) / (distances[insert_idx] - distances[insert_idx - 1])
            adjusted_points[i] = points[insert_idx - 1] + t * (points[insert_idx] - points[insert_idx - 1])

    return adjusted_points.tolist()


def visualize_points(sorted_points, adjusted_points):
    sorted_points = np.array(sorted_points)
    adjusted_points = np.array(adjusted_points)

    # 创建一个新的图形窗口
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))  # 创建1行2列的子图

    # 原始点的可视化
    axes[0].scatter(sorted_points[:, 0], sorted_points[:, 1], c='gray', label='Original Points')
    axes[0].plot(sorted_points[:, 0], sorted_points[:, 1], c='gray', linestyle='--', label='Original Path')
    axes[0].invert_yaxis()  # Y轴反转以匹配图像坐标系
    axes[0].set_title("Original Points")
    axes[0].legend()
    axes[0].grid(True)
    axes[0].axis('equal')

    # 调整后的点的可视化
    axes[1].scatter(adjusted_points[:, 0], adjusted_points[:, 1], c='blue', label='Adjusted Points')
    axes[1].plot(adjusted_points[:, 0], adjusted_points[:, 1], c='blue', linestyle='--', label='Adjusted Path')
    axes[1].invert_yaxis()  # Y轴反转以匹配图像坐标系
    axes[1].set_title("Adjusted Points")
    axes[1].legend()
    axes[1].grid(True)
    axes[1].axis('equal')

    plt.tight_layout()  # 自动调整子图布局
    plt.show()


def replace_first_last_with_average(points):
    new_points = []

    # 初始化第一个和最后一个点，确保它们是float类型
    first_point = np.array(points[0][0], dtype=np.float64)
    middle_point = np.array(points[0][len(points[0]) // 2], dtype=np.float64)

    print("first_point:", first_point)
    print("middle_point:", middle_point)

    for group in points[1:]:
        first_point += np.array(group[0], dtype=np.float64)
        middle_point += np.array(group[len(group) // 2], dtype=np.float64)

    # 计算平均值
    first_point = first_point / len(points)
    middle_point = middle_point / len(points)


    print("first_point:", first_point)
    print("middle_point:", middle_point)

    for group in points:
        group[0] = first_point.tolist()
        group[1] = first_point.tolist()
        group[-1] = first_point.tolist()
        group[len(group) // 2] = middle_point.tolist()
        group[len(group) // 2-1] = middle_point.tolist()
        group[len(group) // 2+1] = middle_point.tolist()

        print(group)
        new_points.append(group)

    return new_points


def distance(p1, p2):
    """计算两个点之间的欧几里得距离"""
    return np.linalg.norm(np.array(p1) - np.array(p2))


def merge_close_points(points, threshold=0.1):
    new_points = []

    for group in points:
        new_group = [group[0]]  # 保持第一个点不变

        # 遍历当前组内的点，比较相邻点的距离
        for i in range(1, len(group) - 1):
            p1 = group[i - 1]
            p2 = group[i]
            p3 = group[i + 1]

            # 计算当前点与前一个点、下一个点的距离
            dist1 = distance(p1, p2)
            dist2 = distance(p2, p3)

            print(dist1, dist2)

            # 如果当前点与前一个点的距离小于阈值，将它们合并
            if dist1 < threshold:
                merged_point = (np.array(p1) + np.array(p2)) / 2
                new_group[-1] = merged_point.tolist()  # 替换前一个点
            else:
                new_group.append(p2)

            # 如果当前点与下一个点的距离小于阈值，将它们合并
            if dist2 < threshold:
                merged_point = (np.array(p2) + np.array(p3)) / 2
                new_group[-1] = merged_point.tolist()  # 替换当前点
            else:
                new_group.append(p3)

        new_group.append(group[-1])  # 保持最后一个点不变
        new_points.append(new_group)

    return new_points

if __name__ == "__main__":
    points1 = [(119, 60), (121, 60), (124, 60), (116, 61), (130, 61), (110, 63), (133, 63), (107, 64), (104, 68),
               (136, 68), (138, 69), (100, 71), (142, 72), (99, 73), (95, 77), (145, 77), (95, 82), (146, 82),
               (148, 86), (94, 87), (92, 92), (150, 92), (91, 95), (149, 95), (151, 99), (90, 100), (90, 104),
               (149, 105), (90, 108), (149, 108), (148, 114), (92, 115), (145, 119), (93, 120), (95, 123), (145, 123),
               (100, 126), (142, 126), (100, 129), (138, 129), (104, 132), (135, 132), (109, 136), (112, 137),
               (132, 137), (115, 138), (120, 138), (121, 138), (130, 139), (126, 140)]
    points2 = [(114, 61), (121, 61), (128, 61), (111, 62), (119, 62), (126, 62), (131, 63), (107, 64), (137, 66),
               (105, 67), (139, 70), (102, 71), (99, 72), (140, 74), (97, 77), (144, 78), (94, 80), (145, 82), (93, 85),
               (147, 87), (90, 91), (149, 92), (90, 94), (149, 96), (91, 100), (150, 100), (90, 106), (148, 106),
               (91, 109), (148, 110), (94, 115), (148, 115), (95, 118), (146, 120), (143, 123), (96, 124), (142, 126),
               (99, 127), (140, 129), (102, 130), (135, 132), (105, 134), (108, 137), (112, 137), (132, 137),
               (130, 138), (116, 140), (120, 140), (122, 140), (124, 140)]

    points_list = [points1, points2]

    sorted_points_list = []

    for points in points_list:
        sorted_pts = sort_points(points)  # 先排序点
        adjusted_pts = adjust_spacing(sorted_pts)  # 调整间距
        # print(len(sorted_pts), len(adjusted_pts))
        # visualize_points(sorted_pts, adjusted_pts)  # 可视化
        sorted_points_list.append(adjusted_pts)

    print(sorted_points_list[0])
    print(sorted_points_list[1])
    replace_first_last_with_average(sorted_points_list)
    print(sorted_points_list[0])
    print(sorted_points_list[1])
