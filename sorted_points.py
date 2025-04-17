import numpy as np
import matplotlib.pyplot as plt


def sort_points(points, center=None):
    # print("开始排序点数:", len(points))
    # print(points)

    if len(points) < 3:
        return points  # 无需排序

    points = np.array(points)

    # 默认中点为所有点的重心
    if center is None:
        center = np.mean(points, axis=0)
        center[0] = 120

    print("计算得到的中心点:", center)

    # 计算每个点的相对坐标
    dx = points[:, 0] - center[0]
    dy = points[:, 1] - center[1]

    # 计算极角，atan2 返回的是从 -π 到 π 的值
    angles = np.arctan2(dy, dx)

    # 将角度调整为以90度为起始点（即原角度 + π/2），并标准化到 [0, 2π]
    angles = (angles + np.pi / 2) % (2 * np.pi)

    # 按角度排序（逆时针排序：从小到大排序）
    sorted_indices = np.argsort(angles)[::-1]  # 逆时针方向
    sorted_points = points[sorted_indices].tolist()

    if abs(sorted_points[0][0] - 120) > abs(sorted_points[-1][0] - 120):
        sorted_points = [sorted_points[-1]] + sorted_points[:-1]

    print(sorted_points[0], sorted_points[-1])
    print("排序后的点集：", sorted_points)

    return sorted_points


def sort_points2(points):
    print("开始排序点数:", len(points))
    print(points)

    if len(points) < 3:
        return points  # 无需排序

    points = np.array(points)

    # 以中轴（中间X值）作为分界线
    x_split = np.median(points[:, 0])
    # x_split = IMAGE_WIDTH // 2

    print(x_split)

    left_side = points[points[:, 0] < x_split]
    right_side = points[points[:, 0] >= x_split]

    # 左侧按Y升序（从下往上）
    left_sorted = left_side[np.argsort(left_side[:, 1])]

    # 右侧按Y降序（从上往下）
    right_sorted = right_side[np.argsort(-right_side[:, 1])]

    # 合并为顺序排列的点集
    sorted_points = np.vstack([left_sorted, right_sorted])
    print(sorted_points)

    return sorted_points.tolist()


def visualize_points(sorted_points):
    sorted_points = np.array(sorted_points)

    plt.figure(figsize=(6, 6))
    plt.scatter(sorted_points[:, 0], sorted_points[:, 1], c='gray', label='Original Points')

    # 连线看顺序
    plt.plot(sorted_points[:, 0], sorted_points[:, 1], c='blue', label='Sorted Path')

    # 在点上标编号看顺序
    for i, (x, y) in enumerate(sorted_points):
        plt.text(x + 2, y + 1, str(i), fontsize=12, color='red')

    plt.gca().invert_yaxis()  # Y轴反转以匹配图像坐标系
    plt.legend()
    plt.title("Point Sorting Visualization")
    plt.grid(True)
    plt.axis('equal')
    plt.show()


if __name__ == "__main__":
    points = [(101, 24), (143, 24), (132, 26), (112, 27), (122, 27), (95, 29), (146, 32), (93, 35), (148, 37), (92, 39),
              (149, 41), (91, 42), (150, 44), (90, 46), (151, 48), (89, 51), (152, 52), (88, 56), (154, 57), (87, 61),
              (155, 62), (85, 66), (157, 68), (83, 73), (157, 73), (153, 76), (85, 78), (89, 81), (152, 81), (149, 83),
              (92, 84), (147, 85), (95, 86), (145, 86), (98, 87), (143, 87), (100, 88), (102, 89), (137, 89), (140, 89),
              (105, 91), (134, 91), (108, 92), (126, 92), (130, 92), (112, 93), (116, 93), (120, 93), (122, 93)]
    sorted_pts = sort_points(points)
    visualize_points(sorted_pts)
