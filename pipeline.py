import os
import numpy as np
from PIL import Image
import open3d as o3d

# 参数配置
IMAGE_DIR = r"D:\Code\us_recon\data\test2"  # 图像目录
NUM_IMAGES = 12
ANGLE_STEP = 15  # 每张图像间隔角度
# PIXEL_SPACING = 1. # mm per pixel
PIXEL_SPACING = 0.04979  # mm per pixel
INTENSITY_THRESHOLD = 128  # 阈值判断白点
IMAGE_WIDTH = 240
IMAGE_HEIGHT = 200

# 输出路径
OUTPUT_PCD_PATH = r"D:\Code\us_recon\data\output_points.ply"
OUTPUT_MESH_PATH = r"D:\Code\us_recon\data\output_mesh.ply"


def extract_2d_points(img: Image.Image):
    img = np.array(img)
    points = []
    for v in range(img.shape[0]):
        for u in range(img.shape[1]):
            if img[v, u] > INTENSITY_THRESHOLD:
                points.append((u, v))
    return points


def convert_to_3d(u, v, angle_deg, center_x, spacing):
    x_offset = (u - center_x) * spacing
    z = v * spacing
    angle_rad = np.deg2rad(angle_deg)
    x = x_offset * np.cos(angle_rad)
    y = x_offset * np.sin(angle_rad)
    return [x, y, z]


def sort_points(points):
    print("开始排序点数:", len(points))
    print(points)

    if len(points) < 3:
        return points  # 无需排序

    points = np.array(points)

    # 以中轴（中间X值）作为分界线
    x_split = np.median(points[:, 0])

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


if __name__ == "__main__":
    all_rings_2d = []
    all_rings_3d = []

    for i in range(NUM_IMAGES):
        filename = os.path.join(IMAGE_DIR, f"{i + 1:02}.BMP")
        if not os.path.exists(filename):
            print(f"未找到图像: {filename}")
            continue

        img = Image.open(filename).convert("L")
        angle_deg = i * ANGLE_STEP
        center_x = img.width // 2
        points_2d = extract_2d_points(img)

        points_2d = sort_points(points_2d)

        points_3d = [convert_to_3d(u, v, angle_deg, center_x, PIXEL_SPACING) for u, v in points_2d]

        all_rings_2d.append(points_2d)
        all_rings_3d.append(points_3d)

    # 构建点云
    vertices = np.concatenate(all_rings_3d, axis=0)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(vertices)
    o3d.io.write_point_cloud(OUTPUT_PCD_PATH, pcd)
    print(f"✅ 点云已保存: {OUTPUT_PCD_PATH}")

    # 构建 mesh
    # Layer_i:    p0 --- p1
    #               \  /  \
    # Layer_i + 1:  q0 --- q1
    triangles = []
    vert_start_idx = 0
    for i in range(NUM_IMAGES - 1):
        ring1 = all_rings_3d[i]
        ring2 = all_rings_3d[i + 1]

        min_len = min(len(ring1), len(ring2))
        for j in range(min_len - 1):
            idx1 = vert_start_idx + j
            idx2 = vert_start_idx + j + 1
            idx3 = vert_start_idx + len(ring1) + j
            idx4 = vert_start_idx + len(ring1) + j + 1

            triangles.append([idx1, idx2, idx3])
            triangles.append([idx2, idx4, idx3])

        vert_start_idx += len(ring1)

    # 闭合最后一圈：将最后一圈和第一圈连接
    ring1 = all_rings_3d[-1]
    ring2 = all_rings_3d[0]
    offset1 = sum(len(r) for r in all_rings_3d[:-1])
    offset2 = 0
    print(offset1, offset2)
    min_len = min(len(ring1), len(ring2))
    for j in range(min_len - 1):
        idx1 = offset1 + j
        idx2 = offset1 + j + 1
        idx3 = offset2 + len(ring2) - j
        idx4 = offset2 + len(ring2) - j - 1

        triangles.append([idx1, idx2, idx3])
        triangles.append([idx2, idx4, idx3])

    # 创建网格
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(np.array(triangles))
    mesh.compute_vertex_normals()

    # 关闭光照
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(mesh)

    render_option = vis.get_render_option()
    render_option.light_on = False  # 关闭光照
    render_option.point_size = 5  # 可选：调整点的大小
    render_option.background_color = np.asarray([1, 1, 1])  # 可选：设置背景颜色为白色

    vis.run()
    vis.destroy_window()

    o3d.io.write_triangle_mesh(OUTPUT_MESH_PATH, mesh)
    print(f"✅ Mesh 已保存: {OUTPUT_MESH_PATH}")
