import os

import numpy as np
import open3d as o3d
from PIL import Image

from adjust_points import adjust_spacing, replace_first_last_with_average
from sorted_points import sort_points


def extract_2d_points(img: Image.Image):
    img = np.array(img)
    points = []
    for v in range(img.shape[0]):
        for u in range(img.shape[1]):
            if img[v, u] > 128:
                points.append((u, v))
    return points


def convert_to_3d(u, v, angle_deg, center_x, spacing):
    x_offset = (u - center_x) * spacing
    y = v * spacing
    angle_rad = np.deg2rad(angle_deg)
    x = x_offset * np.cos(angle_rad)
    z = x_offset * np.sin(angle_rad)
    return [x, y, z]


def generate_point_cloud(all_rings_3d, output_path):
    vertices = np.concatenate(all_rings_3d, axis=0)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(vertices)
    o3d.io.write_point_cloud(output_path, pcd)
    print(f"Point cloud saved: {output_path}")


def generate_mesh(all_rings_3d, output_path):
    triangles = []
    vert_start_idx = 0

    # Build triangles between consecutive rings
    for i in range(len(all_rings_3d) - 1):
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

        # Closing the ring
        last_idx_ring1 = vert_start_idx + len(ring1) - 1
        first_idx_ring1 = vert_start_idx
        last_idx_ring2 = vert_start_idx + len(ring1) + len(ring2) - 1
        first_idx_ring2 = vert_start_idx + len(ring1)

        triangles.append([last_idx_ring1, first_idx_ring1, last_idx_ring2])
        triangles.append([first_idx_ring1, first_idx_ring2, last_idx_ring2])

        vert_start_idx += len(ring1)

    # Connecting the last ring with the first ring
    ring1 = all_rings_3d[-1]
    ring2 = all_rings_3d[0]
    offset1 = sum(len(r) for r in all_rings_3d[:-1])
    offset2 = 0
    min_len = min(len(ring1), len(ring2))

    for j in range(min_len - 1):
        idx1 = offset1 + j
        idx2 = offset1 + j + 1
        idx3 = offset2 + len(ring2) - j - 1
        idx4 = offset2 + len(ring2) - j - 2

        triangles.append([idx1, idx2, idx3])
        triangles.append([idx2, idx4, idx3])

    last_idx_ring1 = offset1 + len(ring1) - 1
    first_idx_ring1 = offset1
    last_idx_ring2 = offset2 + len(ring2) - 1
    first_idx_ring2 = offset2

    triangles.append([last_idx_ring1, first_idx_ring1, last_idx_ring2])
    triangles.append([first_idx_ring1, first_idx_ring2, last_idx_ring2])

    # Create the mesh
    vertices = np.concatenate(all_rings_3d, axis=0)
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(np.array(triangles))
    mesh.compute_vertex_normals()

    # Save the mesh to file
    o3d.io.write_triangle_mesh(output_path, mesh)
    print(f"Mesh saved: {output_path}")

    vis_mesh(mesh)


def vis_mesh(mesh):
    mesh.compute_vertex_normals()  # 确保法线可用

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    vis.add_geometry(mesh)

    render_option = vis.get_render_option()
    render_option.light_on = False
    render_option.background_color = np.asarray([1.0, 1.0, 1.0])
    render_option.mesh_show_back_face = True
    render_option.line_width = 1.0
    render_option.mesh_show_wireframe = True

    vis.run()
    vis.destroy_window()


def test_case(case):
    # Parameters
    DATA_PATH = r"D:\Code\us_recon\data"
    IMAGE_DIR = os.path.join(DATA_PATH, case)
    NUM_IMAGES = 12
    ANGLE_STEP = 15
    PIXEL_SPACING = 0.04979  # mm per pixel
    IMAGE_WIDTH = 240
    IMAGE_HEIGHT = 200

    OUTPUT_PCD_PATH = os.path.join(DATA_PATH, f"{case}_output_points.ply")
    OUTPUT_MESH_PATH = os.path.join(DATA_PATH, f"{case}.ply")

    all_rings_2d = []
    all_rings_3d = []

    for i in range(NUM_IMAGES):
        filename = os.path.join(IMAGE_DIR, f"{i + 1:02}.BMP")
        if not os.path.exists(filename):
            print(f"Image not found: {filename}")
            continue

        img = Image.open(filename).convert("L")
        angle_deg = i * ANGLE_STEP
        center_x = IMAGE_WIDTH // 2
        points_2d = extract_2d_points(img)

        if len(points_2d) < 10:
            print(f"No points found in image: {filename}")
            continue

        points_2d = sort_points(points_2d)
        points_2d = adjust_spacing(points_2d)

        # from sorted_points import visualize_points
        # visualize_points(points_2d)

        points_3d = [convert_to_3d(u, v, angle_deg, center_x, PIXEL_SPACING) for u, v in points_2d]

        all_rings_2d.append(points_2d)
        all_rings_3d.append(points_3d)

    # Adjust first and last rings
    all_rings_3d = replace_first_last_with_average(all_rings_3d)

    # Generate and save point cloud and mesh
    # generate_point_cloud(all_rings_3d, OUTPUT_PCD_PATH)
    generate_mesh(all_rings_3d, OUTPUT_MESH_PATH)


if __name__ == "__main__":
    # test_case('test')
    test_case('test2')
