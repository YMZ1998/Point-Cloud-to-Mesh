import os
import numpy as np
from adjust_points import adjust_spacing, replace_first_last_with_average
from generate_image_by_bin import read_bin_in_groups
from pipeline import convert_to_3d, generate_mesh
from sorted_points import sort_points


def process_case(case, file_path, data_path):
    NUM_IMAGES = 12
    ANGLE_STEP = 15
    PIXEL_SPACING = 0.04979  # mm per pixel
    IMAGE_WIDTH = 240
    NUM_POINTS = 49
    file_path_x = os.path.join(file_path, "BDX.BIN")
    file_path_y = os.path.join(file_path, "BDY.BIN")
    groups_x = read_bin_in_groups(file_path_x, group_size=NUM_POINTS, discard_size=51-NUM_POINTS, num_groups=NUM_IMAGES)
    groups_y = read_bin_in_groups(file_path_y, group_size=NUM_POINTS, discard_size=51-NUM_POINTS, num_groups=NUM_IMAGES)

    xy_groups = [list(zip(y // 2, x // 2)) for x, y in zip(groups_x, groups_y) if len(x) > 0 and len(y) > 0]

    all_rings_2d = []
    all_rings_3d = []

    for i in range(NUM_IMAGES):
        angle_deg = i * ANGLE_STEP
        center_x = IMAGE_WIDTH // 2
        points_2d = xy_groups[i]

        while len(points_2d) < NUM_POINTS:
            points_2d.append(points_2d[-1])

        if np.all(np.array(points_2d) == [0, 0]):
            print(f"No points found in image: {i}")
            continue

        points_2d = sort_points(points_2d)
        points_2d = adjust_spacing(points_2d)

        from sorted_points import visualize_points
        # visualize_points(points_2d)

        points_3d = [convert_to_3d(u, v, angle_deg, center_x, PIXEL_SPACING) for u, v in points_2d]

        all_rings_2d.append(points_2d)
        all_rings_3d.append(points_3d)

    all_rings_3d = replace_first_last_with_average(all_rings_3d)

    generate_mesh(all_rings_3d, os.path.join(data_path, f"{case}.ply"))


if __name__ == "__main__":
    case = 'PATI018'
    base_path = r"D:\Data\超声\20250207_82521\20250207"
    file_path = os.path.join(base_path, case)
    data_path = r"D:\Code\us_recon\data"

    # 运行处理
    process_case(case, file_path, data_path)
