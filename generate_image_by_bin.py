import os

import numpy as np
from PIL import Image

# 配置参数
IMAGE_WIDTH = 240
IMAGE_HEIGHT = 200
POINT_VALUE = 255  # 白点的灰度值
def read_bin_in_groups(file_path, dtype=np.uint16, group_size=49, discard_size=2, num_groups=12):
    if not os.path.exists(file_path):
        print(f"文件不存在: {file_path}")
        return []
    data = np.fromfile(file_path, dtype=dtype)
    groups = []
    index = 0
    for _ in range(num_groups):
        if index + group_size <= len(data):
            group = data[index: index + group_size]
            groups.append(group)
            index += group_size + discard_size
        else:
            break
    return groups


def create_empty_bmp(width, height):
    return Image.new("L", (width, height), 0)  # 黑底


def draw_points_on_image(image, points, value=255):
    for x, y in points:
        if 0 <= x < image.width and 0 <= y < image.height:
            image.putpixel((x, y), value)


# 主程序
if __name__ == "__main__":
    SAVE_PATH = r"D:\Code\us_recon\data\test4"
    os.makedirs(SAVE_PATH, exist_ok=True)

    file_path = r"D:\Data\超声\20250207_82521\20250207\PATI009"
    file_path_x = os.path.join(file_path, "BDX.BIN")
    file_path_y = os.path.join(file_path, "BDY.BIN")

    # 读取坐标点
    groups_x = read_bin_in_groups(file_path_x)
    groups_y = read_bin_in_groups(file_path_y)

    # 组合坐标对
    xy_groups = [list(zip(y // 2, x // 2)) for x, y in zip(groups_x, groups_y) if len(x) > 0 and len(y) > 0]

    # 遍历每组，生成对应图像
    for i, xy_group in enumerate(xy_groups):
        img = create_empty_bmp(IMAGE_WIDTH, IMAGE_HEIGHT)
        draw_points_on_image(img, xy_group, value=POINT_VALUE)
        save_name = os.path.join(SAVE_PATH, f"{i + 1:02}.BMP")
        img.save(save_name)
        print(f"保存图像: {save_name}")
