import numpy as np
import matplotlib.pyplot as plt
import os


def read_bin_in_groups(file_path, dtype=np.uint16, group_size=49, discard_size=2, num_groups=12):
    if not os.path.exists(file_path):
        print(f"文件不存在: {file_path}")
        return []

    data = np.fromfile(file_path, dtype=dtype)
    # print(f"读取文件 {file_path}，数据长度为 {len(data)}")
    # print(data)

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


def plot_xy_group(xy_group, index, save=False, save_dir="output"):
    if not xy_group:
        return

    x_vals, y_vals = zip(*xy_group)
    fig, ax = plt.subplots(figsize=(5, 5))
    ax.scatter(x_vals, y_vals, s=2, c='black')
    ax.invert_yaxis()
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_frame_on(False)
    plt.title(f"Group {index + 1}")

    if save:
        os.makedirs(save_dir, exist_ok=True)
        plt.savefig(os.path.join(save_dir, f"group_{index + 1}.png"), bbox_inches='tight', dpi=300)
    else:
        plt.show()

    plt.close()


if __name__ == "__main__":
    file_path = r"D:\Data\超声\20250207_82521\20250207\PATI005"
    file_path_x = os.path.join(file_path, "BDX.BIN")
    file_path_y = os.path.join(file_path, "BDY.BIN")

    groups_x = read_bin_in_groups(file_path_x)
    groups_y = read_bin_in_groups(file_path_y)

    xy_pairs = [list(zip(x, y)) for x, y in zip(groups_x, groups_y) if len(x) > 0 and len(y) > 0]

    for i, xy_group in enumerate(xy_pairs):
        print(f"第 {i + 1} 组坐标对: {xy_group}")
        plot_xy_group(xy_group, i, save=False)
