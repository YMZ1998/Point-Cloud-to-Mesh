import os

from PIL import Image, ImageDraw
import math
import random

# 图像宽高
IMAGE_WIDTH = 240
IMAGE_HEIGHT = 200

# 椭圆半轴长度
ELLIPSE_A = 30  # 水平半轴
ELLIPSE_B = 40  # 垂直半轴

# 中心点坐标
CENTER_X = IMAGE_WIDTH // 2
CENTER_Y = IMAGE_HEIGHT // 2


# 创建空白的 8 位 BMP 图像
def create_empty_bmp(width, height):
    image = Image.new("L", (width, height), 0)  # "L" 模式表示8位灰度图，初始值为0（黑色）
    return image


# 绘制带随机波动的椭圆散点
def draw_ellipse_with_noise(image, a, b, center_x, center_y, num_points, noise_level):
    draw = ImageDraw.Draw(image)

    # 生成均匀分布的角度，采样为 num_points 个点
    for i in range(num_points):
        angle = i * (360 / num_points)  # 计算等间隔角度
        radians = math.radians(angle)  # 转换为弧度
        x = int(a * math.cos(radians))
        y = int(b * math.sin(radians))

        # 添加随机波动
        noise_x = random.randint(-noise_level, noise_level)
        noise_y = random.randint(-noise_level, noise_level)

        # 映射到图像中心，并应用波动
        pixel_x = center_x + x + noise_x
        pixel_y = center_y + y + noise_y

        # 绘制散点（灰度值设置为255，表示白色）
        if 0 <= pixel_x < IMAGE_WIDTH and 0 <= pixel_y < IMAGE_HEIGHT:
            image.putpixel((pixel_x, pixel_y), 255)


# 主程序
if __name__ == "__main__":
    save_path = r"D:\\Code\\us_recon\\data\\test"
    os.makedirs(save_path, exist_ok=True)
    for i in range(1, 13):  # 循环生成 12 幅图像
        # 创建空白图像
        bmp_image = create_empty_bmp(IMAGE_WIDTH, IMAGE_HEIGHT)

        # 绘制带随机波动的椭圆散点
        draw_ellipse_with_noise(
            bmp_image,
            ELLIPSE_A, ELLIPSE_B,
            CENTER_X, CENTER_Y,
            num_points=50,  # 采样点数
            noise_level=1  # 波动范围（±像素值）
        )

        # 保存为 BMP 文件，以 01.BMP 到 12.BMP 命名
        filename = os.path.join(save_path, f"{i:02}.BMP")  # 格式化文件名
        bmp_image.save(filename)
        print(f"椭圆散点图已生成并保存为 {filename}")
