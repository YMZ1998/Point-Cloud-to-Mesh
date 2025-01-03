import open3d as o3d

# 加载点云数据
input_file = r"D:\Code\us_recon\data\output.ply"  # 替换为你的输入点云文件路径
output_file =r"D:\Code\us_recon\data\output_with_normals.ply"  # 替换为保存路径

# 读取点云
point_cloud = o3d.io.read_point_cloud(input_file)
print("原始点云信息：")
print(point_cloud)

# 计算法线
# 参数：search_param 指定每个点计算法线时的邻域搜索半径或邻居点数
point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10, max_nn=30))

# 可选：调整法线方向，使其一致指向点云外部
point_cloud.orient_normals_consistent_tangent_plane(k=10)

# 保存结果
o3d.io.write_point_cloud(output_file, point_cloud)
print(f"已保存带法线的点云到 {output_file}")

# 可视化结果
o3d.visualization.draw_geometries([point_cloud], point_show_normal=True)
