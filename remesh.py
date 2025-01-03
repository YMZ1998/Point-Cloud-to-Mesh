file_path = r"D:\Code\us_recon\data\output.ply"
output_path = r"D:\Code\us_recon\data\output_mesh.ply"

if __name__ == "__main__":
    import alphashape
    import numpy as np
    import open3d as o3d
    from shapely.geometry import Polygon, MultiPolygon

    # Step 1: 加载点云
    pcd = o3d.io.read_point_cloud(file_path)
    points = np.asarray(pcd.points)

    # Step 2: 生成 Alpha Shape
    alpha = 0.01  # 调整 alpha 值控制边界的细节
    alpha_shape = alphashape.alphashape(points, alpha)
    alpha_shape.export(output_path)


