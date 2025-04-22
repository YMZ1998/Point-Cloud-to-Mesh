import numpy as np
import open3d as o3d
from collections import defaultdict


def loop_subdivision_once(mesh):
    old_vertices = np.asarray(mesh.vertices)
    old_triangles = np.asarray(mesh.triangles)

    edge_map = {}  # 存储边 => 新插值点索引
    vertex_faces = defaultdict(set)
    vertex_neighbors = defaultdict(set)

    # 构建邻接信息
    for face_id, tri in enumerate(old_triangles):
        i0, i1, i2 = tri
        vertex_faces[i0].add(face_id)
        vertex_faces[i1].add(face_id)
        vertex_faces[i2].add(face_id)

        vertex_neighbors[i0].update([i1, i2])
        vertex_neighbors[i1].update([i0, i2])
        vertex_neighbors[i2].update([i0, i1])

    new_vertices = old_vertices.tolist()
    face_points = []

    def get_or_create_edge_vertex(i1, i2):
        edge = tuple(sorted((i1, i2)))
        if edge in edge_map:
            return edge_map[edge]

        v1, v2 = old_vertices[edge[0]], old_vertices[edge[1]]

        # 查找公共邻居（用于内边判断）
        common_faces = vertex_faces[i1].intersection(vertex_faces[i2])
        if len(common_faces) == 2:  # 内边
            # 找到该边相邻的两个三角面中，除了 v1, v2 的两个点
            opp_vertices = []
            for face_idx in common_faces:
                face = old_triangles[face_idx]
                for vid in face:
                    if vid != i1 and vid != i2 and vid not in opp_vertices:
                        opp_vertices.append(vid)
            v3, v4 = old_vertices[opp_vertices[0]], old_vertices[opp_vertices[1]]
            new_v = (3/8)*(v1 + v2) + (1/8)*(v3 + v4)
        else:  # 边界边，直接取中点
            new_v = 0.5 * (v1 + v2)

        new_idx = len(new_vertices)
        new_vertices.append(new_v.tolist())
        edge_map[edge] = new_idx
        return new_idx

    # 添加边点，并生成新面
    for tri in old_triangles:
        i0, i1, i2 = tri
        a = get_or_create_edge_vertex(i0, i1)
        b = get_or_create_edge_vertex(i1, i2)
        c = get_or_create_edge_vertex(i2, i0)

        face_points.append([i0, a, c])
        face_points.append([i1, b, a])
        face_points.append([i2, c, b])
        face_points.append([a, b, c])

    # 平滑原始点
    updated_old_vertices = []
    for i, v in enumerate(old_vertices):
        n = len(vertex_neighbors[i])
        if n < 3:
            updated_old_vertices.append(v)  # 边界点不变
        else:
            beta = (5.0/8.0 - (3.0 + 2.0 * np.cos(2.0 * np.pi / n))**2 / 64.0)
            beta /= n
            neighbor_sum = np.sum([old_vertices[j] for j in vertex_neighbors[i]], axis=0)
            new_v = (1 - n * beta) * v + beta * neighbor_sum
            updated_old_vertices.append(new_v)

    # 更新原始点位置
    for i in range(len(old_vertices)):
        new_vertices[i] = updated_old_vertices[i].tolist()

    # 构造新的网格
    new_mesh = o3d.geometry.TriangleMesh()
    new_mesh.vertices = o3d.utility.Vector3dVector(np.array(new_vertices))
    new_mesh.triangles = o3d.utility.Vector3iVector(np.array(face_points))
    new_mesh.compute_vertex_normals()
    return new_mesh


if __name__ == "__main__":
    from pipeline import vis_mesh

    file_path = r"D:\Code\us_recon\data\test.ply"

    mesh = o3d.io.read_triangle_mesh(file_path)
    print(f"Mesh has {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles.")
    mesh2 = loop_subdivision_once(mesh)
    print(f"Mesh2 has {len(mesh2.vertices)} vertices and {len(mesh2.triangles)} triangles.")
    mesh3 = mesh.subdivide_loop(number_of_iterations=1)
    print(f"Mesh3 has {len(mesh3.vertices)} vertices and {len(mesh3.triangles)} triangles.")
    vis_mesh(mesh)
    vis_mesh(mesh2)
    vis_mesh(mesh3)
