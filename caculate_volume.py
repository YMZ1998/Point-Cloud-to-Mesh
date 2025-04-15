import pyvista as pv
import trimesh
import vtk

# 设置网格文件路径
OUTPUT_MESH_PATH = r"D:\Code\us_recon\data\output_mesh.ply"


def get_volume_pyvista(file_path):
    mesh = pv.read(file_path)
    return mesh.volume


def get_volume_vtk(file_path):
    reader = vtk.vtkPLYReader()
    reader.SetFileName(file_path)
    reader.Update()

    polydata = reader.GetOutput()

    mass_properties = vtk.vtkMassProperties()
    mass_properties.SetInputData(polydata)
    return mass_properties.GetVolume()


def get_volume_trimesh(file_path):
    mesh = trimesh.load_mesh(file_path)
    return mesh.volume


volume_pyvista = get_volume_pyvista(OUTPUT_MESH_PATH)
volume_vtk = get_volume_vtk(OUTPUT_MESH_PATH)
volume_trimesh = get_volume_trimesh(OUTPUT_MESH_PATH)

print(f"PyVista volume: {volume_pyvista} cubic units")
print(f"VTK volume: {volume_vtk} cubic units")
print(f"Trimesh volume: {volume_trimesh} cubic units")
