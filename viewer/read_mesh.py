import open3d as o3d
import open3d.visualization.gui as gui
from pathlib import Path
import numpy as np
from plyfile import PlyData, PlyProperty
import click
from ply_double_to_float import ply_double_to_float
import pandas as pd

thispath = Path(__file__).resolve()


def load_ply(filepath):
    plydata = PlyData.read(filepath)
    data = plydata.elements[0].data
    xyzrgb = np.array([data['x'], data['y'], data['z'], data['red'], data['green'], data['blue']], dtype=np.float32)

    return xyzrgb

@click.command()
@click.option(
    "--mesh_path",
    default="/home/ither1/hl2ss/data/scene_matthieu/scene_matthieu_mesh.ply",
    prompt="Path to .ply file with mesh",
    help="Path to .ply file with mesh",
)
@click.option(
    "--pcd_path",
    default="/home/ither1/hl2ss/data/scene_matthieu/scene_matthieu_pcd.ply",
    prompt="Path to .ply file with pointcloud",
    help="Path to .ply file with pointcloud",
)
def main(mesh_path, pcd_path):

    # Initialize visualizer
    # app = gui.Application.instance
    # app.initialize()
    # vis = o3d.visualization.O3DVisualizer("Scene Hes·so", 1024, 768)
    # vis.show_settings = True

    output_path = Path(pcd_path).parent
    header = ["x", "y", "z", "r", "g", "b"]

    xyzrgb_mesh = load_ply(mesh_path).T
    mesh_df = pd.DataFrame(xyzrgb_mesh, columns=header)
    mesh_df.to_csv(f"{output_path}/xyzrgb_mesh.csv")

    xyzrgb_pcd = load_ply(pcd_path).T
    pcd_df = pd.DataFrame(xyzrgb_pcd, columns=header)
    pcd_df.to_csv(f"{output_path}/xyzrgb_pcd.csv")

    # for coord_mesh in coords_mesh:


    # print(coords_mesh.shape)
    # print(feats_mesh.shape)
    # print(coords_pcd.shape)
    # print(feats_pcd.shape)

    # mesh = o3d.io.read_triangle_mesh(mesh_path)
    # pcd = o3d.io.read_point_cloud(pcd_path)

    # R = mesh.get_rotation_matrix_from_xyz(((np.pi / 2), 0, 0))
    # print(R)
    # mesh.rotate(R, center=(0, 0, 0))

    # # o3d.io.write_point_cloud(pcd_path, pcd)

    # o3d.io.write_triangle_mesh(mesh_path, mesh)
    # ply_double_to_float(mesh_path)

    # # Load scene visualization    
    # vis.add_geometry(f"Scene", mesh)
    # vis.reset_camera_to_default()

    # app.add_window(vis)
    # app.run()



if __name__=="__main__":
    main()
