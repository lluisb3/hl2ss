import open3d as o3d
import open3d.visualization.gui as gui
from pathlib import Path
import numpy as np
from plyfile import PlyData, PlyProperty
import click

thispath = Path(__file__).resolve()


def load_ply(filepath):
    plydata = PlyData.read(filepath)
    data = plydata.elements[0].data
    coords = np.array([data['x'], data['y'], data['z']], dtype=np.float32)
    feats = np.array([data['red'], data['green'], data['blue']], dtype=np.float32)

    return coords, feats


@click.command()
@click.option(
    "--mesh_path",
    default="/home/ither1/hl2ss/data/office/mesh_office.ply",
    prompt="Path to .ply file with mesh",
    help="Path to .ply file with mesh",
)
@click.option(
    "--pcd_path",
    default="/home/ither1/hl2ss/data/scene_office_002_2/pcd_scene_office_002_2.ply",
    prompt="Path to .ply file with pointcloud",
    help="Path to .ply file with pointcloud",
)
def main(mesh_path, pcd_path):

    # Initialize visualizer
    # app = gui.Application.instance
    # app.initialize()
    # vis = o3d.visualization.O3DVisualizer("Scene Hes·so", 1024, 768)
    # vis.show_settings = True

    # coords_mesh, feats_mesh = load_ply(mesh_path)

    # coords_pcd, feats_pcd = load_ply(pcd_path)

    # print(coords_mesh.shape)
    # print(feats_mesh.shape)
    # print(coords_pcd.shape)
    # print(feats_pcd.shape)

    # mesh = o3d.io.read_triangle_mesh(mesh_path)
    pcd = o3d.io.read_point_cloud(pcd_path)

    R = pcd.get_rotation_matrix_from_xyz(((np.pi / 2), 0, 0))
    print(R)
    pcd.rotate(R, center=(0, 0, 0))

    o3d.io.write_point_cloud(pcd_path, pcd)

    # # Load scene visualization    
    # vis.add_geometry(f"Scene", mesh)
    # vis.reset_camera_to_default()

    # app.add_window(vis)
    # app.run()



if __name__=="__main__":
    main()
