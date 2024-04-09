import open3d as o3d
import open3d.visualization.gui as gui
from plyfile import PlyData
import numpy as np

mesh_path = "/home/lluis/hl2ss/data/hololens_docker_v3/complete_scene/hololens_docker_v3_pcd2mesh_ball_piv.obj"
pcd_path = "/home/lluis/docker_volume/hl2ss/scene_meeting_2/scene_meeting_2_pcd.ply"
chair1_path = "/home/lluis/hl2ss/data/hololens_docker_v3/chair_0.9890719056129456.obj"
chair2_path = "/home/lluis/hl2ss/data/hololens_docker_v3/chair_0.9887288808822632.obj"
desk_path = "/home/lluis/hl2ss/data/hololens_docker_v3/desk_0.8694808483123779.obj"

if __name__ == "__main__":

    scene_mesh = o3d.io.read_triangle_mesh(mesh_path)
    chair1_mesh = o3d.io.read_triangle_mesh(chair1_path)
    chair1_mesh.paint_uniform_color([0, 1, 0])
    chair2_mesh = o3d.io.read_triangle_mesh(chair2_path)
    chair2_mesh.paint_uniform_color([0, 1, 0])
    desk_mesh = o3d.io.read_triangle_mesh(desk_path)
    desk_mesh.paint_uniform_color([0, 0, 1])

    axis_aligned_bounding_box_chair1 = chair1_mesh.get_axis_aligned_bounding_box()
    axis_aligned_bounding_box_chair2 = chair2_mesh.get_axis_aligned_bounding_box()
    axis_aligned_bounding_box_desk = desk_mesh.get_axis_aligned_bounding_box()
    
    axis_aligned_bounding_box_chair1.color = (1, 0, 0)
    axis_aligned_bounding_box_chair2.color = (1, 0, 0)
    axis_aligned_bounding_box_desk.color = (1, 0, 0)


    center = np.asarray([chair1_mesh.get_center()])

    center_pcd = o3d.geometry.PointCloud()
    center_pcd.points = o3d.utility.Vector3dVector(center)
    # Initialize visualizer
    app = gui.Application.instance
    app.initialize()

    vis = o3d.visualization.O3DVisualizer("Scene Hes·so", 1024, 768)
    vis.show_settings = True

    # Load scene visualization    
    vis.add_geometry(f"Scene", scene_mesh)
    vis.add_geometry(f"Scene_chair1", chair1_mesh)
    vis.add_geometry(f"Scene_ac1", axis_aligned_bounding_box_chair1)
    vis.add_geometry(f"Scene_chair2", chair2_mesh)
    vis.add_geometry(f"Scene_ac2", axis_aligned_bounding_box_chair2)
    vis.add_geometry(f"Scene_desk", desk_mesh)
    vis.add_geometry(f"Scene_ad", axis_aligned_bounding_box_desk)
    vis.add_geometry(f"Scene_center", center_pcd)
    # vis.add_geometry(f"Scene_3", center_pcd)


    vis.reset_camera_to_default()

    app.add_window(vis)
    app.run()