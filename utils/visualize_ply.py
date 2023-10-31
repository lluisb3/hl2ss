import open3d as o3d
import open3d.visualization.gui as gui
from pathlib import Path

thispath = Path(__file__).resolve()


filename = "multiway_registration.ply"
exp_name = "pointcloud_ita"

path = f"{thispath.parent.parent}/data/{exp_name}"

ply_path = f"{path}/{filename}"
scene_mask = o3d.io.read_point_cloud(ply_path)

# Initialize visualizer
app = gui.Application.instance
app.initialize()

vis = o3d.visualization.O3DVisualizer("Scene Hes·so", 1024, 768)
vis.show_settings = True

# Load scene visualization    
vis.add_geometry(f"Scene", scene_mask)
vis.reset_camera_to_default()

app.add_window(vis)
app.run()