import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os
from pathlib import Path

scene = "mask3d/scene_meeting_6"

output_path = Path(f"output_video/{scene}")
Path(output_path).mkdir(parents=True, exist_ok=True)

# Load .ply file
pcd = o3d.io.read_point_cloud(f"data/{scene}/scene_segmented_pcd.ply")
R = pcd.get_rotation_matrix_from_xyz((-np.pi / 2, 0, -np.pi / 2))
pcd.rotate(R, center=(0, 0, 0))

# Create a window for visualization
vis = o3d.visualization.Visualizer()
vis.create_window()

vis.add_geometry(pcd)

# Get view control object to control the camera
view_ctl = vis.get_view_control()

# Set the focus point (look-at point) to the center of the point cloud
pcd_center = pcd.get_center()
view_ctl.set_lookat(pcd_center)

# Set the front direction
view_ctl.set_front([0, 0, 1])  # Set the initial viewing direction


# Get the current camera parameters
cam_params = view_ctl.convert_to_pinhole_camera_parameters()

# Modify the extrinsic matrix to move the camera up
# For example, raise the camera to a higher Y-position
extrinsic = np.array(cam_params.extrinsic)

# Move the camera along the Y-axis
extrinsic[1, 3] += 0.0

# Modify the extrinsic matrix for a tilt (tilting 30 degrees downward)
R_tilt = np.array([[1, 0, 0],  # Identity rotation around X-axis
                   [0, np.cos(np.radians(50)), -np.sin(np.radians(50))],  # 30 degree rotation down on X-axis
                   [0, np.sin(np.radians(50)), np.cos(np.radians(50))]])  # 30 degree tilt

# Apply this rotation to the extrinsic matrix
extrinsic[:3, :3] = R_tilt.dot(extrinsic[:3, :3])

# Apply the modified camera parameters
cam_params.extrinsic = extrinsic
view_ctl.convert_from_pinhole_camera_parameters(cam_params, allow_arbitrary=True)

view_ctl.set_zoom(0.4)

# Rotate and capture frames
for i in range(360):  # 360 degrees rotation
    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()

    # Rotate the object around the Y-axis
    rotation_angle = np.radians(1)  # Rotation angle in radians
    rotation_matrix = pcd.get_rotation_matrix_from_axis_angle([0, rotation_angle, 0])  # Rotate around Y-axis
    pcd.rotate(rotation_matrix, center=pcd_center)  # Rotate the point cloud)
    
    vis.capture_screen_image(f"{output_path}/frame_{i:03d}.png")

vis.destroy_window()

# # Use FFmpeg to convert images to video
os.system(f"ffmpeg -framerate 30 -i {output_path}/frame_%03d.png -c:v libx264 -pix_fmt yuv420p {output_path}/rotating_scene_video.mp4")