import numpy as np
import open3d as o3d
import cv2
import hl2ss
import hl2ss_3dcv
import hl2ss_io
from pathlib import Path
import time

thispath = Path(__file__).resolve()


# Settings --------------------------------------------------------------------
# HoloLens address
# host = '153.109.130.56'
host = '192.168.1.14'

# Port: RM Depth AHAT or RM Depth Long Throw
port = hl2ss.StreamPort.RM_DEPTH_LONGTHROW

exp_name = 'pointcloud_ita'

# Directory containing the recorded data
path = f'{thispath.parent.parent}/data/{exp_name}'

pcd_folder = Path(f"{path}/pcd_all_v2")
Path(pcd_folder).mkdir(parents=True, exist_ok=True)

# Calibration path (must exist but can be empty)
calibration_path = f'{thispath.parent.parent}/calibration'

# Front RGB camera parameters
pv_width = 640
pv_height = 360
pv_framerate = 30

# Buffer length in seconds
buffer_length = 10

# Maximum depth in meters
max_depth = 3.0

# Voxel size foe downsampling
voxel_size = 0.002

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    return pcd_down

#------------------------------------------------------------------------------
start_time = time.time()
# Get calibration ---------------------------------------------------------
# Get RM Depth Long Throw calibration -------------------------------------
    # Calibration data will be downloaded if it's not in the calibration folder
calibration_lt = hl2ss_3dcv.get_calibration_rm(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, calibration_path)
    
uv2xy = hl2ss_3dcv.compute_uv2xy(calibration_lt.intrinsics, hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT)
xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(uv2xy, calibration_lt.scale)

# Create Open3D visualizer ------------------------------------------------
o3d_lt_intrinsics = o3d.camera.PinholeCameraIntrinsic(hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH,
                                                      hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT, 
                                                      calibration_lt.intrinsics[0, 0], 
                                                      calibration_lt.intrinsics[1, 1], 
                                                      calibration_lt.intrinsics[2, 0], 
                                                      calibration_lt.intrinsics[2, 1])
# vis = o3d.visualization.Visualizer()
# vis.create_window()
pcd = o3d.geometry.PointCloud()
# first_pcd = True

# Create readers --------------------------------------------------------------
rd_pv = hl2ss_io.create_rd(f'{path}/{hl2ss.get_port_name(hl2ss.StreamPort.PERSONAL_VIDEO)}.bin', hl2ss.ChunkSize.SINGLE_TRANSFER, 'bgr24')
rd_depth = hl2ss_io.sequencer(f'{path}/{hl2ss.get_port_name(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)}.bin', hl2ss.ChunkSize.SINGLE_TRANSFER, True)
rd_spatial = hl2ss_io.sequencer(f'{path}/{hl2ss.get_port_name(hl2ss.StreamPort.SPATIAL_INPUT)}.bin', hl2ss.ChunkSize.SINGLE_TRANSFER, True)

# Open readers ----------------------------------------------------------------
rd_pv.open()
rd_depth.open()
rd_spatial.open()

pcds = []
pcd_combined = o3d.geometry.PointCloud()

# Initialize PV intrinsics and extrinsics ---------------------------------
pv_intrinsics = hl2ss.create_pv_intrinsics_placeholder()
pv_extrinsics = np.eye(4, 4, dtype=np.float32)

i = 0
# Main loop -------------------------------------------------------------------
while (True):
    # Get PV frame ------------------------------------------------------------
    data_pv = rd_pv.get_next_packet()
    if (data_pv is None):
        break

    # Find RM VLC frames corresponding to the current PV frame ----------------
    data_depth = rd_depth.get_next_packet(data_pv.timestamp)
    data_spatial = rd_spatial.get_next_packet(data_pv.timestamp)

    if (data_spatial is not None):
        spatial = hl2ss.unpack_si(data_spatial.payload)
        head_pose = spatial.get_head_pose()
        head_position = head_pose.position
        head_forward = head_pose.forward
        head_up = head_pose.up

    if (data_depth is not None):
        depth = hl2ss_3dcv.rm_depth_undistort(data_depth.payload.depth, calibration_lt.undistort_map)
        depth = hl2ss_3dcv.rm_depth_normalize(depth, scale)

        # Display RGBD --------------------------------------------------------
        color = data_pv.payload.image # Depth scaled for visibility
        # cv2.imshow('RGBD', depth)
        # cv2.waitKey(1)

        # Update PV intrinsics ------------------------------------------------
        # PV intrinsics may change between frames due to autofocus
        pv_intrinsics = hl2ss.update_pv_intrinsics(pv_intrinsics, data_pv.payload.focal_length, data_pv.payload.principal_point)
        color_intrinsics, color_extrinsics = hl2ss_3dcv.pv_fix_calibration(pv_intrinsics, pv_extrinsics)
        
        # Generate aligned RGBD image -----------------------------------------
        lt_points         = hl2ss_3dcv.rm_depth_to_points(xy1, depth)
        lt_to_world       = hl2ss_3dcv.camera_to_rignode(calibration_lt.extrinsics) @ hl2ss_3dcv.reference_to_world(data_depth.pose)
        world_to_lt       = hl2ss_3dcv.world_to_reference(data_depth.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_lt.extrinsics)
        world_to_pv_image = hl2ss_3dcv.world_to_reference(data_pv.pose) @ hl2ss_3dcv.rignode_to_camera(color_extrinsics) @ hl2ss_3dcv.camera_to_image(color_intrinsics)
        world_points      = hl2ss_3dcv.transform(lt_points, lt_to_world)
        pv_uv             = hl2ss_3dcv.project(world_points, world_to_pv_image)
        color             = cv2.remap(color, pv_uv[:, :, 0], pv_uv[:, :, 1], cv2.INTER_LINEAR)

        mask_uv = hl2ss_3dcv.slice_to_block((pv_uv[:, :, 0] < 0) | (pv_uv[:, :, 0] >= pv_width) | (pv_uv[:, :, 1] < 0) | (pv_uv[:, :, 1] >= pv_height))
        depth[mask_uv] = 0

        # Convert to Open3D RGBD image and create pointcloud ------------------
        color_image = o3d.geometry.Image(color)
        depth_image = o3d.geometry.Image(depth)

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image, depth_scale=1, depth_trunc=max_depth, convert_rgb_to_intensity=False)

        tmp_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_lt_intrinsics)

        # Display pointcloud --------------------------------------------------
        pcd.points = tmp_pcd.points 
        pcd.colors = tmp_pcd.colors

        pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

        pcd_down = preprocess_point_cloud(pcd, voxel_size)
        o3d.io.write_point_cloud(f"{pcd_folder}/pcd_{i}.ply", pcd_down)

        pcd_combined += pcd_down
        pcds.append(pcd)

        i += 1

    # if (first_pcd):
    #     vis.add_geometry(pcd)
    #     first_pcd = False
    # else:
    #     vis.update_geometry(pcd)

    # vis.poll_events()
    # vis.update_renderer()

    # if (use_ab):
    #     pcd.colors = o3d.utility.Vector3dVector(rgb)
    
o3d.io.write_point_cloud(f"{path}/multiway_registration.ply", pcd_combined)

# Close readers ---------------------------------------------------------------
rd_pv.close()
rd_depth.close()

print(f" Time to complete script: {(time.time()-start_time)}")
