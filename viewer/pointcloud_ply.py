from pynput import keyboard

import multiprocessing as mp
import numpy as np
import open3d as o3d
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_mp
import hl2ss_3dcv
import hl2ss_io
from pathlib import Path

thispath = Path(__file__).resolve()


# Settings --------------------------------------------------------------------
# HoloLens address
# host = '153.109.130.56'
host = '192.168.1.14'

# Port: RM Depth AHAT or RM Depth Long Throw
port = hl2ss.StreamPort.RM_DEPTH_LONGTHROW

exp_name = 'pointcloud'

# Directory containing the recorded data
path = f'{thispath.parent.parent}/data/{exp_name}'

# Calibration path (must exist but can be empty)
calibration_path = '/home/ither1/hl2ss/calibration'

# Front RGB camera parameters
pv_width = 640
pv_height = 360
pv_framerate = 30

# Buffer length in seconds
buffer_length = 10

# Maximum depth in meters
max_depth = 3.0

# Voxel size foe downsampling
voxel_size = 0.02

def pairwise_registration(source, target, max_correspondence_distance_coarse, max_correspondence_distance_fine):
    print("Apply point-to-plane ICP")
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp


def full_registration(pcds, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id], max_correspondence_distance_coarse, max_correspondence_distance_fine)
            print("Build o3d.pipelines.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(
                        np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=True))
    return pose_graph

#------------------------------------------------------------------------------

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
rd_lf = hl2ss_io.sequencer(f'{path}/{hl2ss.get_port_name(hl2ss.StreamPort.RM_VLC_LEFTFRONT)}.bin', hl2ss.ChunkSize.SINGLE_TRANSFER, True)
rd_rf = hl2ss_io.sequencer(f'{path}/{hl2ss.get_port_name(hl2ss.StreamPort.RM_VLC_RIGHTFRONT)}.bin', hl2ss.ChunkSize.SINGLE_TRANSFER, True)
rd_depth = hl2ss_io.sequencer(f'{path}/{hl2ss.get_port_name(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)}.bin', hl2ss.ChunkSize.SINGLE_TRANSFER, True)
# rd_depth = hl2ss_io.create_rd(f'{path}/{hl2ss.get_port_name(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)}.bin', hl2ss.ChunkSize.SINGLE_TRANSFER, 'bgr24')

# Open readers ----------------------------------------------------------------
rd_pv.open()
rd_lf.open()
rd_rf.open()
rd_depth.open()

pcds = []
pcd_combined = o3d.geometry.PointCloud()

# Initialize PV intrinsics and extrinsics ---------------------------------
pv_intrinsics = hl2ss.create_pv_intrinsics_placeholder()
pv_extrinsics = np.eye(4, 4, dtype=np.float32)

# Main loop -------------------------------------------------------------------
while (True):
    # Get PV frame ------------------------------------------------------------
    data_pv = rd_pv.get_next_packet()
    if (data_pv is None):
        break

    # Find RM VLC frames corresponding to the current PV frame ----------------
    data_lf = rd_lf.get_next_packet(data_pv.timestamp) # Get nearest (in time) lf frame
    data_rf = rd_rf.get_next_packet(data_pv.timestamp) # Get nearest (in time) rf frame
    data_depth = rd_depth.get_next_packet(data_pv.timestamp)

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

        # # Display pointcloud --------------------------------------------------
        # xyz = hl2ss_3dcv.rm_depth_to_points(depth, xy1)
        # xyz = hl2ss_3dcv.block_to_list(xyz)
        # rgb = hl2ss_3dcv.block_to_list(ab)
        # d = hl2ss_3dcv.block_to_list(depth).reshape((-1,))
        # xyz = xyz[d > 0, :]
        # rgb = rgb[d > 0, :]
        # rgb = np.hstack((rgb, rgb, rgb))

        # pcd.points = o3d.utility.Vector3dVector(xyz)

        pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

        pcd.estimate_normals()

        # pcd_combined += pcd
        pcds.append(pcd)

    # if (first_pcd):
    #     vis.add_geometry(pcd)
    #     first_pcd = False
    # else:
    #     vis.update_geometry(pcd)

    # vis.poll_events()
    # vis.update_renderer()

    # if (use_ab):
    #     pcd.colors = o3d.utility.Vector3dVector(rgb)

    # if (first_pcd):
    #     vis.add_geometry(pcd)
    #     first_pcd = False
    # else:
    #     vis.update_geometry(pcd)

    # vis.poll_events()
    # vis.update_renderer()

    # pcd.points = o3d.utility.Vector3dVector(points)
    # pcd.colors = o3d.utility.Vector3dVector(colors)


    # image_o3d = o3d.geometry.Image(image)
    # print(image_o3d)
    # depth_o3d = o3d.geometry.Depth(depth)
    # rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(image_o3d, depth_o3d)

    
print(len(pcds))
print("Full registration ...")
max_correspondence_distance_coarse = voxel_size * 15
max_correspondence_distance_fine = voxel_size * 1.5
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    pose_graph = full_registration(pcds,
                                   max_correspondence_distance_coarse,
                                   max_correspondence_distance_fine)
print("Optimizing PoseGraph ...")
option = o3d.pipelines.registration.GlobalOptimizationOption(
    max_correspondence_distance=max_correspondence_distance_fine,
    edge_prune_threshold=0.25,
    reference_node=0)
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option)
    

for point_id in range(len(pcds)):
    pcds[point_id].transform(pose_graph.nodes[point_id].pose)
    pcd_combined += pcds[point_id]
# pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
o3d.io.write_point_cloud(f"{path}/multiway_registration.ply", pcd_combined)


# Close readers ---------------------------------------------------------------
rd_pv.close()
rd_lf.close()
rd_rf.close()
rd_depth.close()