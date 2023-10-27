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
import k4a

thispath = Path(__file__).resolve()


# Settings --------------------------------------------------------------------
# HoloLens address
host = '153.109.130.56'

# Port: RM Depth AHAT or RM Depth Long Throw
port = hl2ss.StreamPort.RM_DEPTH_LONGTHROW

exp_name = 'pointcloud'

# Directory containing the recorded data
path = f'{thispath.parent.parent}/data/{exp_name}'

# Calibration path (must exist but can be empty)
calibration_path = '/home/ither1/hl2ss/calibration'

# Use AB data to color the pointcloud
use_ab = False

# AHAT Profile
ht_profile_z = hl2ss.DepthProfile.SAME
ht_profile_ab = hl2ss.VideoProfile.H265_MAIN

# Buffer length in seconds
buffer_length = 10

voxel_size = 0.02

def pairwise_registration(source, target):
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
                pcds[source_id], pcds[target_id])
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
# Calibration data will be downloaded if it's not in the calibration folder
calibration = hl2ss_3dcv.get_calibration_rm(host, port, calibration_path)
xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(calibration.uv2xy, calibration.scale)
max_depth = 8.0 if (port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW) else (calibration.alias / calibration.scale)
fps = hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS if (port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW) else hl2ss.Parameters_RM_DEPTH_AHAT.FPS

# Create Open3D visualizer ------------------------------------------------
vis = o3d.visualization.Visualizer()
vis.create_window()
pcd = o3d.geometry.PointCloud()
first_pcd = True

# Create readers --------------------------------------------------------------
rd_pv = hl2ss_io.create_rd(f'{path}/{hl2ss.get_port_name(hl2ss.StreamPort.PERSONAL_VIDEO)}.bin', hl2ss.ChunkSize.SINGLE_TRANSFER, 'bgr24')
rd_lf = hl2ss_io.sequencer(f'{path}/{hl2ss.get_port_name(hl2ss.StreamPort.RM_VLC_LEFTFRONT)}.bin', hl2ss.ChunkSize.SINGLE_TRANSFER, True)
rd_rf = hl2ss_io.sequencer(f'{path}/{hl2ss.get_port_name(hl2ss.StreamPort.RM_VLC_RIGHTFRONT)}.bin', hl2ss.ChunkSize.SINGLE_TRANSFER, True)
rd_depth = hl2ss_io.sequencer(f'{path}/{hl2ss.get_port_name(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)}.bin', hl2ss.ChunkSize.SINGLE_TRANSFER, True)

# Open readers ----------------------------------------------------------------
rd_pv.open()
rd_lf.open()
rd_rf.open()
rd_depth.open()

pcds = []
pcd_combined = o3d.geometry.PointCloud()
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
        depth = hl2ss_3dcv.rm_depth_normalize(data_depth.payload.depth, scale)
        ab = hl2ss_3dcv.slice_to_block(data_depth.payload.ab) / 65536

    # Display RGBD --------------------------------------------------------
    image = data_pv.payload.image # Depth scaled for visibility
    # cv2.imshow('RGBD', depth)
    # cv2.waitKey(1)

    # Display pointcloud --------------------------------------------------
    # xyz = hl2ss_3dcv.rm_depth_to_points(depth, xy1)
    # xyz = hl2ss_3dcv.block_to_list(xyz)
    # rgb = hl2ss_3dcv.block_to_list(ab)













    # d = hl2ss_3dcv.block_to_list(depth).reshape((-1,))
    # xyz = xyz[d > 0, :]
    # rgb = rgb[d > 0, :]
    # rgb = np.hstack((rgb, rgb, rgb))

    print(depth.shape)
    print(image.shape)
    print(scale)
    # Create a point cloud
    h, w, _ = depth.shape
    v, u = np.mgrid[0:h, 0:w]
    points = np.dstack((u, v, depth))
    points = points.reshape(-1, 3)
    
    # Add color information
    colors = image.reshape(-1, 3)
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)


    # image_o3d = o3d.geometry.Image(image)
    # print(image_o3d)
    # depth_o3d = o3d.geometry.Depth(depth)
    # rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(image_o3d, depth_o3d)

    # pcd.points = o3d.utility.Vector3dVector(xyz)

    # pcds.append(pcd)
    # if (use_ab):
    #     pcd.colors = o3d.utility.Vector3dVector(rgb)

    # if (first_pcd):
    #     vis.add_geometry(pcd)
    #     first_pcd = False
    # else:
    #     vis.update_geometry(pcd)

    # vis.poll_events()
    # vis.update_renderer()
    print(path)
    device_depth = k4a.Device.open_from_path(f"{path}/rm_depth_longthrow.bin")
    device_color = k4a.Device.open_from_path(f"{path}/personal_video.bin")

    calibration_depth = device_depth.get_calibration()
    depth_data = device_depth.get_next_depth_image()
    point_cloud = k4a.image_to_point_cloud(depth_data, calibration_depth)
    point_cloud_xyx = point_cloud.xyz

    pcd.points = o3d.utility.Vector3dVector(point_cloud_xyx)

    if (first_pcd):
        vis.add_geometry(pcd)
        first_pcd = False
    else:
        vis.update_geometry(pcd)

    vis.poll_events()
    vis.update_renderer()



# print("Full registration ...")
# max_correspondence_distance_coarse = voxel_size * 15
# max_correspondence_distance_fine = voxel_size * 1.5
# with o3d.utility.VerbosityContextManager(
#         o3d.utility.VerbosityLevel.Debug) as cm:
#     pose_graph = full_registration(pcds,
#                                    max_correspondence_distance_coarse,
#                                    max_correspondence_distance_fine)
    


# print("Optimizing PoseGraph ...")
# option = o3d.pipelines.registration.GlobalOptimizationOption(
#     max_correspondence_distance=max_correspondence_distance_fine,
#     edge_prune_threshold=0.25,
#     reference_node=0)
# with o3d.utility.VerbosityContextManager(
#         o3d.utility.VerbosityLevel.Debug) as cm:
#     o3d.pipelines.registration.global_optimization(
#         pose_graph,
#         o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
#         o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
#         option)
    

# pcd_combined = o3d.geometry.PointCloud()
# for point_id in range(len(pcds)):
#     pcds[point_id].transform(pose_graph.nodes[point_id].pose)
#     pcd_combined += pcds[point_id]
# pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
# o3d.io.write_point_cloud("multiway_registration.pcd", pcd_combined_down)


# Close readers ---------------------------------------------------------------
rd_pv.close()
rd_lf.close()
rd_rf.close()
rd_depth.close()