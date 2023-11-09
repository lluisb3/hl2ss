#------------------------------------------------------------------------------
# This script saves a pointcloud (pcd) of a scene in .ply format
# aligning RGBD and Spatial Mapping data.
# Press space to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import multiprocessing as mp
import open3d as o3d
import cv2
import hl2ss
import hl2ss_lnm
import hl2ss_mp
import hl2ss_3dcv
import hl2ss_sa
from pathlib import Path


thispath = Path(__file__).resolve()


# Settings --------------------------------------------------------------------
# HoloLens address
host = '153.109.130.78'

exp_name = "scene_meeting_001_2"

# Calibration path (must exist but can be empty)
calibration_path = f'{thispath.parent.parent}/calibration'

# Camera parameters
pv_width = 640
pv_height = 360
pv_framerate = 30

# Buffer length in seconds
buffer_size = 10

# Integrator parameters
max_depth = 2.0
voxel_size = 0.01
block_resolution = 8
block_count = 100000
device = 'cpu:0'
weight_threshold = 0.5

# Spatial Mapping manager parameters
tpcm = 1000
threads = 2
origin = [0, 0, 0]
radius = 0.5

#------------------------------------------------------------------------------
def on_press(key):
        global enable
        enable = key != keyboard.Key.space
        return enable

# Output folder
output_path = f"{thispath.parent.parent}/data/{exp_name}"
Path(output_path).mkdir(parents=True, exist_ok=True)

# Keyboard events ---------------------------------------------------------
if __name__ == '__main__':
    enable = True

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # Create Open3D visualizer ------------------------------------------------
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.get_render_option().mesh_show_back_face = True

    first_pcd = True

    # Start PV subsystem ------------------------------------------------------
    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Get calibration ---------------------------------------------------------
    calibration_lt = hl2ss_3dcv.get_calibration_rm(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, calibration_path)
    
    uv2xy = hl2ss_3dcv.compute_uv2xy(calibration_lt.intrinsics, hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT)
    xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(uv2xy, calibration_lt.scale)

    # Get SM data -------------------------------------------------------------
    sm_volume = hl2ss.sm_bounding_volume()
    sm_volume.add_sphere(origin, radius)

    sm_manager = hl2ss_sa.sm_manager(host, tpcm, threads)
    sm_manager.open()
    sm_manager.set_volumes(sm_volume)
    sm_manager.get_observed_surfaces()
    sm_manager.close()
    meshes = sm_manager.get_meshes()

    meshes = [hl2ss_sa.sm_mesh_to_open3d_triangle_mesh(mesh) for mesh in meshes]
    for mesh in meshes:
        mesh.compute_vertex_normals()
        vis.add_geometry(mesh)

    # Start streams -----------------------------------------------------------
    producer = hl2ss_mp.producer()
    producer.configure(hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=pv_width, height=pv_height, framerate=pv_framerate, decoded_format='rgb24'))
    producer.configure(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW))
    producer.initialize(hl2ss.StreamPort.PERSONAL_VIDEO, buffer_size * pv_framerate)
    producer.initialize(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, buffer_size * hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS)    
    producer.start(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.start(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

    consumer = hl2ss_mp.consumer()
    manager = mp.Manager()
    sink_pv = consumer.create_sink(producer, hl2ss.StreamPort.PERSONAL_VIDEO, manager, None)
    sink_lt = consumer.create_sink(producer, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, manager, ...)
    sink_pv.get_attach_response()
    sink_lt.get_attach_response()

    # Create integrator -------------------------------------------------------
    integrator = hl2ss_sa.integrator(voxel_size, block_resolution, block_count, device)
    integrator.set_intrinsics(calibration_lt.intrinsics[:3, :3])
    integrator.set_depth_parameters(1.0, max_depth)

    pv_intrinsics = hl2ss.create_pv_intrinsics_placeholder()
    pv_extrinsics = np.eye(4, 4, dtype=np.float32)

    # Main loop ---------------------------------------------------------------
    while (enable):
        # Get frames ----------------------------------------------------------
        sink_lt.acquire()

        _, data_lt = sink_lt.get_most_recent_frame()
        if ((data_lt is None) or (not hl2ss.is_valid_pose(data_lt.pose))):
            continue
        _, data_pv = sink_pv.get_nearest(data_lt.timestamp)
        if ((data_pv is None) or (not hl2ss.is_valid_pose(data_pv.pose))):
            continue

        # Integrate -----------------------------------------------------------
        depth = hl2ss_3dcv.rm_depth_undistort(data_lt.payload.depth, calibration_lt.undistort_map)
        depth = hl2ss_3dcv.rm_depth_normalize(depth, scale)
        color = data_pv.payload.image

        pv_intrinsics = hl2ss.update_pv_intrinsics(pv_intrinsics, data_pv.payload.focal_length, data_pv.payload.principal_point)
        color_intrinsics, color_extrinsics = hl2ss_3dcv.pv_fix_calibration(pv_intrinsics, pv_extrinsics)
        
        lt_points         = hl2ss_3dcv.rm_depth_to_points(xy1, depth)
        lt_to_world       = hl2ss_3dcv.camera_to_rignode(calibration_lt.extrinsics) @ hl2ss_3dcv.reference_to_world(data_lt.pose)
        world_to_lt       = hl2ss_3dcv.world_to_reference(data_lt.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_lt.extrinsics)
        world_to_pv_image = hl2ss_3dcv.world_to_reference(data_pv.pose) @ hl2ss_3dcv.rignode_to_camera(color_extrinsics) @ hl2ss_3dcv.camera_to_image(color_intrinsics)
        world_points      = hl2ss_3dcv.transform(lt_points, lt_to_world)
        pv_uv             = hl2ss_3dcv.project(world_points, world_to_pv_image)
        color             = cv2.remap(color, pv_uv[:, :, 0], pv_uv[:, :, 1], cv2.INTER_LINEAR).astype(np.float32) #/ hl2ss._RANGEOF.U8_MAX
        mask_uv = hl2ss_3dcv.slice_to_block((pv_uv[:, :, 0] < 0) | (pv_uv[:, :, 0] >= pv_width) | (pv_uv[:, :, 1] < 0) | (pv_uv[:, :, 1] >= pv_height))
        depth[mask_uv] = 0

        integrator.set_extrinsics(world_to_lt)
        integrator.set_projection(world_to_lt @ calibration_lt.intrinsics)
        integrator.set_depth(depth)
        integrator.set_color(color)

        try:
            integrator.update()
        except:
            pass

        pcd_tmp = integrator.extract_point_cloud(weight_threshold).to_legacy()

        # Update visualization ------------------------------------------------
        if (first_pcd):
            first_pcd = False
            pcd = pcd_tmp
            vis.add_geometry(pcd)
        else:
            pcd.points = pcd_tmp.points
            pcd.colors = pcd_tmp.colors

            vis.update_geometry(pcd)

        vis.poll_events()
        vis.update_renderer()

    # Stop streams ------------------------------------------------------------
    sink_pv.detach()
    sink_lt.detach()    
    producer.stop(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.stop(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

    # Stop PV subsystem -------------------------------------------------------
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Stop keyboard events ----------------------------------------------------
    listener.join()

    # xyz = np.asarray(pcd.points)
    # rgb = np.asarray(pcd.colors)

    # new_pcd = o3d.geometry.PointCloud()
    # new_pcd.points = o3d.utility.Vector3dVector(xyz)
    # new_pcd.colors = o3d.utility.Vector3dVector(rgb)

    pcd.colors = o3d.utility.Vector3dVector(np.clip(np.asarray(pcd.colors), 0, 1))
    pcd.estimate_normals()
    with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=9)

    mesh.vertex_colors = o3d.utility.Vector3dVector(np.clip(
        np.asarray(mesh.vertex_colors), 0, 1))
    o3d.io.write_point_cloud(f"{output_path}/pcd_{exp_name}.ply", pcd)
    o3d.io.write_triangle_mesh(f"{output_path}/mesh_{exp_name}.ply", mesh)
    # ply_data = PlyData.read(f"{output_path}/mesh_{exp_name}.ply")
    # ply_data.write(f"{output_path}/mesh_{exp_name}_v2.ply")
    print(f"Pcd and mesh saved in {output_path}")

    # Show final point cloud --------------------------------------------------
    vis.run()