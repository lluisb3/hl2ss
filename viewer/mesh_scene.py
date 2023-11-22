#------------------------------------------------------------------------------
# This script downloads Spatial Mapping data from the HoloLens and displays it.
#------------------------------------------------------------------------------

import open3d as o3d
import hl2ss
import hl2ss_lnm
import hl2ss_3dcv
import hl2ss_sa
import numpy as np
from pathlib import Path


thispath = Path(__file__).resolve()

# Settings --------------------------------------------------------------------

# HoloLens address
host = '153.109.130.67'

exp_name = "meeting_100000"

# Output folder
output_path = f"{thispath.parent.parent}/data/{exp_name}"
Path(output_path).mkdir(parents=True, exist_ok=True)

# Maximum triangles per cubic meter
tpcm = 100000

# Data format
vpf = hl2ss.SM_VertexPositionFormat.R32G32B32A32Float
tif = hl2ss.SM_TriangleIndexFormat.R32Uint
vnf = hl2ss.SM_VertexNormalFormat.R32G32B32A32Float

# Include normals
normals = True

# include bounds
bounds = False

# Maximum number of active threads (on the HoloLens) to compute meshes
threads = 2

# Region of 3D space to sample (bounding box)
# All units are in meters
center  = [0.0, 0.0, 0.0] # Position of the box
extents = [3.0, 3.0, 3.0] # Dimensions of the box

#------------------------------------------------------------------------------

# Download meshes -------------------------------------------------------------
# See
# https://learn.microsoft.com/en-us/windows/mixed-reality/develop/native/spatial-mapping-in-directx
# for details

client = hl2ss_lnm.ipc_sm(host, hl2ss.IPCPort.SPATIAL_MAPPING)

client.open()

client.create_observer()

volumes = hl2ss.sm_bounding_volume()
volumes.add_box(center, extents)
client.set_volumes(volumes)

surface_infos = client.get_observed_surfaces()
tasks = hl2ss.sm_mesh_task()
for surface_info in surface_infos:
    tasks.add_task(surface_info.id, tpcm, vpf, tif, vnf, normals, bounds)

meshes = client.get_meshes(tasks, threads)

client.close()

print(f'Observed {len(surface_infos)} surfaces')

# Display meshes --------------------------------------------------------------

open3d_meshes = []
first_mesh = True

for index, mesh in meshes.items():
    id_hex = surface_infos[index].id.hex()
    timestamp = surface_infos[index].update_time

    if (mesh is None):
        print(f'Task {index}: surface id {id_hex} compute mesh failed')
        continue

    mesh.unpack(vpf, tif, vnf)

    # Surface timestamps are given in Windows FILETIME (utc)
    print(f'Task {index}: surface id {id_hex} @ {timestamp} has {mesh.vertex_positions.shape[0]} vertices {mesh.triangle_indices.shape[0]} triangles {mesh.vertex_normals.shape[0]} normals')

    hl2ss_3dcv.sm_mesh_normalize(mesh)

    open3d_mesh = hl2ss_sa.sm_mesh_to_open3d_triangle_mesh(mesh)
    open3d_mesh.compute_vertex_normals()
    open3d_mesh.vertex_colors = open3d_mesh.vertex_normals
    open3d_meshes.append(open3d_mesh)

    open3d_mesh.vertex_colors = o3d.utility.Vector3dVector(np.clip(
        np.asarray(open3d_mesh.vertex_colors), 0, 1))

    if (first_mesh):
        first_mesh = False
        saved_mesh = open3d_mesh
    else:
        saved_mesh += open3d_mesh

o3d.io.write_triangle_mesh(f"{output_path}/mesh_{exp_name}.ply", saved_mesh)
print(f"Mesh saved in {output_path}")

o3d.visualization.draw_geometries([saved_mesh], mesh_show_back_face=True)
