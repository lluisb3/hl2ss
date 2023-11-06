import open3d as o3d
import open3d.visualization.gui as gui
from pathlib import Path
import numpy as np
from plyfile import PlyData


def load_ply(filepath):
    plydata = PlyData.read(filepath)
    data = plydata.elements[0].data
    coords = np.array([data['x'], data['y'], data['z']], dtype=np.float32).T
    feats = np.array([data['red'], data['green'], data['blue']], dtype=np.float32).T

    return coords, feats


thispath = Path(__file__).resolve()

def check_properties(mesh):
    mesh.compute_vertex_normals()

    edge_manifold = mesh.is_edge_manifold(allow_boundary_edges=True)
    edge_manifold_boundary = mesh.is_edge_manifold(allow_boundary_edges=False)
    vertex_manifold = mesh.is_vertex_manifold()
    self_intersecting = mesh.is_self_intersecting()
    watertight = mesh.is_watertight()
    orientable = mesh.is_orientable()

    print(f"  edge_manifold:          {edge_manifold}")
    print(f"  edge_manifold_boundary: {edge_manifold_boundary}")
    print(f"  vertex_manifold:        {vertex_manifold}")
    print(f"  self_intersecting:      {self_intersecting}")
    print(f"  watertight:             {watertight}")
    print(f"  orientable:             {orientable}")

filename = "mesh_scene_v1.ply"
path = f"{thispath.parent.parent}/data/try"

ply_path = f"{path}/{filename}"

coords, feats = load_ply(ply_path)

print(coords)
print(feats)
print("")

mesh = o3d.io.read_triangle_mesh(ply_path)

print(mesh)
print("")
print(np.asarray(mesh.vertices))
print("")
print(np.asarray(mesh.triangles))
print("")
print(np.asarray(mesh.vertex_colors))
print("")
# check_properties(mesh)

# for vertex in mesh.vertices:
#     print(vertex)

print("Try to render a mesh with normals (exist: " +
          str(mesh.has_vertex_normals()) + ") and colors (exist: " +
          str(mesh.has_vertex_colors()) + ")")
