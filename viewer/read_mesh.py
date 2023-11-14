import open3d as o3d
import open3d.visualization.gui as gui
from pathlib import Path
import numpy as np
from plyfile import PlyData, PlyProperty
import open3d.core as o3c
import click

thispath = Path(__file__).resolve()


def load_ply(filepath):
    plydata = PlyData.read(filepath)
    data = plydata.elements[0].data
    coords = np.array([data['x'], data['y'], data['z']], dtype=np.float32).T
    feats = np.array([data['red'], data['green'], data['blue']], dtype=np.float32).T

    return coords, feats


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


@click.command()
@click.option(
    "--experiment_name",
    default="scene",
    prompt="Name of the scene to visualize",
    help="Name of the scene to visualize",
)
def main(experiment_name):
    filename = f"mesh_{experiment_name}.ply"
    path = f"{thispath.parent.parent}/data/{experiment_name}"

    ply_path = f"{path}/{filename}"

    coords, feats = load_ply(ply_path)

    print(coords)
    print(feats)
    print("")

    # mesh = o3d.io.read_triangle_mesh(ply_path)

    # points=np.asarray(mesh.vertices)
    # dtype = o3d.core.float32
    # p_tensor = o3d.core.Tensor(points, dtype=dtype)
    # pc = o3d.t.geometry.PointCloud(p_tensor)
    # # o3d.t.io.write_point_cloud("data/float32.ply", pc)

    # # pc.estimate_normals()
    # with o3d.utility.VerbosityContextManager(
    #     o3d.utility.VerbosityLevel.Debug) as cm:
    #     mesh, densities = o3d.t.geometry.TriangleMesh.create_from_point_cloud_poisson(
    #     pc, depth=9)
    # o3d.t.io.write_triangle_mesh("data/mesh_float32.ply", mesh)


    #Read the pointcloud
    plydata = PlyData.read(ply_path)

    #go through all property one by one and if it is a double, we change it to an equivalent property in float
    real_properties = []
    for i in plydata.elements[0].properties:
        if str(PlyProperty(i.name, "double")) == str(i):
            real_properties.append(PlyProperty(i.name, "float32"))
        else:
            real_properties.append(i)
    real_properties = tuple(real_properties)

    #Save the same ply file but with float properties instead of double
    plydata.elements[0].properties = real_properties

    #Write the data back to the original pointcloud
    plydata.write(f"{path}/mesh_float.ply")


    # print(mesh)
    # print(type(mesh))
    # print(type(mesh.vertices))
    # print(type(mesh.triangles))
    # print(type(mesh.vertex_colors))
    # print("")
    # print(np.asarray(mesh.vertices))
    # print("")
    # print(np.asarray(mesh.triangles))
    # print("")
    # print(np.asarray(mesh.vertex_colors))
    # print("")
    # # check_properties(mesh)

    # # for vertex in mesh.vertices:
    # #     print(vertex)

    # print("Try to render a mesh with normals (exist: " +
    #         str(mesh.has_vertex_normals()) + ") and colors (exist: " +
    #         str(mesh.has_vertex_colors()) + ")")


if __name__=="__main__":
    main()
