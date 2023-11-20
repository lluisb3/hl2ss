import open3d as o3d
import open3d.visualization.gui as gui
import click


@click.command()
@click.option(
    "--pcd_path",
    prompt="Path to .ply file with pointcloud",
    help="Path to .ply file with pointcloud",
)
def main(pcd_path):
    scene_mask = o3d.io.read_point_cloud(pcd_path)

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

if __name__ == '__main__':
    main()