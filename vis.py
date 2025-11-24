import numpy as np
import open3d as o3d

LANDING_RADIUS = 1.0
CYLINDER_HEIGHT = 1.0

def visualize_landing_zone(data_path, zone_center):
    data = np.load(data_path)
    
    if data.shape[1] == 6:
        xyz = data[:, :3].astype(np.float32)
        colors = (data[:, 3:6].astype(np.float32) / 65535.0).clip(0,1)
    else:
        xyz = data[:, :3].copy()
        colors = None

    # Создаём облако точек
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors)
    else:
        pcd.paint_uniform_color([0.7, 0.7, 0.7])

    # Создаём цилиндр безопасной зоны
    zone = o3d.geometry.TriangleMesh.create_cylinder(radius=LANDING_RADIUS, height=CYLINDER_HEIGHT)
    zone.translate(zone_center + np.array([0, 0, CYLINDER_HEIGHT/2]))
    zone.paint_uniform_color([0, 1, 0])
    zone.compute_vertex_normals()

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Landing Zone", width=1280, height=720)
    vis.add_geometry(pcd)
    vis.add_geometry(zone)
    vis.add_geometry(center_s)
    vis.run()
    vis.destroy_window()

if __name__ == "__main__":
    #Номер патча должен совпадать с тем, который в файле config
    N = 6
    PATH = rf"D:\Documents\patches\npy\patch_{N}.npy"
    best_global = np.array([5.78112188e+05, 3.04048812e+05, 2.08691000e+02], dtype=np.float32)
    visualize_landing_zone(PATH, best_global)