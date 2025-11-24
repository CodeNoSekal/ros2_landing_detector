#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct
import math
import time
from scipy.spatial import cKDTree
from landing_zone_detector_py import config

# ---------------- ПАРАМЕТРЫ ----------------
LANDING_RADIUS = config.LANDING_RADIUS
SCAN_MAX_RADIUS = 15.0
GRID_STEP = 1
MIN_POINTS_IN_PATCH = 60
MAX_HEIGHT_STD = 0.12
MAX_HEIGHT_OUTLIER = 0.25
MAX_TILT_DEG = 8.0
TOP_CANDIDATES = 100
SAMPLE_PER_PATCH = 150

def pointcloud2_to_xyz_array(cloud_msg):
    """Преобразует PointCloud2 в numpy Nx3"""
    dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
    data = np.frombuffer(cloud_msg.data, dtype=dtype)
    pts = np.vstack((data['x'], data['y'], data['z'])).T
    return pts

def fit_plane(points):
    X, Y, Z = points[:,0], points[:,1], points[:,2]
    A = np.c_[X, Y, np.ones_like(X)]
    C, _, _, _ = np.linalg.lstsq(A, Z, rcond=None)
    a, b, _ = C
    n = np.array([-a, -b, 1.0])
    n /= np.linalg.norm(n)
    tilt = math.degrees(math.acos(abs(n[2])))
    return tilt

def find_safe_landing_point(pts):
    x_min, x_max = pts[:,0].min(), pts[:,0].max()
    y_min, y_max = pts[:,1].min(), pts[:,1].max()
    center_xy = np.array([(x_min + x_max)/2, (y_min + y_max)/2])
    pts[:, :2] -= center_xy

    d = np.linalg.norm(pts[:, :2], axis=1)
    pts = pts[d <= SCAN_MAX_RADIUS]
    if len(pts) < MIN_POINTS_IN_PATCH:
        return None

    xs = np.arange(pts[:,0].min(), pts[:,0].max(), GRID_STEP)
    ys = np.arange(pts[:,1].min(), pts[:,1].max(), GRID_STEP)
    gx, gy = np.meshgrid(xs, ys)
    grid = np.vstack([gx.ravel(), gy.ravel()]).T

    tree = cKDTree(pts[:, :2])
    idxs = tree.query_ball_point(grid, r=LANDING_RADIUS)

    heights = np.array([
        np.std(pts[i,2]) if len(i) >= MIN_POINTS_IN_PATCH else 999
        for i in idxs
    ])
    ok_mask = heights < MAX_HEIGHT_STD
    good_cells = grid[ok_mask]
    idxs_good = [idxs[i] for i in np.nonzero(ok_mask)[0]]
    if len(good_cells) == 0:
        return None

    dists = np.linalg.norm(good_cells, axis=1)
    top_idx = np.argsort(dists)[:TOP_CANDIDATES]
    best_patch, best_dist, best_tilt, best_std, best_z = None, 1e9, 90, 1e9, 0

    for c, idx in zip(good_cells[top_idx], np.array(idxs_good, dtype=object)[top_idx]):
        patch = pts[idx]
        if len(patch) > SAMPLE_PER_PATCH:
            patch = patch[np.random.choice(len(patch), SAMPLE_PER_PATCH, replace=False)]
        z_std = np.std(patch[:,2])
        z_range = np.max(patch[:,2]) - np.median(patch[:,2])
        if z_std > MAX_HEIGHT_STD or z_range > MAX_HEIGHT_OUTLIER:
            continue
        tilt = fit_plane(patch)
        if tilt < MAX_TILT_DEG:
            dist = np.linalg.norm(c)
            if dist < best_dist:
                best_patch, best_dist = c, dist
                best_tilt, best_std = tilt, z_std
                best_z = np.median(patch[:,2])

    if best_patch is None:
        return None

    best_global = np.array([best_patch[0]+center_xy[0],
                            best_patch[1]+center_xy[1],
                            best_z])
    return best_global, best_tilt, best_std

class LandingZoneSubscriber(Node):
    def __init__(self):
        super().__init__('landing_zone_subscriber')
        self.subscription = self.create_subscription(PointCloud2, 'lidar_points', self.listener_callback, 10)
        self.subscription

    def listener_callback(self, msg):
        t0 = time.time()
        pts = pointcloud2_to_xyz_array(msg)
        result = find_safe_landing_point(pts)
        if result is None:
            self.get_logger().warn("Безопасная зона не найдена")
        else:
            center, tilt, std = result
            self.get_logger().info(f"Центр безопасной зоны: {center.round(3)}")
            self.get_logger().info(f"Угол наклона: {tilt:.2f}° | std: {std:.3f} м | {(time.time()-t0):.3f} с")

def main(args=None):
    rclpy.init(args=args)
    node = LandingZoneSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
