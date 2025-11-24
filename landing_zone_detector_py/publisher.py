#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from landing_zone_detector_py import config

def create_pointcloud2(points: np.ndarray) -> PointCloud2:
    """Преобразует numpy массив Nx3 в PointCloud2 (игнорируя RGB, если есть)"""
    # Берем только первые 3 колонки XYZ
    if points.shape[1] >= 3:
        points_xyz = points[:, :3].astype(np.float32)
    else:
        raise ValueError("Массив должен иметь как минимум 3 колонки (XYZ)")

    msg = PointCloud2()
    msg.header.frame_id = "map"
    msg.height = 1
    msg.width = points_xyz.shape[0]
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    ]
    msg.is_bigendian = False
    msg.point_step = 12  # 3 * 4 байта
    msg.row_step = msg.point_step * points_xyz.shape[0]
    msg.is_dense = True
    msg.data = points_xyz.tobytes()
    return msg

class NpyPublisher(Node):
    def __init__(self):
        super().__init__('npy_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'lidar_points', 10)

        patch_path = f"{config.PATCH_FOLDER}/patch_{config.CURRENT_PATCH}.npy"

        self.get_logger().info(f"Загрузка облака точек из {patch_path}")
        points = np.load(patch_path)
        self.get_logger().info(f"Загружено {points.shape[0]} точек (формат {points.shape[1]} колонок)")

        self.msg = create_pointcloud2(points)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.msg)
        self.get_logger().info(f"Отправлено облако из {self.msg.width} точек (XYZ)")

def main(args=None):
    rclpy.init(args=args)
    node = NpyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
