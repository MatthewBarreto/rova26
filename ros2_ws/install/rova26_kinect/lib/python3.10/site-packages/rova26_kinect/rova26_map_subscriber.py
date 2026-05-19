#!/usr/bin/env python3
"""
Rova 26 - 3D Map Subscriber + Obstacle Avoidance Node
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import threading

from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header, Bool, Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA

try:
    import open3d as o3d
    OPEN3D = True
except ImportError:
    OPEN3D = False

STOP_DISTANCE_MM  = 500
SLOW_DISTANCE_MM  = 1000
VOXEL_SIZE_M      = 0.05
MAX_MAP_POINTS    = 500000


class Rova26MapSubscriber(Node):

    def __init__(self):
        super().__init__('rova26_map_subscriber')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.map_points     = np.zeros((0, 3), dtype=np.float32)
        self.wall_detected  = False
        self.band_distances = []
        self.camera_info    = None
        self.map_lock       = threading.Lock()

        self.create_subscription(PointCloud2,
            '/rova26/kinect/point_cloud',    self._cb_pointcloud,  sensor_qos)
        self.create_subscription(CameraInfo,
            '/rova26/kinect/camera_info',    self._cb_camera_info, reliable_qos)
        self.create_subscription(Bool,
            '/rova26/kinect/wall_detected',  self._cb_wall_flag,   reliable_qos)
        self.create_subscription(Float32MultiArray,
            '/rova26/kinect/wall_distances', self._cb_wall_dist,   reliable_qos)
        self.create_subscription(Image,
            '/rova26/kinect/depth_image',    self._cb_depth,       sensor_qos)

        self.pub_map     = self.create_publisher(PointCloud2, '/rova26/map3d/voxel_map',    reliable_qos)
        self.pub_markers = self.create_publisher(MarkerArray, '/rova26/map3d/wall_markers', reliable_qos)
        self.pub_cmd     = self.create_publisher(String,      '/rova26/drive/command',      reliable_qos)

        self.create_timer(0.5, self._publish_map)
        self.get_logger().info('Rova 26 - Map subscriber ready')

    def _cb_camera_info(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('Rova 26 - Camera info received')

    def _cb_wall_flag(self, msg):
        self.wall_detected = msg.data

    def _cb_wall_dist(self, msg):
        self.band_distances = list(msg.data)
        self._obstacle_avoidance()

    def _cb_depth(self, msg):
        if msg.encoding != '16UC1':
            return
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        h, w  = depth.shape
        roi   = depth[h//4:3*h//4, w//4:3*w//4]
        valid = roi[(roi > 100) & (roi < 5000)]
        if valid.size:
            centre_dist = float(np.min(valid))
            if centre_dist < STOP_DISTANCE_MM:
                self.get_logger().warn(
                    f'Rova 26 - OBSTACLE {centre_dist:.0f}mm dead ahead'
                )

    def _cb_pointcloud(self, msg):
        pts = self._pc2_to_numpy(msg)
        if pts is None or len(pts) == 0:
            return
        with self.map_lock:
            self.map_points = np.vstack([self.map_points, pts])
            if OPEN3D and len(self.map_points) > 1000:
                pcd        = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(self.map_points)
                pcd        = pcd.voxel_down_sample(VOXEL_SIZE_M)
                self.map_points = np.asarray(pcd.points, dtype=np.float32)
            if len(self.map_points) > MAX_MAP_POINTS:
                self.map_points = self.map_points[-MAX_MAP_POINTS:]

    def _obstacle_avoidance(self):
        if not self.band_distances:
            return
        middle = [d for i, d in enumerate(self.band_distances) if i in [3, 4] and d > 0]
        if not middle:
            return
        closest = min(middle)
        cmd_msg = String()
        if closest < STOP_DISTANCE_MM:
            cmd_msg.data = 'STOP'
            self.get_logger().warn(f'Rova 26 - STOP - obstacle at {closest:.0f}mm')
        elif closest < SLOW_DISTANCE_MM:
            cmd_msg.data = 'SLOW'
            self.get_logger().info(f'Rova 26 - SLOW - obstacle at {closest:.0f}mm')
        else:
            cmd_msg.data = 'CLEAR'
        self.pub_cmd.publish(cmd_msg)

    def _publish_map(self):
        with self.map_lock:
            pts = self.map_points.copy()
        if len(pts) == 0:
            return
        stamp           = self.get_clock().now().to_msg()
        header          = Header()
        header.stamp    = stamp
        header.frame_id = 'map'
        self.pub_map.publish(self._numpy_to_pc2(pts, header))
        self.get_logger().debug(f'Rova 26 - Map: {len(pts):,} points')

    def _pc2_to_numpy(self, msg):
        if msg.point_step < 12:
            return None
        n   = msg.width * msg.height
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        xyz = np.zeros((n, 3), dtype=np.float32)
        for i, name in enumerate(['x', 'y', 'z']):
            for field in msg.fields:
                if field.name == name:
                    xyz[:n, i] = np.frombuffer(
                        raw[field.offset::msg.point_step][:n * 4],
                        dtype=np.float32
                    )
                    break
        mask = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 0.01)
        return xyz[mask]

    def _numpy_to_pc2(self, pts, header):
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg              = PointCloud2()
        msg.header       = header
        msg.height       = 1
        msg.width        = len(pts)
        msg.fields       = fields
        msg.is_bigendian = False
        msg.point_step   = 12
        msg.row_step     = 12 * len(pts)
        msg.data         = pts.astype(np.float32).tobytes()
        msg.is_dense     = True
        return msg

    def destroy_node(self):
        self.get_logger().info('Rova 26 - Map subscriber stopped')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Rova26MapSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
