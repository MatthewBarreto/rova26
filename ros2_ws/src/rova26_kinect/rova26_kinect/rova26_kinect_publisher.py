#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import pyk4a
from pyk4a import PyK4A, Config, ColorResolution, DepthMode, FPS

from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header, Bool, Float32MultiArray

WALL_MIN_MM    = 200
WALL_MAX_MM    = 3000
WALL_THRESHOLD = 0.15
WALL_BANDS     = 8


class Rova26KinectPublisher(Node):

    def __init__(self):
        super().__init__('rova26_kinect_publisher')

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

        self.pub_depth       = self.create_publisher(Image,             '/rova26/kinect/depth_image',    sensor_qos)
        self.pub_rgb         = self.create_publisher(Image,             '/rova26/kinect/rgb_image',      sensor_qos)
        self.pub_pointcloud  = self.create_publisher(PointCloud2,       '/rova26/kinect/point_cloud',    sensor_qos)
        self.pub_camera_info = self.create_publisher(CameraInfo,        '/rova26/kinect/camera_info',    reliable_qos)
        self.pub_wall_flag   = self.create_publisher(Bool,              '/rova26/kinect/wall_detected',  reliable_qos)
        self.pub_wall_dist   = self.create_publisher(Float32MultiArray, '/rova26/kinect/wall_distances', reliable_qos)

        self.k4a = PyK4A(Config(
            color_resolution=ColorResolution.RES_720P,
            depth_mode=DepthMode.NFOV_UNBINNED,
            camera_fps=FPS.FPS_15,
            synchronized_images_only=True,
        ))
        self.k4a.start()
        self.calibration     = self.k4a.calibration
        self.camera_info_msg = self._build_camera_info()
        self.get_logger().info('Rova 26 - Kinect started')
        self.create_timer(1.0 / 15.0, self.capture_and_publish)

    def capture_and_publish(self):
        capture = self.k4a.get_capture()
        if capture is None:
            return
        depth = capture.depth
        color = capture.color
        if depth is None:
            return
        header            = Header()
        header.stamp      = self.get_clock().now().to_msg()
        header.frame_id   = 'rova26_depth_frame'
        self.pub_depth.publish(self._depth_to_image(depth, header))
        if color is not None:
            self.pub_rgb.publish(self._color_to_image(color, header))
        wall_detected, band_distances = self._detect_walls(depth)
        wall_msg      = Bool()
        wall_msg.data = bool(wall_detected)
        self.pub_wall_flag.publish(wall_msg)
        dist_msg      = Float32MultiArray()
        dist_msg.data = [float(d) for d in band_distances]
        self.pub_wall_dist.publish(dist_msg)
        pc_msg = self._build_point_cloud(depth, header)
        if pc_msg:
            self.pub_pointcloud.publish(pc_msg)
        self.camera_info_msg.header = header
        self.pub_camera_info.publish(self.camera_info_msg)
        if wall_detected:
            self.get_logger().warn(
                f'Rova 26 - WALL DETECTED - zones (mm): {[round(d) for d in band_distances]}'
            )

    def _detect_walls(self, depth):
        h, w        = depth.shape
        band_height = h // WALL_BANDS
        valid_mask  = (depth > WALL_MIN_MM) & (depth < WALL_MAX_MM)
        band_distances = []
        for i in range(WALL_BANDS):
            start = i * band_height
            end   = start + band_height
            band  = depth[start:end, :]
            mask  = valid_mask[start:end, :]
            count = np.sum(mask)
            if count > 0:
                band_distances.append(float(np.mean(band[mask])))
            else:
                band_distances.append(0.0)
        wall_fraction = np.sum(valid_mask) / depth.size
        wall_detected = wall_fraction >= WALL_THRESHOLD
        return wall_detected, band_distances

    def _build_point_cloud(self, depth, header):
        K  = self.calibration.get_camera_matrix(pyk4a.CalibrationType.DEPTH)
        fx = K[0, 0]; fy = K[1, 1]
        cx = K[0, 2]; cy = K[1, 2]
        h, w = depth.shape
        u    = np.tile(np.arange(w), h).reshape(h, w)
        v    = np.repeat(np.arange(h), w).reshape(h, w)
        z    = depth.astype(np.float32) / 1000.0
        mask = (z > WALL_MIN_MM / 1000.0) & (z < WALL_MAX_MM / 1000.0)
        x = ((u - cx) * z / fx)[mask]
        y = ((v - cy) * z / fy)[mask]
        z = z[mask]
        if x.size == 0:
            return None
        points = np.column_stack([x, y, z]).astype(np.float32)
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg              = PointCloud2()
        msg.header       = header
        msg.height       = 1
        msg.width        = len(points)
        msg.fields       = fields
        msg.is_bigendian = False
        msg.point_step   = 12
        msg.row_step     = 12 * len(points)
        msg.data         = points.tobytes()
        msg.is_dense     = True
        return msg

    def _depth_to_image(self, depth, header):
        msg          = Image()
        msg.header   = header
        msg.height   = depth.shape[0]
        msg.width    = depth.shape[1]
        msg.encoding = '16UC1'
        msg.step     = depth.shape[1] * 2
        msg.data     = depth.astype(np.uint16).tobytes()
        return msg

    def _color_to_image(self, color, header):
        msg          = Image()
        msg.header   = header
        msg.height   = color.shape[0]
        msg.width    = color.shape[1]
        msg.encoding = 'bgra8'
        msg.step     = color.shape[1] * 4
        msg.data     = color.tobytes()
        return msg

    def _build_camera_info(self):
        K    = self.calibration.get_camera_matrix(pyk4a.CalibrationType.DEPTH)
        dist = self.calibration.get_distortion_coefficients(pyk4a.CalibrationType.DEPTH)
        msg                  = CameraInfo()
        msg.width            = 640
        msg.height           = 576
        msg.distortion_model = 'plumb_bob'
        msg.k                = K.flatten().tolist()
        msg.d                = dist.flatten().tolist()
        msg.r                = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p                = [K[0,0], 0.0, K[0,2], 0.0,
                                0.0, K[1,1], K[1,2], 0.0,
                                0.0, 0.0, 1.0, 0.0]
        return msg

    def destroy_node(self):
        self.k4a.stop()
        self.get_logger().info('Rova 26 - Kinect stopped')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Rova26KinectPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
