import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


class MapReceivedNode(Node):

    def __init__(self):
        super().__init__("map_received_node")

        self.map_received = False

        self.subscription = self.create_subscription(
            PointCloud2,
            "/points",
            self.map_callback,
            10
        )

        self.get_logger().info("Waiting for map data...")

    def map_callback(self, msg):

        if not self.map_received:

            self.get_logger().info("Map received!")

            points = pc2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True
            )

            for point in points:
                print(point)
                break

            self.map_received = True


def main(args=None):
    rclpy.init(args=args)
    node = MapReceivedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
