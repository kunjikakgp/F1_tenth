#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        # Load CSV (x, y columns only)
        points = np.genfromtxt(
            "Spielberg_centerline.csv",
            delimiter=",",
            comments="#",
            dtype=float,
            usecols=(0, 1)
        )

        # Publisher
        self.marker_pub = self.create_publisher(Marker, '/path_points', 1)

        # Create marker
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.ns = "path"
        self.marker.id = 0
        self.marker.type = Marker.SPHERE_LIST
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.2   # dot size
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.2
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0   # red
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0

        # Add all CSV points
        for x, y in points:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            self.marker.points.append(p)

        # Publish repeatedly
        self.timer = self.create_timer(1.0, self.publish_marker)

    def publish_marker(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
