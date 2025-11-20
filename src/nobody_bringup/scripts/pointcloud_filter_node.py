#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import qos_profile_sensor_data

class PointCloudFilterNode(Node):
    def __init__(self):
        super().__init__('pointcloud_filter_node')
        self.declare_parameter('input_topic', '/pointcloud')
        self.declare_parameter('output_topic', '/filtered_pointcloud')
        self.declare_parameter('filter_type', 'bounding_box')
        self.declare_parameter('bounding_box_min', [-1.0, -1.0, -1.0])
        self.declare_parameter('bounding_box_max', [1.0, 1.0, 1.0])

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.filter_type = self.get_parameter('filter_type').value
        self.bbox_min = np.array(self.get_parameter('bounding_box_min').value)
        self.bbox_max = np.array(self.get_parameter('bounding_box_max').value)
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.pc_callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.get_logger().info(f"Filtering pointcloud from {self.input_topic} to {self.output_topic}")
        self.get_logger().info(f"Bounding box: min={self.bbox_min}, max={self.bbox_max}")

    def pc_callback(self, msg):
        # 1. PRESERVE ALL FIELDS
        # Read all points with all fields preserved
        try:
            # read_points returns generator, convert to list to get all points
            # skip_nans=False ensures we keep all points including NaN values
            points_list = list(pc2.read_points(msg, skip_nans=False))
            if len(points_list) == 0:
                self.get_logger().warn("Empty pointcloud received")
                return
        except Exception as e:
            self.get_logger().warn(f"Failed to read PointCloud2: {e}")
            return

        # Each point is a tuple, extract x,y,z for filtering
        xyz = np.array([[p[0], p[1], p[2]] for p in points_list])
        
        # 2. APPLY FILTER
        # Filter points *outside* the bounding box (remove points inside the box)
        mask = ~((xyz[:,0] >= self.bbox_min[0]) & (xyz[:,0] <= self.bbox_max[0]) &
                 (xyz[:,1] >= self.bbox_min[1]) & (xyz[:,1] <= self.bbox_max[1]) &
                 (xyz[:,2] >= self.bbox_min[2]) & (xyz[:,2] <= self.bbox_max[2]))
        
        # Apply the mask to keep all fields (using original points_list)
        filtered_points_list = [points_list[i] for i in range(len(points_list)) if mask[i]]
        
        # 3. RE-PUBLISH WITH ALL FIELDS PRESERVED
        
        # Create new message with same fields as input
        filtered_msg = pc2.create_cloud(msg.header, msg.fields, filtered_points_list)
        
        self.pub.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()