#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import qos_profile_sensor_data
import tf2_ros
from tf2_ros import TransformException  # Import the specific exception
from tf2_geometry_msgs import do_transform_point # Use the recommended import
from geometry_msgs.msg import PointStamped

def get_bbox_corners(min_pt, max_pt):
    # 8 corners of (min_x, min_y, min_z) to (max_x, max_y, max_z)
    return [
        [min_pt[0], min_pt[1], min_pt[2]],
        [min_pt[0], min_pt[1], max_pt[2]],
        [min_pt[0], max_pt[1], min_pt[2]],
        [min_pt[0], max_pt[1], max_pt[2]],
        [max_pt[0], min_pt[1], min_pt[2]],
        [max_pt[0], min_pt[1], max_pt[2]],
        [max_pt[0], max_pt[1], min_pt[2]],
        [max_pt[0], max_pt[1], max_pt[2]],
    ]

class PointCloudFilterNode(Node):
    def __init__(self):
        super().__init__('pointcloud_filter_node')
        self.declare_parameter('input_topic', '/pointcloud')
        self.declare_parameter('output_topic', '/filtered_pointcloud')
        self.declare_parameter('filter_type', 'bounding_box')
        self.declare_parameter('bounding_box_frame', 'base_link')
        self.declare_parameter('bounding_box_min', [-1.0, -1.0, -1.0])
        self.declare_parameter('bounding_box_max', [1.0, 1.0, 1.0])

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.filter_type = self.get_parameter('filter_type').value
        self.bbox_frame = self.get_parameter('bounding_box_frame').value
        self.bbox_min = np.array(self.get_parameter('bounding_box_min').value)
        self.bbox_max = np.array(self.get_parameter('bounding_box_max').value)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.pc_callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.get_logger().info(f"Filtering pointcloud from {self.input_topic} to {self.output_topic}")

    def pc_callback(self, msg):
        # 1. TRANSFORM BOUNDING BOX TO POINTCLOUD FRAME
        try:
            # Get transform from bbox_frame (source) to pointcloud frame (target)
            tf = self.tf_buffer.lookup_transform(
                msg.header.frame_id,  # Target frame (e.g., 'radar')
                self.bbox_frame,       # Source frame (e.g., 'base_link')
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except TransformException as e:
            # This is where your REAL error is happening
            self.get_logger().warn(
                f"Could not transform '{self.bbox_frame}' to '{msg.header.frame_id}': {e}")
            return

        corners = get_bbox_corners(self.bbox_min, self.bbox_max)
        transformed_corners = []
        for corner in corners:
            # Create a PointStamped in the source frame
            pt_in_bbox_frame = PointStamped()
            pt_in_bbox_frame.header.frame_id = self.bbox_frame
            # Note: Do not set header.stamp, do_transform_point does not use it
            pt_in_bbox_frame.point.x, pt_in_bbox_frame.point.y, pt_in_bbox_frame.point.z = corner
            
            # Transform the point to the target frame (pointcloud frame)
            pt_in_pc_frame = do_transform_point(pt_in_bbox_frame, tf)
            transformed_corners.append([
                pt_in_pc_frame.point.x, 
                pt_in_pc_frame.point.y, 
                pt_in_pc_frame.point.z
            ])

        # Create the new Axis-Aligned Bounding Box (AABB) in the pointcloud frame
        aabb_min = np.min(transformed_corners, axis=0)
        aabb_max = np.max(transformed_corners, axis=0)

        # 2. PRESERVE ALL FIELDS
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
        
        # 3. APPLY FILTER
        # Filter points *outside* the transformed box
        mask = ~((xyz[:,0] >= aabb_min[0]) & (xyz[:,0] <= aabb_max[0]) &
                 (xyz[:,1] >= aabb_min[1]) & (xyz[:,1] <= aabb_max[1]) &
                 (xyz[:,2] >= aabb_min[2]) & (xyz[:,2] <= aabb_max[2]))
        
        # Apply the mask to keep all fields (using original points_list)
        filtered_points_list = [points_list[i] for i in range(len(points_list)) if mask[i]]
        
        # 4. RE-PUBLISH WITH ALL FIELDS PRESERVED
        
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