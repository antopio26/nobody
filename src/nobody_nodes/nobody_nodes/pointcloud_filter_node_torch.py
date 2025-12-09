#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import torch
import numpy as np
import math
import struct
import sys

class PointCloudFilterNode(Node):
    def __init__(self):
        super().__init__('pointcloud_filter_node')

        # Declare parameters (matching C++ node)
        self.declare_parameter("input_topic", "/pointcloud")
        self.declare_parameter("output_topic", "/filtered_pointcloud")
        self.declare_parameter("bounding_box_min", [-1.0, -1.0, -1.0])
        self.declare_parameter("bounding_box_max", [1.0, 1.0, 1.0])
        self.declare_parameter("enable_downsampling", False)
        self.declare_parameter("voxel_size", 0.05)
        self.declare_parameter("enable_height_filter", False)
        self.declare_parameter("min_height", -float('inf'))
        self.declare_parameter("max_height", float('inf'))
        self.declare_parameter("enable_radius_filter", False)
        self.declare_parameter("max_radius", float('inf'))

        # Get parameters
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        
        bbox_min_list = self.get_parameter("bounding_box_min").value
        bbox_max_list = self.get_parameter("bounding_box_max").value
        
        # Check device
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f"Using device: {self.device}")

        # Parameter setup
        if len(bbox_min_list) == 3:
            self.bbox_min = torch.tensor(bbox_min_list, device=self.device, dtype=torch.float32)
        else:
            self.get_logger().error("bounding_box_min must have 3 elements")
            self.bbox_min = torch.tensor([-1.0, -1.0, -1.0], device=self.device, dtype=torch.float32)

        if len(bbox_max_list) == 3:
            self.bbox_max = torch.tensor(bbox_max_list, device=self.device, dtype=torch.float32)
        else:
            self.get_logger().error("bounding_box_max must have 3 elements")
            self.bbox_max = torch.tensor([1.0, 1.0, 1.0], device=self.device, dtype=torch.float32)

        self.enable_downsampling = self.get_parameter("enable_downsampling").value
        self.voxel_size = self.get_parameter("voxel_size").value
        self.enable_height_filter = self.get_parameter("enable_height_filter").value
        self.min_height = self.get_parameter("min_height").value
        self.max_height = self.get_parameter("max_height").value
        self.enable_radius_filter = self.get_parameter("enable_radius_filter").value
        self.max_radius = self.get_parameter("max_radius").value

        # QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pc_callback,
            qos
        )
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)

        self.get_logger().info(f"Filtering pointcloud from {self.input_topic} to {self.output_topic}")
        self.get_logger().info(f"Bounding box: min={bbox_min_list}, max={bbox_max_list}")

    def pc_callback(self, msg):
        # deserialization is CPU bound and unavoidable with standard messages
        # We read only x, y, z. standard C++ node converts to PCL PointXYZ (stripping others)
        # So we will do the same.
        
        # Read points as numpy array (N, 3)
        # field_names=None reads all fields, but we specifically want xyz for processing
        # Actually, reading as float32 bytes directly is faster using general numpy buffer
        # But pc2.read_points_numpy is convenient.
        
        # NOTE: read_points_numpy reads all fields into a structured array by default or separate columns?
        # sensor_msgs_py.point_cloud2.read_points returns generator.
        # There is read_points_numpy in newer ROS2 versions, let's check if available or implement manual
        
        points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = list(points_gen)
        
        if not points_list:
            return

        # Convert to tensor
        # Convert to numpy array first (might be structured)
        points_np = np.array(points_list)
        
        # If structured array (fields x, y, z), extract them
        if points_np.dtype.names:
            points_np = np.stack([points_np['x'], points_np['y'], points_np['z']], axis=1)
            
        points_np = points_np.astype(np.float32)
        points = torch.from_numpy(points_np).to(self.device) # (N, 3)

        if points.shape[0] == 0:
            return

        # 1. Crop Box Filter
        # Remove points INSIDE the box (as per C++ logic: setNegative(true))
        # Condition: inside if all coords are within min/max
        # Mask for points INSIDE
        
        # C++:
        # box_filter.setMin(bbox_min_);
        # box_filter.setMax(bbox_max_);
        # box_filter.setNegative(true); // Remove points inside
        
        # In torch: keep if NOT inside
        # Inside = (x >= min_x) & (x <= max_x) & (y >= min_y) ...
        
        in_box_mask = (
            (points[:, 0] >= self.bbox_min[0]) & (points[:, 0] <= self.bbox_max[0]) &
            (points[:, 1] >= self.bbox_min[1]) & (points[:, 1] <= self.bbox_max[1]) &
            (points[:, 2] >= self.bbox_min[2]) & (points[:, 2] <= self.bbox_max[2])
        )
        
        # Keep points that are NOT inside the box
        points = points[~in_box_mask]

        if points.shape[0] == 0:
            self.publish_empty(msg)
            return

        # 2. Height Filter
        if self.enable_height_filter:
            # PassThrough z
            # setFilterLimits(min, max). Default negative=False (keep INSIDE)
            height_mask = (points[:, 2] >= self.min_height) & (points[:, 2] <= self.max_height)
            points = points[height_mask]

        if points.shape[0] == 0:
            self.publish_empty(msg)
            return

        # 3. Radius Filter
        if self.enable_radius_filter:
            # Keep if simple radius (x^2 + y^2) <= r^2
            r2 = points[:, 0]**2 + points[:, 1]**2
            radius_mask = r2 <= (self.max_radius ** 2)
            points = points[radius_mask]
            
        if points.shape[0] == 0:
            self.publish_empty(msg)
            return

        # 4. Voxel Grid Downsampling
        if self.enable_downsampling:
            # Quantize
            coords = torch.round(points / self.voxel_size).int()
            
            # Use unique to find unique voxels
            # To use unique on rows, we can view as specialized struct or pack unique ID
            # Assuming discrete coordinates within int32 range.
            
            # Simple approach: unique on computed combined key or use unique with dim=0 (slower in older torch)
            # torch.unique with dim=0 is available in modern torch
            
            unique_coords, inverse_indices = torch.unique(coords, sorted=True, return_inverse=True, dim=0)
            
            # Compute centroids
            # scatter_add_ or index_add_
            
            n_voxels = unique_coords.shape[0]
            
            # Sum points per voxel
            # zeros (N_voxels, 3)
            # index_add_(dim, index, source)
            
            voxel_sums = torch.zeros((n_voxels, 3), device=self.device, dtype=torch.float32)
            voxel_sums.index_add_(0, inverse_indices, points)
            
            # Count points per voxel
            voxel_counts = torch.zeros((n_voxels, 1), device=self.device, dtype=torch.float32)
            ones = torch.ones((points.shape[0], 1), device=self.device, dtype=torch.float32)
            voxel_counts.index_add_(0, inverse_indices, ones)
            
            # Centroids
            points = voxel_sums / voxel_counts

        # 5. Publish
        # Convert back to CPU numpy
        points_out_np = points.cpu().numpy()
        
        # Create PointCloud2
        # We need to construct the message manually or use create_cloud_xyz32
        
        # Header
        header = msg.header
        
        # create_cloud_xyz32 is efficient
        out_msg = pc2.create_cloud_xyz32(header, points_out_np)
        
        self.pub.publish(out_msg)

    def publish_empty(self, original_msg):
        out_msg = pc2.create_cloud_xyz32(original_msg.header, [])
        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
