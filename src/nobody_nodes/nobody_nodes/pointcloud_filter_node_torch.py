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
import ctypes

class PointCloudFilterNode(Node):
    def __init__(self):
        super().__init__('pointcloud_filter_node')

        # Declare parameters
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

        # Precompute squared radius for efficiency
        self.max_radius_sq = self.max_radius ** 2

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
        
        # Field offsets cache
        self._cached_fields = None
        self._cached_offsets = None
        self._cached_point_step = None

    def read_points_numpy_fast(self, msg):
        """
        Fast reading of PointCloud2 message directly into numpy array.
        Assumes Little Endian and float32 for x, y, z.
        """
        # 1. Update cache if message format changes
        if msg.point_step != self._cached_point_step or msg.fields != self._cached_fields:
            self._cached_point_step = msg.point_step
            self._cached_fields = msg.fields
            
            # Find offsets for x, y, z
            x_offset = -1
            y_offset = -1
            z_offset = -1
            
            for f in msg.fields:
                if f.name == 'x': x_offset = f.offset
                elif f.name == 'y': y_offset = f.offset
                elif f.name == 'z': z_offset = f.offset
            
            if x_offset == -1 or y_offset == -1 or z_offset == -1:
                return None, "Missing x, y, or z fields"

            self._cached_offsets = (x_offset, y_offset, z_offset)
            
        x_off, y_off, z_off = self._cached_offsets
        point_step = msg.point_step
        width = msg.width
        height = msg.height
        n_points = width * height
        
        if n_points == 0:
            return np.empty((0, 3), dtype=np.float32), None

        # Verify data length
        expected = n_points * point_step
        if len(msg.data) < expected:
             return None, f"Data size mismatch. Expected {expected}, got {len(msg.data)}"

        # Create strided views directly into the buffer
        # This avoids copying data for each column
        # Note: 'buffer' argument in ndarray requires python 3. memoryview or bytes.
        # msg.data is usually bytes or array.array
        
        try:
            # We must use a memoryview to pass to buffer (msg.data might be a custom type in some ros2 python versions, but usually bytes or array)
            # If msg.data is array.array('B'), use memoryview(msg.data)
            buf = memoryview(msg.data)
            
            # Helper to create strided view
            # shape=(n_points,), dtype=float32, offset=..., strides=(point_step,)
            def make_view(offset):
                return np.ndarray(
                    shape=(n_points,),
                    dtype=np.float32,
                    buffer=buf,
                    offset=offset,
                    strides=(point_step,)
                )

            view_x = make_view(x_off)
            view_y = make_view(y_off)
            view_z = make_view(z_off)

            # Stack them. This performs ONE copy to create the contiguous (N,3) array
            points = np.stack((view_x, view_y, view_z), axis=-1)
            
            return points, None
            
        except Exception as e:
            return None, f"Buffer view creation failed: {e}"


    def pc_callback(self, msg):
        # 1. Deserialize (CPU) optimized
        points_np, error = self.read_points_numpy_fast(msg)
        if error:
            # Fallback to slow method if fast path fails (e.g. funny fields)
            # OR just log error. For performance node, logging error is better than silently being slow.
            # But let's try fallback just in case of weird alignment? No, user wants speed.
            self.get_logger().error(f"Fast read failed: {error}")
            return
            
        if points_np.shape[0] == 0:
            return

        # 2. Transfer to GPU (CPU->GPU)
        # Using non_blocking=True requires pinned memory on CPU side usually, 
        # but even without it checking it is harmless or slight overhead.
        # Given we just allocated 'points_np', it's pageable.
        points = torch.from_numpy(points_np).to(self.device)

        # 3. Filtering Logic (GPU)
        
        # Unpack columns for readability (views)
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        
        # Start with Crop Box mask (Negative logic: remove points INSIDE box)
        # "Filter remove points inside box" -> Keep points NOT inside.
        # Inside = (x >= min) & ...
        # Keep = ~Inside
        
        # Check against bbox_min/max
        # Note: self.bbox_min is tensor on device
        
        in_box = (x >= self.bbox_min[0]) & (x <= self.bbox_max[0]) & \
                 (y >= self.bbox_min[1]) & (y <= self.bbox_max[1]) & \
                 (z >= self.bbox_min[2]) & (z <= self.bbox_max[2])
                 
        keep_mask = ~in_box
        
        # Height Filter
        if self.enable_height_filter:
            # Keep if min <= z <= max
            height_mask = (z >= self.min_height) & (z <= self.max_height)
            keep_mask &= height_mask
            
        # Radius Filter
        if self.enable_radius_filter:
            # Keep if x^2+y^2 <= r^2
            r2 = x.square() + y.square()
            radius_mask = r2 <= self.max_radius_sq
            keep_mask &= radius_mask
            
        # Apply combined mask
        points = points[keep_mask]
        
        if points.size(0) == 0:
            self.publish_empty(msg)
            return

        # 4. Voxel Grid Downsampling (GPU)
        if self.enable_downsampling:
            # Quantize coordinates
            # (N, 3) -> (N, 3) int
            coords = (points / self.voxel_size).round().int()
            
            # Find unique voxels
            # dim=0 returns unique rows
            unique_coords, inverse_indices = torch.unique(coords, sorted=True, return_inverse=True, dim=0)
            
            n_voxels = unique_coords.size(0)
            
            # Compute centroids
            # We need to sum points per voxel and count them.
            
            # Initialize outputs
            voxel_sums = torch.zeros((n_voxels, 3), device=self.device, dtype=torch.float32)
            voxel_counts = torch.zeros((n_voxels, 1), device=self.device, dtype=torch.float32)
            
            # Ones for counting
            ones = torch.ones((points.size(0), 1), device=self.device, dtype=torch.float32)
            
            # Accumulate
            voxel_sums.index_add_(0, inverse_indices, points)
            voxel_counts.index_add_(0, inverse_indices, ones)
            
            # Calculate mean
            points = voxel_sums / voxel_counts

        # 5. Serialize (GPU->CPU)
        # Move back to CPU
        points_out_np = points.cpu().numpy()
        
        # Create output PointCloud2
        # create_cloud_xyz32 handles header and float32 array
        out_msg = pc2.create_cloud_xyz32(msg.header, points_out_np)
        
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
