#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import qos_profile_sensor_data
import open3d as o3d

class PointCloudFilterNode(Node):
    def __init__(self):
        super().__init__('pointcloud_filter_node')
        self.declare_parameter('input_topic', '/pointcloud')
        self.declare_parameter('output_topic', '/filtered_pointcloud')
        self.declare_parameter('bounding_box_min', [-1.0, -1.0, -1.0])
        self.declare_parameter('bounding_box_max', [1.0, 1.0, 1.0])
        self.declare_parameter('enable_downsampling', False)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('enable_height_filter', False)
        self.declare_parameter('min_height', -float('inf'))
        self.declare_parameter('max_height', float('inf'))
        self.declare_parameter('enable_radius_filter', False)
        self.declare_parameter('max_radius', float('inf'))

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.bbox_min = np.array(self.get_parameter('bounding_box_min').value)
        self.bbox_max = np.array(self.get_parameter('bounding_box_max').value)
        self.enable_downsampling = self.get_parameter('enable_downsampling').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.enable_height_filter = self.get_parameter('enable_height_filter').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.enable_radius_filter = self.get_parameter('enable_radius_filter').value
        self.max_radius = self.get_parameter('max_radius').value
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.pc_callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.get_logger().info(f"Filtering pointcloud from {self.input_topic} to {self.output_topic}")
        self.get_logger().info(f"Bounding box: min={self.bbox_min}, max={self.bbox_max}")
        if self.enable_downsampling:
            self.get_logger().info(f"Downsampling enabled with voxel size: {self.voxel_size}")
        if self.enable_height_filter:
            self.get_logger().info(f"Height filter enabled: {self.min_height} to {self.max_height}")
        if self.enable_radius_filter:
            self.get_logger().info(f"Radius filter enabled: max radius {self.max_radius}")

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
        
        # 2. APPLY FILTERS
        # Start with all points included
        mask = np.ones(len(xyz), dtype=bool)
        
        # Filter points *outside* the bounding box (remove points inside the box)
        bbox_mask = ~((xyz[:,0] >= self.bbox_min[0]) & (xyz[:,0] <= self.bbox_max[0]) &
                      (xyz[:,1] >= self.bbox_min[1]) & (xyz[:,1] <= self.bbox_max[1]) &
                      (xyz[:,2] >= self.bbox_min[2]) & (xyz[:,2] <= self.bbox_max[2]))
        mask &= bbox_mask
        
        # Apply height filter (keep points within height range)
        if self.enable_height_filter:
            height_mask = (xyz[:,2] >= self.min_height) & (xyz[:,2] <= self.max_height)
            mask &= height_mask
        
        # Apply radius filter (keep points within max radius from origin in XY plane)
        if self.enable_radius_filter:
            radius = np.sqrt(xyz[:,0]**2 + xyz[:,1]**2)
            radius_mask = radius <= self.max_radius
            mask &= radius_mask
        
        # Apply the mask to keep all fields (using original points_list)
        filtered_points_list = [points_list[i] for i in range(len(points_list)) if mask[i]]
        
        # 2.5. APPLY DOWNSAMPLING (if enabled)
        if self.enable_downsampling and len(filtered_points_list) > 0:
            # Extract XYZ coordinates for downsampling
            xyz_filtered = np.array([[p[0], p[1], p[2]] for p in filtered_points_list])
            
            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz_filtered)
            
            # Apply voxel downsampling
            pcd_downsampled = pcd.voxel_down_sample(voxel_size=self.voxel_size)
            
            # Get downsampled coordinates
            xyz_downsampled = np.asarray(pcd_downsampled.points)
            
            # Find closest matches in original filtered points to preserve all fields
            # For each downsampled point, find the nearest point in the filtered set
            from scipy.spatial import cKDTree
            tree = cKDTree(xyz_filtered)
            _, indices = tree.query(xyz_downsampled)

            print(f"Downsampled from {len(filtered_points_list)} to {len(xyz_downsampled)} points")
            
            # Keep only the points that correspond to downsampled points
            filtered_points_list = [filtered_points_list[i] for i in indices]
        
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