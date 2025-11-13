import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
import tf2_ros
import tf2_py as tf2
import tf2_sensor_msgs.tf2_sensor_msgs

class PointCloudFilterNode(Node):
    def __init__(self):
        super().__init__('pointcloud_filter_node')

        # Declare parameters
        self.declare_parameter('input_topic', '/pointcloud')
        self.declare_parameter('output_topic', '/filtered_pointcloud')
        self.declare_parameter('filter_type', 'bounding_box')  # Could be extended
        self.declare_parameter('bounding_box_frame', 'base_link')
        self.declare_parameter('bounding_box_min', [-1.0, -1.0, -1.0])
        self.declare_parameter('bounding_box_max', [1.0, 1.0, 1.0])

        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.filter_type = self.get_parameter('filter_type').value
        self.bbox_frame = self.get_parameter('bounding_box_frame').value
        self.bbox_min = np.array(self.get_parameter('bounding_box_min').value)
        self.bbox_max = np.array(self.get_parameter('bounding_box_max').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pc_callback,
            qos_profile_sensor_data)

        self.pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            10)

        self.get_logger().info(f"Filtering pointcloud from {self.input_topic} to {self.output_topic} by {self.filter_type}")

    def pc_callback(self, msg: PointCloud2):
        # Transform pointcloud to bounding box frame if necessary
        target_frame = self.bbox_frame
        if msg.header.frame_id != target_frame:
            try:
                # Lookup transform
                t = self.tf_buffer.lookup_transform(
                    target_frame,
                    msg.header.frame_id,
                    msg.header.stamp,
                    timeout=rclpy.duration.Duration(seconds=0.5))
                # Transform pointcloud
                msg_transformed = tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(msg, t)
            except Exception as e:
                self.get_logger().warn(f"Transform from {msg.header.frame_id} to {target_frame} failed: {e}")
                return
        else:
            msg_transformed = msg

        points = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(msg_transformed, skip_nans=True)])

        # Apply filter based on filter_type (currently bounding box)
        if self.filter_type == 'bounding_box':
            mask = self._filter_bounding_box(points)
            filtered_points = points[mask]
        else:
            self.get_logger().warn(f"Unknown filter type: {self.filter_type}")
            return

        if len(filtered_points) == 0:
            self.get_logger().info("No points left after filtering")
            return

        # Create filtered PointCloud2 message in bounding box frame
        filtered_pc2 = pc2.create_cloud_xyz32(msg_transformed.header, filtered_points)

        self.pub.publish(filtered_pc2)

    def _filter_bounding_box(self, points: np.ndarray) -> np.ndarray:
        # Keep points outside bounding box (filter out points inside)
        inside_x = (points[:, 0] >= self.bbox_min[0]) & (points[:, 0] <= self.bbox_max[0])
        inside_y = (points[:, 1] >= self.bbox_min[1]) & (points[:, 1] <= self.bbox_max[1])
        inside_z = (points[:, 2] >= self.bbox_min[2]) & (points[:, 2] <= self.bbox_max[2])
        inside_bbox = inside_x & inside_y & inside_z
        # Return mask for points outside the bounding box
        return ~inside_bbox


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
