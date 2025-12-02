#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import numpy as np
# Hack to fix AttributeError: module 'numpy' has no attribute 'float'
np.float = float
import tf_transformations
import math

class BaseFootprintPublisher(Node):
    def __init__(self):
        super().__init__('base_footprint_publisher')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to publish the transform at a high rate (e.g., 50Hz)
        self.create_timer(0.02, self.publish_transform)
        
        self.get_logger().info("Base Footprint Publisher Started")

    def publish_transform(self):
        try:
            # Get the transform from odom to base_link
            # We want to find where base_link is relative to the "ground" (odom Z=0)
            t = self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                rclpy.time.Time())
            
            # Extract translation and rotation
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tz = t.transform.translation.z
            
            rx = t.transform.rotation.x
            ry = t.transform.rotation.y
            rz = t.transform.rotation.z
            rw = t.transform.rotation.w
            
            # Convert quaternion to RPY
            roll, pitch, yaw = tf_transformations.euler_from_quaternion([rx, ry, rz, rw])
            
            # We want base_footprint to be at the same X,Y as base_link (in odom frame)
            # But at Z=0, Roll=0, Pitch=0 (in odom frame)
            # So base_footprint in odom is:
            # x = tx, y = ty, z = 0
            # R = 0, P = 0, Y = yaw
            
            # Now we need the transform base_link -> base_footprint
            # T_odom_bl = [R_bl | t_bl]
            # T_odom_bf = [R_bf | t_bf]  where t_bf.z = 0, R_bf only has yaw
            # We want T_bl_bf = (T_odom_bl)^-1 * T_odom_bf
            
            # Construct matrices
            mat_odom_bl = tf_transformations.compose_matrix(
                translate=[tx, ty, tz],
                angles=[roll, pitch, yaw]
            )
            
            mat_odom_bf = tf_transformations.compose_matrix(
                translate=[tx, ty, 0.0],
                angles=[0.0, 0.0, yaw]
            )
            
            mat_bl_odom = tf_transformations.inverse_matrix(mat_odom_bl)
            mat_bl_bf = tf_transformations.concatenate_matrices(mat_bl_odom, mat_odom_bf)
            
            # Extract translation and rotation for the message
            scale, shear, angles, trans, persp = tf_transformations.decompose_matrix(mat_bl_bf)
            quat = tf_transformations.quaternion_from_euler(*angles)
            
            # Create and publish the transform
            t_msg = TransformStamped()
            t_msg.header.stamp = self.get_clock().now().to_msg()
            t_msg.header.frame_id = 'base_link'
            t_msg.child_frame_id = 'base_footprint'
            
            t_msg.transform.translation.x = trans[0]
            t_msg.transform.translation.y = trans[1]
            t_msg.transform.translation.z = trans[2]
            
            t_msg.transform.rotation.x = quat[0]
            t_msg.transform.rotation.y = quat[1]
            t_msg.transform.rotation.z = quat[2]
            t_msg.transform.rotation.w = quat[3]
            
            self.tf_broadcaster.sendTransform(t_msg)
            
        except Exception as e:
            # It's normal to fail initially while waiting for tf
            self.get_logger().warn(f"Failed to get transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BaseFootprintPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
