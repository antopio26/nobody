#!/usr/bin/env python3
"""
Argo Vision Node - ROS2 node for real-time object segmentation and pose estimation.

This node subscribes to RGB and depth images from a RealSense camera,
performs YOLO-based segmentation, and publishes the pose of detected objects.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster

class TargetPosEstimationNode(Node):
    """ROS2 node for YOLO-based object segmentation and pose estimation."""

    def __init__(self):
        """Initialize the vision node."""
        super().__init__('target_pos_estimation_node')

        # Declare parameters
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/depth/camera_info')
        self.declare_parameter('pose_topic', '/argo_vision/object_pose')
        self.declare_parameter('set_target_class_service', '/argo_vision/set_target_class')
        self.declare_parameter('yolo_model', 'yolo11n-seg.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 10.0)

        # Get parameter values and store in config dictionary
        self.config = {
            'image_topic': self.get_parameter('image_topic').value,
            'depth_topic': self.get_parameter('depth_topic').value,
            'camera_info_topic': self.get_parameter('camera_info_topic').value,
            'pose_topic': self.get_parameter('pose_topic').value,
            'set_target_class_service': self.get_parameter('set_target_class_service').value,
            'yolo_model': self.get_parameter('yolo_model').value,
            'confidence_threshold': self.get_parameter('confidence_threshold').value,
            'min_depth': self.get_parameter('min_depth').value,
            'max_depth': self.get_parameter('max_depth').value
        }

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Initialize YOLO model (lazy loading)
        self.yolo_model = None
        self._yolo_initialized = False

        # Target class to detect (set via service)
        self.target_class = None

        # Camera intrinsics (will be set from CameraInfo message)
        self.camera_info = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Latest images
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_rgb_header = None

        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            self.config['image_topic'],
            self._rgb_callback,
            sensor_qos
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.config['depth_topic'],
            self._depth_callback,
            sensor_qos
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.config['camera_info_topic'],
            self._camera_info_callback,
            sensor_qos
        )

        # Create publisher for object pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.config['pose_topic'],
            10
        )

        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Import service type - this will be generated at build time
        # For now, we use a try-except to handle the case where it hasn't been built yet
        try:
            from argo_vision.srv import SetTargetClass
            self.srv = self.create_service(
                SetTargetClass,
                self.config['set_target_class_service'],
                self._set_target_class_callback
            )
            self.get_logger().info('SetTargetClass service created successfully')
        except ImportError:
            self.get_logger().warning(
                'SetTargetClass service not available yet. '
                'Please rebuild the package to generate service interfaces.'
            )
            self.srv = None

        # Create timer for processing (10 Hz)
        self.timer = self.create_timer(0.1, self._process_callback)

        self.get_logger().info('Argo Vision Node initialized')
        self.get_logger().info(f'Subscribing to RGB: {self.config["image_topic"]}')
        self.get_logger().info(f'Subscribing to Depth: {self.config["depth_topic"]}')
        self.get_logger().info(f'Publishing pose to: {self.config["pose_topic"]}')



    def _init_yolo(self):
        """Initialize YOLO model (lazy loading)."""
        if self._yolo_initialized:
            return True

        try:
            from ultralytics import YOLO
            self.yolo_model = YOLO(self.config['yolo_model'])
            self._yolo_initialized = True
            self.get_logger().info(
                f'YOLO model loaded: {self.config["yolo_model"]}'
            )
            return True
        except ImportError:
            self.get_logger().error(
                'ultralytics package not installed. '
                'Please install with: pip install ultralytics'
            )
            return False
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            return False

    def _rgb_callback(self, msg):
        """Callback for RGB image messages."""
        try:
            self.latest_rgb = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_rgb_header = msg.header
        except Exception as e:
            self.get_logger().error(f'Error converting RGB image: {e}')

    def _depth_callback(self, msg):
        """Callback for depth image messages."""
        try:
            # RealSense depth is typically 16UC1 (millimeters)
            if msg.encoding == '16UC1':
                self.latest_depth = self.cv_bridge.imgmsg_to_cv2(msg, '16UC1')
                # Convert to meters
                self.latest_depth = self.latest_depth.astype(np.float32) / 1000.0
            elif msg.encoding == '32FC1':
                self.latest_depth = self.cv_bridge.imgmsg_to_cv2(msg, '32FC1')
            else:
                self.latest_depth = self.cv_bridge.imgmsg_to_cv2(
                    msg, desired_encoding='passthrough'
                )
                if self.latest_depth.dtype == np.uint16:
                    self.latest_depth = self.latest_depth.astype(np.float32) / 1000.0
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')

    def _camera_info_callback(self, msg):
        """Callback for camera info messages."""
        if self.camera_info is None:
            self.camera_info = msg
            # Extract intrinsic parameters
            self.fx = msg.k[0]  # Focal length x
            self.fy = msg.k[4]  # Focal length y
            self.cx = msg.k[2]  # Principal point x
            self.cy = msg.k[5]  # Principal point y
            self.get_logger().info(
                f'Camera intrinsics received: fx={self.fx:.2f}, fy={self.fy:.2f}, '
                f'cx={self.cx:.2f}, cy={self.cy:.2f}'
            )

    def _set_target_class_callback(self, request, response):
        """Service callback to set the target class for detection."""
        self.target_class = request.target_class
        response.success = True
        response.message = f'Target class set to: {self.target_class}'
        self.get_logger().info(response.message)
        return response

    def _process_callback(self):
        """Timer callback to process images and detect objects."""
        # Check if we have all required data
        if self.latest_rgb is None or self.latest_depth is None:
            return

        if self.camera_info is None:
            self.get_logger().warning('Waiting for camera info...', throttle_duration_sec=5.0)
            return

        if self.target_class is None:
            self.get_logger().info(
                'No target class set. Call the set_target_class service.',
                throttle_duration_sec=10.0
            )
            return

        # Initialize YOLO if not already done
        if not self._init_yolo():
            return

        # Run YOLO segmentation
        try:
            results = self.yolo_model(
                self.latest_rgb,
                conf=self.config['confidence_threshold'],
                verbose=False
            )
        except Exception as e:
            self.get_logger().error(f'YOLO inference error: {e}')
            return

        # Process results
        if len(results) == 0:
            return

        result = results[0]

        # Check if we have masks (segmentation results)
        if result.masks is None or result.boxes is None:
            return

        # Find the target class in detections
        target_mask = None
        for i, box in enumerate(result.boxes):
            class_id = int(box.cls[0])
            class_name = result.names[class_id]

            if class_name.lower() == self.target_class.lower():
                # Get the mask for this detection
                if result.masks is not None and i < len(result.masks.data):
                    target_mask = result.masks.data[i].cpu().numpy()
                    break

        if target_mask is None:
            self.get_logger().debug(
                f'Target class "{self.target_class}" not found in frame'
            )
            return

        # Resize mask to match depth image dimensions if necessary
        if target_mask.shape != self.latest_depth.shape:
            target_mask = cv2.resize(
                target_mask,
                (self.latest_depth.shape[1], self.latest_depth.shape[0]),
                interpolation=cv2.INTER_NEAREST
            )

        # Compute centroid from depth map
        pose = self._compute_centroid_pose(target_mask)

        if pose is not None:
            # Set frame_id from camera image header
            pose.header.frame_id = self.latest_rgb_header.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()

            # Publish pose
            self.pose_pub.publish(pose)

            # Publish TF
            t = TransformStamped()
            t.header.stamp = pose.header.stamp
            t.header.frame_id = pose.header.frame_id
            t.child_frame_id = f"{self.target_class}_detected"
            t.transform.translation.x = pose.pose.position.x
            t.transform.translation.y = pose.pose.position.y
            t.transform.translation.z = pose.pose.position.z
            t.transform.rotation = pose.pose.orientation
            
            self.tf_broadcaster.sendTransform(t)

            self.get_logger().debug(
                f'Published pose for "{self.target_class}": '
                f'x={pose.pose.position.x:.3f}, '
                f'y={pose.pose.position.y:.3f}, '
                f'z={pose.pose.position.z:.3f}'
            )

    def _compute_centroid_pose(self, mask):
        """
        Compute the 3D centroid pose from segmentation mask and depth map.

        Args:
            mask: Binary segmentation mask

        Returns:
            PoseStamped message with centroid position, or None if invalid
        """
        # Get valid depth points within the mask
        mask_bool = mask > 0.5

        # Filter by depth range
        depth_masked = np.where(
            mask_bool &
            (self.latest_depth > self.config['min_depth']) &
            (self.latest_depth < self.config['max_depth']),
            self.latest_depth,
            0
        )

        # Get indices of valid points
        valid_indices = np.where(depth_masked > 0)

        if len(valid_indices[0]) == 0:
            self.get_logger().debug('No valid depth points in mask')
            return None

        # Get depth values at valid points
        depths = depth_masked[valid_indices]

        # Compute 3D points using pinhole camera model
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        # Z = depth
        v_coords = valid_indices[0]  # row indices
        u_coords = valid_indices[1]  # column indices

        x_coords = (u_coords - self.cx) * depths / self.fx
        y_coords = (v_coords - self.cy) * depths / self.fy
        z_coords = depths

        # Compute centroid (mean of all 3D points)
        centroid_x = float(np.mean(x_coords))
        centroid_y = float(np.mean(y_coords))
        centroid_z = float(np.mean(z_coords))

        # Create PoseStamped message
        pose = PoseStamped()
        pose.pose.position.x = centroid_x
        pose.pose.position.y = centroid_y
        pose.pose.position.z = centroid_z

        # Set orientation to identity (we don't estimate orientation)
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0

        return pose


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = TargetPosEstimationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()