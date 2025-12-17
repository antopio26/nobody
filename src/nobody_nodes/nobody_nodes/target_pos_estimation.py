#!/usr/bin/env python3
"""
Argo Vision Node - ROS2 node for real-time object segmentation and pose estimation.

This node subscribes to RGB images and PointCloud2 data,
performs YOLO-based segmentation, and publishes the pose of detected objects.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, CompressedImage, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from nobody_interfaces.srv import SetTargetClass
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

class TargetPosEstimationNode(Node):
    """ROS2 node for YOLO-based object segmentation and pose estimation."""

    def __init__(self):
        """Initialize the vision node."""
        super().__init__('target_pos_estimation_node')

        # Declare parameters
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw/compressed')
        self.declare_parameter('pointcloud_topic', '/camera/camera/depth/color/points')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('pose_topic', '/argo_vision/object_pose')
        self.declare_parameter('annotated_image_topic', '/argo_vision/annotated_image/compressed')
        self.declare_parameter('filtered_pointcloud_topic', '/argo_vision/filtered_pointcloud')
        self.declare_parameter('set_target_class_service', '/argo_vision/set_target_class')
        self.declare_parameter('yolo_model', 'yolo11n-seg.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('outlier_std_dev_multiplier', 2.0)


        # Get parameter values and store in config dictionary
        self.config = {
            'image_topic': self.get_parameter('image_topic').value,
            'pointcloud_topic': self.get_parameter('pointcloud_topic').value,
            'camera_info_topic': self.get_parameter('camera_info_topic').value,
            'pose_topic': self.get_parameter('pose_topic').value,
            'annotated_image_topic': self.get_parameter('annotated_image_topic').value,
            'filtered_pointcloud_topic': self.get_parameter('filtered_pointcloud_topic').value,
            'set_target_class_service': self.get_parameter('set_target_class_service').value,
            'yolo_model': self.get_parameter('yolo_model').value,
            'confidence_threshold': self.get_parameter('confidence_threshold').value,
            'outlier_std_dev_multiplier': self.get_parameter('outlier_std_dev_multiplier').value,
        }

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Initialize YOLO model (lazy loading)
        self.yolo_model = None
        self._yolo_initialized = False

        # Target class to detect (set via service)
        self.target_class = None

        # Latest data
        self.latest_rgb = None
        self.latest_pc = None
        self.latest_rgb_header = None
        self.camera_info = None
        self.camera_matrix = None
        self.dist_coeffs = None

        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers
        self.rgb_sub = self.create_subscription(
            CompressedImage,
            self.config['image_topic'],
            self._rgb_callback,
            sensor_qos
        )

        self.pc_sub = self.create_subscription(
            PointCloud2,
            self.config['pointcloud_topic'],
            self._pc_callback,
            sensor_qos
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.config['camera_info_topic'],
            self._camera_info_callback,
            sensor_qos
        )

        # Create publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.config['pose_topic'],
            10
        )
        self.annotated_image_pub = self.create_publisher(
            CompressedImage,
            self.config['annotated_image_topic'],
            10
        )
        self.filtered_pc_pub = self.create_publisher(
            PointCloud2,
            self.config['filtered_pointcloud_topic'],
            10
        )


        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.srv = self.create_service(
            SetTargetClass,
            self.config['set_target_class_service'],
            self._set_target_class_callback
        )

        # Create timer for processing (10 Hz)
        self.timer = self.create_timer(0.1, self._process_callback)

        self.get_logger().info('Argo Vision Node initialized')
        self.get_logger().info(f'Subscribing to RGB (Compressed): {self.config["image_topic"]}')
        self.get_logger().info(f'Subscribing to PointCloud: {self.config["pointcloud_topic"]}')
        self.get_logger().info(f'Subscribing to CameraInfo: {self.config["camera_info_topic"]}')
        self.get_logger().info(f'Publishing pose to: {self.config["pose_topic"]}')
        self.get_logger().info(f'Publishing annotated image to: {self.config["annotated_image_topic"]}')
        self.get_logger().info(f'Publishing filtered pointcloud to: {self.config["filtered_pointcloud_topic"]}')



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
        """Callback for CompressedImage messages."""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.latest_rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.latest_rgb_header = msg.header
        except Exception as e:
            self.get_logger().error(f'Error decompressing RGB image: {e}')

    def _pc_callback(self, msg):
        """Callback for PointCloud2 messages."""
        self.latest_pc = msg

    def _camera_info_callback(self, msg):
        """Callback for CameraInfo messages."""
        if self.camera_info is None:
            self.camera_info = msg
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera info received.')
            # Unsubscribe after receiving camera info once
            self.destroy_subscription(self.camera_info_sub)


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
        if self.latest_rgb is None or self.latest_pc is None or self.camera_matrix is None:
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

        # --- Create local copies of data to ensure consistency ---
        rgb_image = self.latest_rgb.copy()
        point_cloud = self.latest_pc
        header = point_cloud.header

        # Run YOLO segmentation
        try:
            results = self.yolo_model(
                rgb_image,
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
        detected_classes = []
        for i, box in enumerate(result.boxes):
            class_id = int(box.cls[0])
            class_name = result.names[class_id]
            detected_classes.append(class_name)

            if class_name.lower() == self.target_class.lower():
                # Get the mask for this detection
                if result.masks is not None and i < len(result.masks.data):
                    target_mask = result.masks.data[i].cpu().numpy()
                    break

        if target_mask is None:
            self.get_logger().info(
                f'Target class "{self.target_class}" not found in frame. '
                f'Detected classes: {list(set(detected_classes))}'
            )
            return

        # Resize mask to match point cloud dimensions if necessary
        if target_mask.shape != (rgb_image.shape[0], rgb_image.shape[1]):
            target_mask = cv2.resize(
                target_mask,
                (rgb_image.shape[1], rgb_image.shape[0]),
                interpolation=cv2.INTER_NEAREST
            )

        # Compute centroid from point cloud
        pose, filtered_points = self._compute_centroid_pose(target_mask, point_cloud)

        if pose is not None and filtered_points is not None:
            current_stamp = self.get_clock().now().to_msg()

            # --- Publish everything ---
            self._publish_pose(pose, header, current_stamp)
            self._publish_tf(pose, header, current_stamp)
            self._publish_filtered_pointcloud(filtered_points, header, current_stamp)
            self._publish_annotated_image(rgb_image, target_mask, header, current_stamp)


            self.get_logger().info(
                f'Published pose for "{self.target_class}": '
                f'x={pose.pose.position.x:.3f}, '
                f'y={pose.pose.position.y:.3f}, '
                f'z={pose.pose.position.z:.3f}'
            )

    def _compute_centroid_pose(self, mask, point_cloud):
        """
        Compute the 3D centroid pose from segmentation mask and point cloud.
        Also filters outliers from the selected points.

        Args:
            mask: Binary segmentation mask
            point_cloud: The PointCloud2 message

        Returns:
            Tuple of (PoseStamped, np.ndarray) or (None, None)
        """
        # Get valid points within the mask
        mask_bool = mask > 0.5
        
        # If the point cloud is structured and registered, use the faster uvs method
        if point_cloud.height > 1 and point_cloud.width > 1:
            # This part is for structured point clouds
            if mask.shape != (point_cloud.height, point_cloud.width):
                mask_resized = cv2.resize(
                    mask,
                    (point_cloud.width, point_cloud.height),
                    interpolation=cv2.INTER_NEAREST
                )
            else:
                mask_resized = mask
            
            mask_bool_resized = mask_resized > 0.5

            points_generator = pc2.read_points(
                point_cloud,
                field_names=('x', 'y', 'z'),
                skip_nans=True,
                uvs=[(u, v) for u, v in zip(*np.where(mask_bool_resized))]
            )
            points = list(points_generator)

        else:
            # This part is for unstructured point clouds
            # We need to project each point onto the image plane
            points = []
            # Create a rotation vector and translation vector for projection
            rvec = np.zeros(3, dtype=np.float32)
            tvec = np.zeros(3, dtype=np.float32)

            # Read all points from the unstructured point cloud
            all_points = pc2.read_points_numpy(point_cloud, field_names=('x', 'y', 'z'), skip_nans=True)
            
            if all_points.shape[0] == 0:
                self.get_logger().warn('Point cloud is empty.')
                return None, None

            # Project all points to the image plane
            # We assume the point cloud is in the camera's optical frame
            image_points, _ = cv2.projectPoints(all_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)

            # Get image dimensions
            img_h, img_w = mask.shape

            # Filter points that are within the image bounds
            image_points = image_points.squeeze()
            
            # Create a boolean index for points within the image frame
            in_bounds_idx = (
                (image_points[:, 0] >= 0) & (image_points[:, 0] < img_w) &
                (image_points[:, 1] >= 0) & (image_points[:, 1] < img_h)
            )

            # Get the integer pixel coordinates for in-bounds points
            pixel_coords = image_points[in_bounds_idx].astype(int)
            
            # Check if these pixel coordinates are inside the segmentation mask
            # The mask_bool has shape (height, width)
            # pixel_coords has shape (num_points, 2) with (x, y) which is (col, row)
            # We need to index mask_bool with (row, col) which is (y, x)
            mask_values = mask_bool[pixel_coords[:, 1], pixel_coords[:, 0]]

            # The final points are the 3D points that are in bounds AND in the mask
            valid_3d_points_idx = np.where(in_bounds_idx)[0][mask_values]
            points = all_points[valid_3d_points_idx]


        if not isinstance(points, np.ndarray):
            points_arr = np.array(points)
        else:
            points_arr = points

        if points_arr.shape[0] == 0:
            self.get_logger().info('No valid points in mask')
            return None, None

        # --- Outlier removal ---
        if points_arr.shape[0] > 10: # Only filter if we have enough points
            mean = np.mean(points_arr, axis=0)
            std_dev = np.std(points_arr, axis=0)
            distance_from_mean = np.linalg.norm(points_arr - mean, axis=1)
            
            max_allowed_distance = np.median(distance_from_mean) + \
                self.config['outlier_std_dev_multiplier'] * np.std(distance_from_mean)
            
            inliers = distance_from_mean < max_allowed_distance
            filtered_points = points_arr[inliers]

            if filtered_points.shape[0] == 0:
                self.get_logger().info('Outlier filter removed all points.')
                return None, None
        else:
            filtered_points = points_arr


        # Compute centroid (mean of all 3D points)
        centroid = np.mean(filtered_points, axis=0)

        # Create PoseStamped message
        pose = PoseStamped()
        pose.pose.position.x = float(centroid[0])
        pose.pose.position.y = float(centroid[1])
        pose.pose.position.z = float(centroid[2])

        # Set orientation to identity (we don't estimate orientation)
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0

        return pose, filtered_points

    def _publish_pose(self, pose, header, stamp):
        pose.header.frame_id = header.frame_id
        pose.header.stamp = stamp
        self.pose_pub.publish(pose)

    def _publish_tf(self, pose, header, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = header.frame_id
        t.child_frame_id = f"{self.target_class}_detected"
        t.transform.translation.x = pose.pose.position.x
        t.transform.translation.y = pose.pose.position.y
        t.transform.translation.z = pose.pose.position.z
        t.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def _publish_filtered_pointcloud(self, points, header, stamp):
        if points is None or len(points) == 0:
            return
        
        pc_header = Header(stamp=stamp, frame_id=header.frame_id)
        
        # Create PointCloud2 message
        pc_msg = pc2.create_cloud_xyz32(pc_header, points)
        self.filtered_pc_pub.publish(pc_msg)

    def _publish_annotated_image(self, image, mask, header, stamp):
        # Resize mask to image dimensions
        if image.shape[:2] != mask.shape:
            mask = cv2.resize(mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)

        # Create a color overlay for the mask
        color_mask = np.zeros_like(image)
        color_mask[mask > 0.5] = [0, 255, 0]  # Green

        # Blend the original image and the mask
        annotated_image = cv2.addWeighted(image, 0.7, color_mask, 0.3, 0)

        # Resize to half resolution
        h, w = annotated_image.shape[:2]
        resized_image = cv2.resize(annotated_image, (w // 4, h // 4), interpolation=cv2.INTER_AREA)

        # Compress to JPEG
        _, buffer = cv2.imencode('.jpg', resized_image)
        
        # Create and publish CompressedImage message
        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = stamp
        compressed_msg.header.frame_id = header.frame_id
        compressed_msg.format = "jpeg"
        compressed_msg.data = buffer.tobytes()
        self.annotated_image_pub.publish(compressed_msg)


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