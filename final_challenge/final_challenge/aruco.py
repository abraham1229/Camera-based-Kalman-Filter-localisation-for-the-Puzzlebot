import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import math
from geometry_msgs.msg import PoseArray, Pose, TransformStamped, PointStamped, PoseStamped
from std_msgs.msg import Float32MultiArray
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import transforms3d

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # Declare parameters with default values
        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.declare_parameter('init_pose_yaw', 0.0)
        
        # Subscribe to camera image
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.listener_callback,
            10)
            
        # Publisher specifically for odometry node
        self.range_bearing_publisher = self.create_publisher(
            Float32MultiArray,
            'aruco_range_bearing',
            10)
        
        # Set up CV bridge
        self.bridge = CvBridge()
        
        # ArUco setup
        self.markerLength = 0.14  # 14cm marker size
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        self.parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)
        
        # Camera matrix and distortion coefficients for Gazebo (no distortion)
        self.camera_matrix = np.array([
            [528.4337, 0.0, 320.0],   # fx, 0, cx
            [0.0, 528.4338, 240.0],   # 0, fy, cy
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros((5, 1))  # No distortion in Gazebo
        
        # Known ArUco marker positions in the WORLD frame (x, y coordinates)
        # These are the absolute positions of markers in the simulation world
        self.marker_positions_world = {
            0: (2.5, -0.5),    # Marker ID 0 at world position (2.5, -0.5)
            1: (2.5, 2.5),     # Marker ID 1 at world position (2.5, 2.5)
            2: (-0.5, 2.5),    # Marker ID 2 at world position (-0.5, 2.5)
            3: (-0.5, -0.5),   # Marker ID 3 at world position (-0.5, -0.5)
            4: (1.0, 1.0),     # Marker ID 4 at world position (1.0, 1.0)
        }
        
        # TF2 setup for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Robot's initial position in world frame
        self.robot_initial_world_x = self.get_parameter('init_pose_x').value
        self.robot_initial_world_y = self.get_parameter('init_pose_y').value
        self.robot_initial_world_yaw = self.get_parameter('init_pose_yaw').value
        
        self.get_logger().info('ArucoDetectorNode started with proper TF handling')

    def transform_detection_to_base_link(self, tvec, rvec):
        """
        Transform ArUco detection from camera_link_optical frame to base_link frame.        
        Args:
            tvec: Translation vector from ArUco detection (in camera_link_optical frame)
            rvec: Rotation vector from ArUco detection (in camera_link_optical frame)
            
        Returns:
            tuple: (range, bearing) in base_link frame coordinates
        """
        try:
            # Create a PoseStamped message for the detected marker in camera_link_optical frame
            marker_pose_camera = PoseStamped()
            marker_pose_camera.header.frame_id = 'camera_link_optical'
            marker_pose_camera.header.stamp = self.get_clock().now().to_msg()
            
            # Set position from ArUco detection
            marker_pose_camera.pose.position.x = float(tvec[0])
            marker_pose_camera.pose.position.y = float(tvec[1])
            marker_pose_camera.pose.position.z = float(tvec[2])
            
            # Convert rotation vector to quaternion for orientation
            rotation_matrix, _ = cv.Rodrigues(rvec)
            quat = transforms3d.quaternions.mat2quat(rotation_matrix)
            marker_pose_camera.pose.orientation.w = float(quat[0])
            marker_pose_camera.pose.orientation.x = float(quat[1])
            marker_pose_camera.pose.orientation.y = float(quat[2])
            marker_pose_camera.pose.orientation.z = float(quat[3])
            
            # Transform from camera_link_optical to base_link
            # This accounts for the rotation between camera and robot coordinate systems
            marker_pose_base = self.tf_buffer.transform(
                marker_pose_camera, 
                'base_link', 
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Extract position in base_link frame
            x_base = marker_pose_base.pose.position.x
            y_base = marker_pose_base.pose.position.y
            z_base = marker_pose_base.pose.position.z
            
            # Calculate range and bearing in base_link frame
            # Now these measurements are properly aligned with robot's coordinate system
            range_base = math.sqrt(x_base**2 + y_base**2 + z_base**2)
            bearing_base = math.atan2(y_base, x_base)  # Bearing from robot's forward direction
            
            return range_base, bearing_base
            
        except Exception as e:
            self.get_logger().error(f"Failed to transform marker detection to base_link: {e}")
            # Fallback: return original measurements (may be inaccurate)
            range_camera = np.linalg.norm(tvec)
            bearing_camera = math.atan2(tvec[0], tvec[2])  # Note: different axes in camera frame
            return range_camera, bearing_camera

    def listener_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        # Detect ArUco markers
        markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(img)

        if markerIds is not None and len(markerCorners) > 0:
            # Draw detected markers on image
            cv.aruco.drawDetectedMarkers(img, markerCorners, markerIds)
            
            # Flatten marker IDs for easier processing
            ids = markerIds.flatten()
            
            # For each detected marker
            for i, (marker_corner, marker_id) in enumerate(zip(markerCorners, ids)):
                corners = marker_corner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                
                # Convert corner points to integers for drawing
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                
                # Calculate center point
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                
                # Draw center point
                cv.circle(img, (cX, cY), 4, (0, 0, 255), -1)
                
                # Estimate pose of the marker relative to camera_link_optical
                rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
                    [marker_corner], 
                    self.markerLength, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                
                # Draw axis for each marker (in camera frame for visualization)
                cv.drawFrameAxes(img, self.camera_matrix, self.dist_coeffs, rvecs, tvecs, 0.05)
                
                # Transform detection from camera_link_optical to base_link frame
                # This is the key step that fixes the coordinate frame mismatch
                range_base, bearing_base = self.transform_detection_to_base_link(
                    tvecs[0][0], rvecs[0][0]
                )
                
                # Add text showing range and bearing (now in base_link frame)
                cv.putText(img, f"ID: {marker_id}", (cX, cY - 15), 
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv.putText(img, f"Range: {range_base:.2f}m", (cX, cY + 15), 
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv.putText(img, f"Bearing: {math.degrees(bearing_base):.2f}°", (cX, cY + 30), 
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Check if this marker's position is known in world frame
                if marker_id in self.marker_positions_world:
                    # Get the marker's position in the world frame
                    marker_world_x, marker_world_y = self.marker_positions_world[marker_id]
                    
                    # Create range and bearing message for the odometry node
                    # Now using properly transformed measurements in base_link frame
                    range_bearing_msg = Float32MultiArray()
                    range_bearing_msg.data = [
                        float(marker_id),           # Marker ID
                        float(range_base),          # Range to marker (in base_link frame)
                        float(bearing_base),        # Bearing to marker (in base_link frame)
                        float(marker_world_x),      # Marker X position in world frame
                        float(marker_world_y)       # Marker Y position in world frame
                    ]
                    
                    # Publish range and bearing for odometry node
                    self.range_bearing_publisher.publish(range_bearing_msg)
                    
                    # Log detection with transformed measurements
                    self.get_logger().info(
                        f"Detected marker {marker_id}: range={range_base:.2f}m, "
                        f"bearing={math.degrees(bearing_base):.1f}° (base_link frame), "
                        f"world_pos=({marker_world_x:.2f}, {marker_world_y:.2f})"
                    )
                else:
                    self.get_logger().warn(f"Detected marker {marker_id} but its world position is unknown")

        # Display the image with detected markers
        cv.imshow("ArUco Detection", img)
        k = cv.waitKey(1)
        if k % 256 == 27:  # ESC key
            self.get_logger().info("Escape hit, shutting down node...")
            rclpy.shutdown()
            cv.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()