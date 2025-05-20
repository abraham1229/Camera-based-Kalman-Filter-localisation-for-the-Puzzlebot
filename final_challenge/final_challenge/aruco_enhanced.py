import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import math
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32MultiArray

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        # Subscribe to camera image
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # '/video_source/raw'
            self.listener_callback,
            10)
        
        # Create publishers for ArUco data
        self.marker_pose_publisher = self.create_publisher(
            PoseArray,
            'aruco_poses',
            10)
            
        # Publisher specifically for odometry node
        self.range_bearing_publisher = self.create_publisher(
            Float32MultiArray,
            'aruco_range_bearing',
            10)
        
        # Set up CV bridge
        self.bridge = CvBridge()
        
        # ArUco setup
        self.markerLength = 0.09  # 9cm marker size
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        self.parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)
        
        # Camera matrix and distortion coefficients
        # These are placeholder values - for accurate measurements you should calibrate your camera
        self.camera_matrix = np.array([
            [800.0, 0.0, 320.0],
            [0.0, 800.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros((5, 1))
        
        # Known ArUco marker positions in the world (x, y coordinates)
        # Fill in the actual positions of your markers in the environment
        self.marker_positions = {
            0: (1.0, 0.0),    # Marker ID 0 at position (1.0, 0.0)
            1: (0.0, 1.0),    # Marker ID 1 at position (0.0, 1.0)
            2: (-1.0, 0.0),   # etc.
            3: (0.0, -1.0),
            # Add more markers as needed
        }
        
        # Declare parameters for camera settings
        self.declare_parameter('camera_fx', 800.0)
        self.declare_parameter('camera_fy', 800.0)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)
        
        # Update camera matrix from parameters
        self.camera_matrix[0, 0] = self.get_parameter('camera_fx').value
        self.camera_matrix[1, 1] = self.get_parameter('camera_fy').value
        self.camera_matrix[0, 2] = self.get_parameter('camera_cx').value
        self.camera_matrix[1, 2] = self.get_parameter('camera_cy').value
        
        self.get_logger().info('ArucoDetectorNode started, waiting for images on /image_raw')

    def listener_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        # Detect ArUco markers
        markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(img)

        # Create pose array message
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = "camera_frame"  # Set appropriate frame

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
                
                # Estimate pose of the marker
                rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
                    [marker_corner], 
                    self.markerLength, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                
                # Draw axis for each marker
                cv.drawFrameAxes(img, self.camera_matrix, self.dist_coeffs, rvecs, tvecs, 0.05)
                
                # Calculate range (distance) to the marker
                range_to_marker = np.linalg.norm(tvecs)
                
                # Calculate bearing (angle) to the marker in camera frame
                bearing_to_marker = math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                
                # Add text showing range and bearing
                cv.putText(img, f"ID: {marker_id}", (cX, cY - 15), 
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv.putText(img, f"Range: {range_to_marker:.2f}m", (cX, cY + 15), 
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv.putText(img, f"Bearing: {math.degrees(bearing_to_marker):.1f}°", (cX, cY + 30), 
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Check if this marker's position is known
                if marker_id in self.marker_positions:
                    # Get the marker's position in the world
                    marker_pos_x, marker_pos_y = self.marker_positions[marker_id]
                    
                    # Create range and bearing message for the odometry node
                    range_bearing_msg = Float32MultiArray()
                    range_bearing_msg.data = [
                        float(marker_id),                  # Marker ID
                        float(range_to_marker),            # Range to marker
                        float(bearing_to_marker),          # Bearing to marker
                        float(marker_pos_x),               # Marker X position
                        float(marker_pos_y)                # Marker Y position
                    ]
                    
                    # Publish range and bearing for odometry
                    self.range_bearing_publisher.publish(range_bearing_msg)
                    
                    # Log detection
                    self.get_logger().info(
                        f"Detected marker {marker_id} at range: {range_to_marker:.2f}m, "
                        f"bearing: {math.degrees(bearing_to_marker):.1f}°, "
                        f"world position: ({marker_pos_x}, {marker_pos_y})"
                    )
                else:
                    self.get_logger().warn(f"Detected marker {marker_id} but its position is unknown")
                
                # Create a pose message for this marker and add to array
                marker_pose = Pose()
                marker_pose.position.x = tvecs[0][0][0]
                marker_pose.position.y = tvecs[0][0][1]
                marker_pose.position.z = tvecs[0][0][2]
                
                # Convert rotation vector to quaternion
                rot_matrix, _ = cv.Rodrigues(rvecs[0])
                rot_matrix_3x3 = np.eye(4)
                rot_matrix_3x3[:3, :3] = rot_matrix
                
                # Extract quaternion
                q_x = rot_matrix_3x3[0, 0]
                q_y = rot_matrix_3x3[1, 1]
                q_z = rot_matrix_3x3[2, 2]
                q_w = math.sqrt(1 + q_x + q_y + q_z) / 2
                
                marker_pose.orientation.w = q_w
                marker_pose.orientation.x = (rot_matrix_3x3[2, 1] - rot_matrix_3x3[1, 2]) / (4 * q_w)
                marker_pose.orientation.y = (rot_matrix_3x3[0, 2] - rot_matrix_3x3[2, 0]) / (4 * q_w)
                marker_pose.orientation.z = (rot_matrix_3x3[1, 0] - rot_matrix_3x3[0, 1]) / (4 * q_w)
                
                pose_array_msg.poses.append(marker_pose)

        # Publish detected marker poses
        if len(pose_array_msg.poses) > 0:
            self.marker_pose_publisher.publish(pose_array_msg)

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
