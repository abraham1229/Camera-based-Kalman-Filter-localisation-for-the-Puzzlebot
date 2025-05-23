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
        self.markerLength = 0.14  # 9cm marker size
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        self.parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)
        
        # Camera matrix and distortion coefficients for Gazebo (no distortion)
        # These values should match your Gazebo camera plugin settings
        self.camera_matrix = np.array([
            [528.4337, 0.0, 320.0],   # fx, 0, cx
            [0.0, 528.4338, 240.0],   # 0, fy, cy
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros((5, 1))  # No distortion in Gazebo
        
        # Known ArUco marker positions in the world (x, y coordinates)
        # Fill in the actual positions of your markers in the environment
        self.marker_positions = {
            0: (2.5 ,-0.5),    # Marker ID 0 at position (1.0, 0.0)
            1: (2.5 , 2.5),    # Marker ID 1 at position (0.0, 1.0)
            2: (-0.5, 2.5),   # etc.
            3: (-0.5,-0.5),
            4: (1.0 , 1.0),
            # Add more markers as needed
        }
        self.get_logger().info('ArucoDetectorNode started')

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
