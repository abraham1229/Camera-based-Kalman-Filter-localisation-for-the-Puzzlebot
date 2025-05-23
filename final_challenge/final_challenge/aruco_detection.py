import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import math
from geometry_msgs.msg import PoseArray, Pose, TransformStamped, PointStamped
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
        
        # Robot's initial position in world frame (should match your spawn position)
        # This represents where the robot starts in the world coordinate system
        self.robot_initial_world_x = self.get_parameter('init_pose_x').value
        self.robot_initial_world_y = self.get_parameter('init_pose_y').value
        self.robot_initial_world_yaw = self.get_parameter('init_pose_yaw').value
        
        # Publish static transform from world to odom frame
        # This transform represents the offset between the world origin and where
        # the robot considers its odometry origin to be

        #self.publish_world_to_odom_transform()
        
        self.get_logger().info('ArucoDetectorNode started with TF2 support')

    def publish_world_to_odom_transform(self):
        """
        Publish a static transform from world to odom frame.
        
        This transform is crucial because:
        1. ArUco markers have known positions in the 'world' frame
        2. Robot odometry operates in the 'odom' frame (starting from robot's initial position)
        3. We need to transform ArUco measurements from world coordinates to odom coordinates
        
        The transform represents: "Where is the odom frame origin relative to the world frame?"
        Since the robot starts at (1.5, 1.5) in world but considers itself at (0, 0) in odom,
        the odom frame origin is at the robot's starting position in world coordinates.
        """
        static_transform = TransformStamped()
        
        # Set up the transform header
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'world'  # Parent frame
        static_transform.child_frame_id = 'odom'    # Child frame
        
        static_transform.transform.translation.x = self.robot_initial_world_x
        static_transform.transform.translation.y = self.robot_initial_world_y
        static_transform.transform.translation.z = 0.0
        
        # Convert initial yaw to quaternion for rotation
        quat = transforms3d.euler.euler2quat(0, 0, self.robot_initial_world_yaw)
        static_transform.transform.rotation.x = quat[1]
        static_transform.transform.rotation.y = quat[2]
        static_transform.transform.rotation.z = quat[3]
        static_transform.transform.rotation.w = quat[0]
        
        # Publish the static transform
        self.static_tf_broadcaster.sendTransform(static_transform)
        

    def transform_marker_position_to_odom(self, marker_world_x, marker_world_y):
        """
        Transform a marker's world coordinates to odom frame coordinates.
        
        This function performs the crucial coordinate transformation:
        1. Takes marker position in world frame
        2. Converts it to odom frame coordinates
        3. Returns the transformed position for use in the Kalman filter
        
        Args:
            marker_world_x: Marker's X position in world frame
            marker_world_y: Marker's Y position in world frame
            
        Returns:
            tuple: (marker_odom_x, marker_odom_y) - Marker position in odom frame
        """
        try:
            # Create a point in the world frame
            point_world = PointStamped()
            point_world.header.frame_id = 'world'
            point_world.header.stamp = self.get_clock().now().to_msg()
            point_world.point.x = marker_world_x
            point_world.point.y = marker_world_y
            point_world.point.z = 0.0
            
            # Transform the point from world frame to odom frame
            # This accounts for the offset between world origin and odom origin
            point_odom = self.tf_buffer.transform(point_world, 'world', timeout=rclpy.duration.Duration(seconds=1.0))
            
            return point_odom.point.x, point_odom.point.y
            
        except Exception as e:
            self.get_logger().error(f"Failed to transform marker position: {e}")
            # Fallback: manual transformation if TF fails
            # This is a simple translation assuming no rotation between frames
            marker_odom_x = marker_world_x - self.robot_initial_world_x
            marker_odom_y = marker_world_y - self.robot_initial_world_y
            self.get_logger().warn(f"Using manual transformation fallback")
            return marker_odom_x, marker_odom_y

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
                
                # Estimate pose of the marker relative to camera
                rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
                    [marker_corner], 
                    self.markerLength, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                
                # Draw axis for each marker
                cv.drawFrameAxes(img, self.camera_matrix, self.dist_coeffs, rvecs, tvecs, 0.05)
                
                # Calculate range (distance) to the marker in camera frame
                range_to_marker = np.linalg.norm(tvecs)
                
                # Calculate bearing (angle) to the marker in camera frame
                # Note: This is the angle from camera's forward direction to the marker
                bearing_to_marker = math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                
                # Add text showing range and bearing
                cv.putText(img, f"ID: {marker_id}", (cX, cY - 15), 
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv.putText(img, f"Range: {range_to_marker:.2f}m", (cX, cY + 15), 
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv.putText(img, f"Bearing: {math.degrees(bearing_to_marker):.1f}°", (cX, cY + 30), 
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Check if this marker's position is known in world frame
                if marker_id in self.marker_positions_world:
                    # Get the marker's position in the world frame
                    marker_world_x, marker_world_y = self.marker_positions_world[marker_id]
                    
                    # Transform marker position from world frame to odom frame
                    # This is the key transformation that accounts for the robot's starting position
                    
                    #marker_odom_x, marker_odom_y = self.transform_marker_position_to_odom(
                    #    marker_world_x, marker_world_y)
                    
                    # Create range and bearing message for the odometry node
                    # Now the marker position is in odom frame coordinates, which matches
                    # the coordinate system used by the robot's odometry and Kalman filter
                    range_bearing_msg = Float32MultiArray()
                    range_bearing_msg.data = [
                        float(marker_id),                  # Marker ID
                        float(range_to_marker),            # Range to marker (from camera)
                        float(bearing_to_marker),          # Bearing to marker (from camera)
                        float(marker_world_x), #marker_odom_x # Marker X position in ODOM frame
                        float(marker_world_y)  #marker_odom_y # Marker Y position in ODOM frame
                    ]
                    
                    # Publish range and bearing for odometry node
                    self.range_bearing_publisher.publish(range_bearing_msg)
                    
                    # Log detection with both world and odom coordinates for debugging
                    self.get_logger().info(
                        f"Detected marker {marker_id}: range={range_to_marker:.2f}m, "
                        f"bearing={math.degrees(bearing_to_marker):.1f}°, "
                        f"world_pos=({marker_world_x:.2f}, {marker_world_y:.2f}), "
                        #f"odom_pos=({marker_odom_x:.2f}, {marker_odom_y:.2f})"
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