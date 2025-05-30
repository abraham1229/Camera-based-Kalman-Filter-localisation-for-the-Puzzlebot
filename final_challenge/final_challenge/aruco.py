import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
from cv2 import aruco
from collections import deque
import time

from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header

class SmoothedArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco')

        # Calibración hardcodeada
        self.K = np.array([
            [776.88108, 0.0, 662.6974],
            [0.0, 777.29396, 406.37559],
            [0.0, 0.0, 1.0]
        ])
        
        # Distortion coefficients D (plumb_bob model)
        self.D = np.array([-0.33888, 0.116192, 0.000114, -0.001292, 0.0])

        self.markerLength = 0.082

        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)

        self.bridge = CvBridge()

        # Smoothing parameters
        self.window_size = 10  # Size of moving average window
        self.smoothing_method = "kalman"  # Options: "moving_average", "exponential", "kalman", "median"
        
        # Data storage for each marker ID
        self.marker_data = {}  # Dictionary to store data for each marker
        
        # Exponential moving average parameters
        self.alpha = 0.2  # Smoothing factor (0 < alpha < 1)
        
        # Kalman filter parameters
        self.kalman_filters = {}

        self.subscription_image = self.create_subscription(
            Image,
            '/video_source/raw',
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data)

        self.publisher_aruco = self.create_publisher(
            ArucoDetection,
            '/aruco_detections',
            rclpy.qos.qos_profile_sensor_data)

        self.get_logger().info(f"Aruco detector node started with {self.smoothing_method} smoothing")

    def initialize_marker_data(self, marker_id):
        """Initialize data structures for a new marker ID"""
        if marker_id not in self.marker_data:
            self.marker_data[marker_id] = {
                'distances': deque(maxlen=self.window_size),
                'positions': deque(maxlen=self.window_size),
                'last_ema_distance': None,
                'last_ema_position': None,
                'last_time': time.time()
            }
            
            # Initialize Kalman filter for this marker
            self.initialize_kalman_filter(marker_id)

    def initialize_kalman_filter(self, marker_id):
        """Initialize a simple 1D Kalman filter for distance smoothing"""
        if marker_id not in self.kalman_filters:
            # State: [distance, velocity]
            # Measurement: distance
            kalman = cv.KalmanFilter(2, 1)
            
            # Transition matrix (A)
            dt = 0.033  # Assuming ~30fps
            kalman.transitionMatrix = np.array([[1., dt], [0., 1.]], dtype=np.float32)
            
            # Measurement matrix (H)
            kalman.measurementMatrix = np.array([[1., 0.]], dtype=np.float32)
            
            # Process noise covariance (Q)
            kalman.processNoiseCov = np.array([[0.01, 0.], [0., 0.01]], dtype=np.float32)
            
            # Measurement noise covariance (R)
            kalman.measurementNoiseCov = np.array([[0.1]], dtype=np.float32)
            
            # Error covariance matrix (P)
            kalman.errorCovPost = np.array([[1., 0.], [0., 1.]], dtype=np.float32)
            
            self.kalman_filters[marker_id] = kalman

    def smooth_distance_moving_average(self, marker_id, distance):
        """Apply moving average smoothing to distance"""
        data = self.marker_data[marker_id]
        data['distances'].append(distance)
        
        if len(data['distances']) >= 2:  # Need at least 2 points
            return np.mean(data['distances'])
        return distance

    def smooth_distance_exponential(self, marker_id, distance):
        """Apply exponential moving average smoothing to distance"""
        data = self.marker_data[marker_id]
        
        if data['last_ema_distance'] is None:
            data['last_ema_distance'] = distance
            return distance
        
        # EMA formula: S(t) = α * X(t) + (1-α) * S(t-1)
        smoothed = self.alpha * distance + (1 - self.alpha) * data['last_ema_distance']
        data['last_ema_distance'] = smoothed
        return smoothed

    def smooth_distance_median(self, marker_id, distance):
        """Apply median filter smoothing to distance"""
        data = self.marker_data[marker_id]
        data['distances'].append(distance)
        
        if len(data['distances']) >= 3:  # Need at least 3 points for meaningful median
            return np.median(data['distances'])
        return distance

    def smooth_distance_kalman(self, marker_id, distance):
        """Apply Kalman filter smoothing to distance"""
        kalman = self.kalman_filters[marker_id]
        
        # Predict
        prediction = kalman.predict()
        
        # Update with measurement
        measurement = np.array([[distance]], dtype=np.float32)
        kalman.correct(measurement)
        
        return float(kalman.statePost[0])

    def smooth_position(self, marker_id, position):
        """Apply smoothing to position (x, y, z)"""
        data = self.marker_data[marker_id]
        
        # Store position as numpy array for easier processing
        pos_array = np.array([position.x, position.y, position.z])
        data['positions'].append(pos_array)
        
        if self.smoothing_method == "moving_average" and len(data['positions']) >= 2:
            smoothed_pos = np.mean(data['positions'], axis=0)
        elif self.smoothing_method == "exponential":
            if data['last_ema_position'] is None:
                data['last_ema_position'] = pos_array
                smoothed_pos = pos_array
            else:
                smoothed_pos = self.alpha * pos_array + (1 - self.alpha) * data['last_ema_position']
                data['last_ema_position'] = smoothed_pos
        elif self.smoothing_method == "median" and len(data['positions']) >= 3:
            smoothed_pos = np.median(data['positions'], axis=0)
        else:
            smoothed_pos = pos_array
        
        # Convert back to Point message
        smoothed_point = Point()
        smoothed_point.x = float(smoothed_pos[0])
        smoothed_point.y = float(smoothed_pos[1])
        smoothed_point.z = float(smoothed_pos[2])
        
        return smoothed_point

    def apply_smoothing(self, marker_id, distance):
        """Apply the selected smoothing method to distance"""
        if self.smoothing_method == "moving_average":
            return self.smooth_distance_moving_average(marker_id, distance)
        elif self.smoothing_method == "exponential":
            return self.smooth_distance_exponential(marker_id, distance)
        elif self.smoothing_method == "median":
            return self.smooth_distance_median(marker_id, distance)
        elif self.smoothing_method == "kalman":
            return self.smooth_distance_kalman(marker_id, distance)
        else:
            return distance

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        markerCorners, markerIds, rejected = self.detector.detectMarkers(gray)

        # Preparar mensaje ArucoDetection
        detection_msg = ArucoDetection()
        detection_msg.header = msg.header
        detection_msg.markers = []

        if markerIds is not None and len(markerIds) > 0:
            aruco.drawDetectedMarkers(img, markerCorners, markerIds)
            for i, marker_id in enumerate(markerIds.flatten()):
                # Initialize data for new markers
                self.initialize_marker_data(int(marker_id))
                
                # Pose estimation (en el frame de la cámara)
                rvec, tvec = aruco.estimatePoseSingleMarkers(
                    markerCorners[i], self.markerLength, self.K, self.D)[:2]
                cv.drawFrameAxes(img, self.K, self.D, rvec, tvec, 0.01)

                # Raw distance calculation
                raw_distance = np.linalg.norm(tvec[0][0])
                
                # Apply smoothing to distance
                smoothed_distance = self.apply_smoothing(int(marker_id), raw_distance)

                # Create position point
                raw_position = Point()
                raw_position.x = float(tvec[0][0][0])
                raw_position.y = float(tvec[0][0][1])
                raw_position.z = float(tvec[0][0][2])
                
                # Apply smoothing to position
                smoothed_position = self.smooth_position(int(marker_id), raw_position)

                # Armar el MarkerPose con datos suavizados
                marker_pose = MarkerPose()
                marker_pose.marker_id = int(marker_id)
                
                # Use smoothed position
                marker_pose.pose = Pose()
                marker_pose.pose.position = smoothed_position
                
                # Orientación en quaternion (a partir de rvec) - sin suavizar por ahora
                quat = self.rvec_to_quaternion(rvec[0][0])
                orientation = Quaternion()
                orientation.x = quat[0]
                orientation.y = quat[1]
                orientation.z = quat[2]
                orientation.w = quat[3]
                marker_pose.pose.orientation = orientation

                detection_msg.markers.append(marker_pose)

                # Visual feedback with both raw and smoothed values
                cX = int((markerCorners[i][0][0][0] + markerCorners[i][0][2][0]) / 2.0)
                cY = int((markerCorners[i][0][0][1] + markerCorners[i][0][2][1]) / 2.0)
                
                self.get_logger().info(
                    f"Id={int(marker_id)}, Raw={raw_distance:.3f}m, Smoothed={smoothed_distance:.3f}m"
                )
                
                # Display smoothed distance
                cv.putText(img, f"{smoothed_distance:.2f} m", (cX, cY - 15),
                           cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)
                
                # Optionally display raw distance for comparison
                cv.putText(img, f"Raw: {raw_distance:.2f} m", (cX, cY - 35),
                           cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)

        # Publicar detección (aunque esté vacío, es importante para odometry)
        self.publisher_aruco.publish(detection_msg)

        # Visualización local (uncomment if needed)
        # cv.imshow("Aruco Detection", img)
        # key = cv.waitKey(1)
        # if key == 27:
        #     self.get_logger().info("ESC pressed, closing visualization window.")
        #     cv.destroyAllWindows()

    def rvec_to_quaternion(self, rvec):
        """Convierte un vector de rotación Rodrigues (rvec) a quaternion (x, y, z, w)"""
        R, _ = cv.Rodrigues(rvec)
        qw = np.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) / 2.0
        qx = (R[2, 1] - R[1, 2]) / (4.0 * qw)
        qy = (R[0, 2] - R[2, 0]) / (4.0 * qw)
        qz = (R[1, 0] - R[0, 1]) / (4.0 * qw)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = SmoothedArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()