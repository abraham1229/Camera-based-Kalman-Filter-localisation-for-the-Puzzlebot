import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import transforms3d
import math
import numpy as np
from tf2_ros import TransformBroadcaster

class Odometry_Node(Node):   

    def __init__(self):
        super().__init__('Odometria')

        # Declare parameters with default values
        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.declare_parameter('init_pose_yaw', 0.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter("use_linear_model", True)
        self.declare_parameter('motion_model_noise', False)

        # Set up subscriptions for wheel velocities
        self.subscription_velocity_left = self.create_subscription(
            Float32,
            'VelocityEncL',
            self.signal_callback_left,
            rclpy.qos.qos_profile_sensor_data)
        
        self.subscription_velocity_right = self.create_subscription(
            Float32,
            'VelocityEncR',
            self.signal_callback_right,
            rclpy.qos.qos_profile_sensor_data)
        
        # Subscribe to ArUco marker range and bearing data
        # This data now contains marker positions in odom frame coordinates
        self.subscription_aruco = self.create_subscription(
            Float32MultiArray,
            'aruco_range_bearing',
            self.aruco_callback,
            10)
        
        # Publishers
        self.pub_odometry = self.create_publisher(Odometry, 'odometria', 1000)
        
        # TF2 broadcaster for publishing odom -> base_link transform
        # This transform represents where the robot is relative to its odometry origin
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for main odometry loop at 100Hz
        self.timer_period = 0.01 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Physical robot parameters
        self.radius = 0.05    # Wheel radius in meters
        self.lenght = 0.19    # Distance between wheels in meters

        # Velocity variables from wheel encoders
        self.vel_left = 0.0   # Left wheel velocity
        self.vel_right = 0.0  # Right wheel velocity
        
        # Computed velocities
        self.velocidadTheta = 0.0  # Angular velocity
        self.velLineal = 0.0       # Linear velocity
        
        # Initialize covariance matrix (avoid singularities with small initial values)
        self.Sigma = 1e-3 * np.eye(3)

        # Process noise covariance matrix for the motion model
        # This represents uncertainty in the robot's motion
        self.covariance = 0.0001 * np.array([
            [0.1059, 0.1106, 0.1516],
            [0.1106, 1.7173, 0.6924],
            [0.1516, 0.6924, 1.5237]
        ])

        # Measurement noise covariance matrix for ArUco observations
        # This represents uncertainty in range and bearing measurements
        self.R = np.array([
            [0.1, 0],    # Range measurement noise variance
            [0, 0.1]     # Bearing measurement noise variance
        ])
        
        # Kalman filter variables for ArUco measurements
        self.have_new_aruco = False
        self.latest_aruco_measurement = np.zeros(2)  # [range, bearing]
        self.latest_landmark_pos = np.zeros(2)       # [x, y] in odom frame
        self.latest_aruco_id = -1

        # Motion model noise parameters (Probabilistic Robotics, Chapter 5)
        # These parameters control how much noise to add based on robot motion
        self.alpha = [0.05, 0.001, 0.05, 0.01, 0.01]
        
        # Normalized Innovation Squared (NIS) monitoring for filter consistency
        self.nis_values = []
        self.nis_window_size = 30
        self.chi2_lower = 0.103  # chi2 lower bound for 95% confidence, 2 DoF
        self.chi2_upper = 5.991  # chi2 upper bound for 95% confidence, 2 DoF

        # Robot state variables (position and orientation in odom frame)
        self.posX = self.get_parameter('init_pose_x').value
        self.posY = self.get_parameter('init_pose_y').value
        self.theta = self.get_parameter('init_pose_yaw').value
        
        # Get configuration parameters
        self.use_linear_model = self.get_parameter('use_linear_model').get_parameter_value().bool_value
        self.motion_noise = self.get_parameter('motion_model_noise').get_parameter_value().bool_value
        
        # Initialize state vector for Kalman filter
        self.s = np.array([self.posX, self.posY, self.theta])
        
        self.get_logger().info('Enhanced Odometry node initialized with TF2 support')

    def timer_callback(self):
        """
        Main odometry computation loop.
        
        This function:
        1. Computes robot velocities from wheel encoders
        2. Updates robot pose using motion model
        3. Applies Kalman filter corrections from ArUco measurements
        4. Publishes odometry message and TF transforms
        """
        # Compute robot velocities from differential drive kinematics
        self.velocidadTheta = self.radius * ((self.vel_right - self.vel_left) / self.lenght)
        self.velLineal = self.radius * ((self.vel_right + self.vel_left) / 2)  # Calibration factor
        
        # Update robot orientation
        self.theta += self.velocidadTheta * self.timer_period
                
        # Update robot position using kinematic model
        self.posX += self.velLineal * math.cos(self.theta) * self.timer_period
        self.posY += self.velLineal * math.sin(self.theta) * self.timer_period

        # Create Odometry message
        odom_msg = Odometry()

        if self.use_linear_model:
            # Use linearized Extended Kalman Filter (EKF) approach
            
            # Current state vector [x, y, theta]
            s = np.array([self.posX, self.posY, self.theta])
            u = np.array([self.velLineal, self.velocidadTheta])  # Control input

            # Predict step: Update state and covariance using motion model
            self.s, self.Sigma = self.linearized_state_update(s, u)
            
            # Correction step: Apply ArUco measurement if available
            if self.have_new_aruco:
                z = self.latest_aruco_measurement  # [range, bearing]
                landmark = self.latest_landmark_pos # [x, y] in odom frame
                
                # Apply Kalman filter correction using ArUco measurement
                self.kalman_filter(z, landmark)
                self.have_new_aruco = False
                
                # Update robot pose from corrected state
                self.posX, self.posY, self.theta = self.s
                
                self.get_logger().info(
                    f"After Kalman correction: x={self.s[0]:.3f}, y={self.s[1]:.3f}, "
                    f"θ={math.degrees(self.s[2]):.2f}°"
                )
            else:
                # Update robot pose from predicted state (no ArUco measurement)
                self.posX, self.posY, self.theta = self.s
                self.get_logger().info(
                        f"Without Kalman correction: x={self.s[0]:.3f}, y={self.s[1]:.3f}, "
                        f"θ={math.degrees(self.s[2]):.2f}°"
                    )
            
            # Set covariance in odometry message for visualization/debugging
            odom_msg.pose.covariance = [
                self.Sigma[0, 0], self.Sigma[0, 1], 0.0, 0.0, 0.0, self.Sigma[0, 2], # Row 1
                self.Sigma[1, 0], self.Sigma[1, 1], 0.0, 0.0, 0.0, self.Sigma[1, 2], # Row 2
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                                        # Row 3 (z)
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                                        # Row 4 (roll)
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                                        # Row 5 (pitch)
                self.Sigma[2, 0], self.Sigma[2, 1], 0.0, 0.0, 0.0, self.Sigma[2, 2]  # Row 6 (yaw)
            ]

        # Normalize theta to [-π, π] range
        self.theta = self.normalize_angle(self.theta)

        # Fill odometry message header
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.get_parameter('odom_frame').get_parameter_value().string_value.strip('/')
        
        # Set child frame (robot's base_link frame)
        ns = self.get_namespace().strip('/')
        odom_msg.child_frame_id = f'{ns}/base_link' if ns else 'base_link'

        # Set position in odom frame
        odom_msg.pose.pose.position.x = self.posX
        odom_msg.pose.pose.position.y = self.posY
        odom_msg.pose.pose.position.z = 0.0

        # Convert theta (yaw) to quaternion for orientation
        quat = transforms3d.euler.euler2quat(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[1]
        odom_msg.pose.pose.orientation.y = quat[2]
        odom_msg.pose.pose.orientation.z = quat[3]
        odom_msg.pose.pose.orientation.w = quat[0]

        # Set velocity information
        odom_msg.twist.twist.linear.x = self.velLineal      # Forward velocity
        odom_msg.twist.twist.angular.z = self.velocidadTheta # Angular velocity

        # Publish the Odometry message
        self.pub_odometry.publish(odom_msg)
        
        # Publish TF transform from odom to base_link
        # This transform tells ROS where the robot is relative to the odom frame origin
        #self.publish_odom_to_base_link_transform(odom_msg.header.stamp,  odom_msg.header.frame_id, odom_msg.child_frame_id)

    def publish_odom_to_base_link_transform(self, timestamp, header_frame_id, child_frame_id):
        """
        Publish the transform from odom frame to base_link frame.
        
        This transform is essential for the TF tree and represents:
        "Where is the robot (base_link) relative to the odometry origin (odom)?"
        
        Args:
            timestamp: Current timestamp for the transform
            child_frame_id: The robot's base_link frame name
        """
        transform = TransformStamped()
        
        # Set up transform header
        transform.header.stamp = timestamp
        transform.header.frame_id = header_frame_id  # Parent frame
        transform.child_frame_id = child_frame_id  # Child frame (base_link)
        
        # Set translation (robot's position in odom frame)
        transform.transform.translation.x = self.posX
        transform.transform.translation.y = self.posY
        transform.transform.translation.z = 0.0
        
        # Set rotation (robot's orientation in odom frame)
        quat = transforms3d.euler.euler2quat(0, 0, self.theta)
        transform.transform.rotation.x = quat[1]
        transform.transform.rotation.y = quat[2]
        transform.transform.rotation.z = quat[3]
        transform.transform.rotation.w = quat[0]
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

    def signal_callback_left(self, msg):
        """Process left wheel velocity data from encoder."""
        if msg is not None:
            self.vel_left = msg.data if abs(msg.data) > 1e-3 else 0.0

    def signal_callback_right(self, msg):
        """Process right wheel velocity data from encoder."""
        if msg is not None:
            self.vel_right = msg.data if abs(msg.data) > 1e-3 else 0.0

    def normalize_angle(self, theta):
        """Normalize angle to [-π, π] range."""
        return (theta + math.pi) % (2 * math.pi) - math.pi
    
    def noise(self, v, w):
        """
        Compute motion-dependent process noise covariance.
        
        This function implements the noise model from Probabilistic Robotics
        where process noise depends on the commanded velocities.
        
        Args:
            v: Linear velocity
            w: Angular velocity
            
        Returns:
            3x3 process noise covariance matrix
        """
        # Linear velocity variance depends on both velocities
        var_v = (self.alpha[0] * v**2) + (self.alpha[1] * w**2)
        # Angular velocity variance depends on both velocities
        var_w = (self.alpha[2] * v**2) + (self.alpha[3] * w**2)
        # Cross-correlation term
        var_vw = self.alpha[4] * v * w  

        return np.array([
            [var_v, 0, var_vw],
            [0, var_v, var_vw],
            [var_vw, var_vw, var_w]
        ])
    
    def aruco_callback(self, msg):
        """
        Process ArUco marker data from the ArUco detector node.
        
        The marker positions are now in odom frame coordinates thanks to
        the coordinate transformation performed in the ArUco detector node.
        
        Message format: [marker_id, range, bearing, marker_x_odom, marker_y_odom]
        """
        if len(msg.data) != 5:
            self.get_logger().error(f"Invalid ArUco data format. Expected 5 values, got {len(msg.data)}")
            return
            
        # Extract data from message
        marker_id = int(msg.data[0])
        range_to_marker = msg.data[1]
        bearing_to_marker = msg.data[2]
        marker_x_odom = msg.data[3]  # Marker X position in odom frame
        marker_y_odom = msg.data[4]  # Marker Y position in odom frame
        
        # Store the measurement for use in the Kalman filter
        # The key improvement: marker positions are now in odom frame coordinates
        self.latest_aruco_measurement = np.array([range_to_marker, bearing_to_marker])
        self.latest_landmark_pos = np.array([marker_x_odom, marker_y_odom])
        self.latest_aruco_id = marker_id
        self.have_new_aruco = True
        
        # Log received ArUco data for debugging
        self.get_logger().debug(
            f"Received ArUco marker {marker_id}: range={range_to_marker:.2f}m, "
            f"bearing={math.degrees(bearing_to_marker):.1f}°, "
            f"odom_position=({marker_x_odom:.2f}, {marker_y_odom:.2f})"
        )
    
    def linearized_state_update(self, s, u):
        """
        Prediction step of the Extended Kalman Filter using linearized motion model.
        
        This function implements the EKF prediction equations:
        1. Linearize the nonlinear motion model around current state
        2. Predict new state using the linearized model
        3. Update covariance using Jacobian matrices
        
        Args:
            s: Current state vector [x, y, theta]
            u: Control input vector [v, w]
            
        Returns:
            tuple: (predicted_state, predicted_covariance)
        """
        # Compute Jacobian matrices for linearization
        # A_k represents how the state changes with respect to the previous state
        A_k = np.array([
            [1, 0, -u[0] * math.sin(self.theta) * self.timer_period],
            [0, 1,  u[0] * math.cos(self.theta) * self.timer_period],
            [0, 0, 1]
        ])

        # B_k represents how the state changes with respect to the control input
        B_k = np.array([
            [math.cos(self.theta) * self.timer_period, 0],
            [math.sin(self.theta) * self.timer_period, 0],
            [0, self.timer_period]
        ])

        # Predict new state using linearized motion model
        s_new = A_k @ s + B_k @ u
        # Normalize heading angle
        s_new[2] = self.normalize_angle(s_new[2])

        # Predict covariance using linearized covariance propagation
        if self.motion_noise:
            # Add motion-dependent process noise if enabled
            Q = self.noise(u[0], u[1])
            Sigma_new = A_k @ self.Sigma @ A_k.T + B_k @ Q @ B_k.T + self.covariance
        else:
            # Use constant process noise
            Sigma_new = A_k @ self.Sigma @ A_k.T + self.covariance
            
        return s_new, Sigma_new
    
    def kalman_filter(self, z, landmark_pos):
        """
        Update step of the Extended Kalman Filter using ArUco measurements.
        
        This function implements the EKF correction equations:
        1. Compute expected measurement based on current state estimate
        2. Calculate innovation (difference between actual and expected measurement)
        3. Update state and covariance using Kalman gain
        
        The key improvement: landmark_pos is now in odom frame coordinates,
        ensuring consistency with the robot's state estimation.
        
        Args:
            z (np.array): Measurement vector [range, bearing] from camera
            landmark_pos (np.array): Landmark position [x, y] in odom frame
        """
        # Compute relative position from robot to landmark in odom frame
        dx = landmark_pos[0] - self.s[0]  # Difference in X
        dy = landmark_pos[1] - self.s[1]  # Difference in Y
        theta = self.s[2]                 # Robot's current heading
        
        # Avoid division by zero in range calculation
        eps = 1e-6
        p = max(dx**2 + dy**2, eps)  # Squared distance to landmark

        # Compute expected measurement based on current state estimate
        # This is what we would expect to measure if our state estimate is correct
        z_hat = np.array([
            np.sqrt(p),                           # Expected range
            np.arctan2(dy, dx) - theta           # Expected bearing (relative to robot heading)
        ])
        z_hat[1] = self.normalize_angle(z_hat[1])

        # Compute measurement Jacobian (how measurement changes with state)
        # This linearizes the nonlinear measurement model around current state
        G = np.array([
            [-dx / np.sqrt(p), -dy / np.sqrt(p), 0],  # Range derivatives
            [ dy / p,          -dx / p,         -1]   # Bearing derivatives
        ])

        # Compute innovation covariance (uncertainty in innovation)
        S = G @ self.Sigma @ G.T + self.R

        # Compute Kalman gain (optimal weighting between prediction and measurement)
        K = self.Sigma @ G.T @ np.linalg.inv(S)

        # Compute innovation (difference between actual and expected measurement)
        innovation = z - z_hat
        innovation[1] = self.normalize_angle(innovation[1])  # Normalize bearing innovation

        # Calculate Normalized Innovation Squared (NIS) for filter monitoring
        nis = innovation.T @ np.linalg.inv(S) @ innovation
        self.nis_values.append(nis)
        if len(self.nis_values) > self.nis_window_size:
            self.nis_values.pop(0)

        # Monitor filter consistency using NIS
        self.monitor_consistency()

        # Update state estimate using Kalman gain and innovation
        self.s += K @ innovation
        # Normalize heading angle
        self.s[2] = self.normalize_angle(self.s[2])

        # Update covariance using Joseph form (numerically stable)
        I = np.eye(3)
        self.Sigma = (I - K @ G) @ self.Sigma @ (I - K @ G).T + K @ self.R @ K.T

    def monitor_consistency(self):
        """
        Monitor Extended Kalman Filter consistency using NIS statistics.
        
        This function checks if the filter is performing well by analyzing
        the Normalized Innovation Squared (NIS) values over a sliding window.
        
        Proper NIS values should follow a chi-squared distribution.
        If too many values are outside the expected bounds, it indicates:
        - Filter overconfidence (NIS too high): Model uncertainties underestimated
        - Filter underconfidence (NIS too low): Model uncertainties overestimated
        """
        if len(self.nis_values) < self.nis_window_size:
            return

        # Count NIS values outside expected bounds
        too_high = sum(1 for nis in self.nis_values if nis > self.chi2_upper)
        too_low = sum(1 for nis in self.nis_values if nis < self.chi2_lower)

        high_percent = too_high / len(self.nis_values) * 100
        low_percent = too_low / len(self.nis_values) * 100

        # Log warnings if filter consistency is poor
        if high_percent > 15:  # More than 15% above upper bound
            self.get_logger().warn(
                f"Filter appears overconfident (NIS too high): {high_percent:.1f}% > 15%"
            )
        elif low_percent > 15:  # More than 15% below lower bound
            self.get_logger().warn(
                f"Filter appears underconfident (NIS too low): {low_percent:.1f}% > 15%"
            )

def main(args=None):
    """Main function to run the enhanced odometry node."""
    rclpy.init(args=args)
    node = Odometry_Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()