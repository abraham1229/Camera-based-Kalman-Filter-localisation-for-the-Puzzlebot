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

        # Declare parameters with default values for initial pose and frame names
        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.declare_parameter('init_pose_yaw', 0.0)
        self.declare_parameter('odom_frame', 'world')  # Parent frame for odometry
        self.declare_parameter("use_linear_model", True) # Flag to enable EKF
        self.declare_parameter('motion_model_noise', True) # Flag to enable motion-dependent noise
        self.declare_parameter('base_frame', 'base_footprint') # Child frame for odometry and TF

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
        # This data is assumed to contain marker positions in world frame coordinates
        self.subscription_aruco = self.create_subscription(
            Float32MultiArray,
            'aruco_range_bearing',
            self.aruco_callback,
            10) # Queue size for ArUco messages

        # Publishers
        self.pub_odometry = self.create_publisher(Odometry, 'odometria', 1000) # Odometry message publisher

        # TF2 broadcaster for publishing world -> base_footprint transform
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for main odometry loop (EKF update rate)
        self.timer_period = 0.01 # 100 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Physical robot parameters
        self.radius = 0.05    # Wheel radius in meters
        self.lenght = 0.19    # Distance between wheels in meters (axle length)

        # Velocity variables from wheel encoders (updated by callbacks)
        self.vel_left = 0.0   # Left wheel velocity
        self.vel_right = 0.0  # Right wheel velocity

        # Computed velocities (used in motion model)
        self.velocidadTheta = 0.0  # Angular velocity (yaw rate)
        self.velLineal = 0.0       # Linear velocity (forward speed)

        # Initialize state vector for Kalman filter: [x, y, theta]
        # This will be the single source of truth for the robot's pose
        self.s = np.array([
            self.get_parameter('init_pose_x').value,
            self.get_parameter('init_pose_y').value,
            self.get_parameter('init_pose_yaw').value
        ])

        # Initialize covariance matrix (P) for the state [x, y, theta]
        # Small initial values indicate high confidence in initial pose
        self.Sigma = 1e-3 * np.eye(3)

        # Process noise covariance matrix (Q) for the motion model
        # This will be used if motion_model_noise is False (constant noise)
        # If motion_model_noise is True, 'self.noise' function will compute Q dynamically.
        # This matrix is now only a fallback/base noise if motion_model_noise is off.
        self.Q_constant = 0.0001 * np.array([
            [0.1059, 0.1106, 0.1516],
            [0.1106, 1.7173, 0.6924],
            [0.1516, 0.6924, 1.5237]
        ])

        # Measurement noise covariance matrix (R) for ArUco observations [range, bearing]
        # Tune these values based on your sensor's actual noise characteristics
        self.R = np.array([
            [0.1**2, 0],           # Variance for range measurement (m^2)
            [0, np.deg2rad(2)**2]  # Variance for bearing measurement (rad^2)
        ])
# ...existing code...

        # Kalman filter variables for ArUco measurements
        self.have_new_aruco = False # Flag to indicate new measurement is available
        self.latest_aruco_measurement = np.zeros(2)  # [range, bearing] from camera
        self.latest_landmark_pos = np.zeros(2)       # [x, y] in world frame (from ArUco message)
        self.latest_aruco_id = -1 # ID of the detected marker

        # Motion model noise parameters (alpha values from Probabilistic Robotics)
        # These control how much noise is added based on linear and angular velocities
        self.alpha = [0.05, 0.001, 0.05, 0.01, 0.01]

        # Normalized Innovation Squared (NIS) monitoring for filter consistency
        self.nis_values = [] # Stores recent NIS values
        self.nis_window_size = 30 # Number of NIS values to keep for analysis
        # Chi-squared bounds for 95% confidence, 2 degrees of freedom (for 2D measurement)
        self.chi2_lower = 0.103
        self.chi2_upper = 5.991

        # Get configuration parameters
        self.use_linear_model = self.get_parameter('use_linear_model').get_parameter_value().bool_value
        self.motion_noise = self.get_parameter('motion_model_noise').get_parameter_value().bool_value

        # Get frame names from parameters and handle namespace prefixing
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        ns = self.get_namespace().strip('/')
        if ns:
            self.odom_frame = f'{ns}/{self.odom_frame}' if not self.odom_frame.startswith(ns) else self.odom_frame
            self.base_frame = f'{ns}/{self.base_frame}' if not self.base_frame.startswith(ns) else self.base_frame

        # Store last update time for dynamic dt calculation
        self.last_update_time = self.get_clock().now()

        self.get_logger().info(f'Odometry node initialized with EKF. Odom frame: {self.odom_frame}, Base frame: {self.base_frame}')


    def timer_callback(self):
        # Calculate time elapsed since last update
        current_time_ros = self.get_clock().now()
        dt = (current_time_ros - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = current_time_ros

        # Compute robot velocities from differential drive kinematics
        # These are the control inputs (u) for the motion model
        self.velocidadTheta = self.radius * ((self.vel_right - self.vel_left) / self.lenght)
        self.velLineal = self.radius * ((self.vel_right + self.vel_left) / 2)

        # Control input vector [linear_velocity, angular_velocity]
        u = np.array([self.velLineal, self.velocidadTheta])

        # EKF Prediction Step
        # The state 'self.s' is updated directly by linearized_state_update
        self.s, self.Sigma = self.linearized_state_update(self.s, u, dt)

        # EKF Correction Step (if a new ArUco measurement is available)
        if self.use_linear_model and self.have_new_aruco:
            z = self.latest_aruco_measurement  # [range, bearing]
            landmark = self.latest_landmark_pos # [x, y] in world frame

            # Apply Kalman filter correction using ArUco measurement
            self.kalman_filter(z, landmark)
            self.have_new_aruco = False # Reset flag after using measurement

            self.get_logger().info(
                f"After Kalman correction: x={self.s[0]:.3f}, y={self.s[1]:.3f}, "
                f"θ={math.degrees(self.s[2]):.2f}°"
            )
        elif self.use_linear_model:
            # Log predicted state if no correction was applied
            self.get_logger().info(
                f"Without Kalman correction (predicted): x={self.s[0]:.3f}, y={self.s[1]:.3f}, "
                f"θ={math.degrees(self.s[2]):.2f}°"
            )
        else:
            # If EKF is not used, perform simple dead reckoning integration
            # This path is typically for debugging or simpler applications
            self.s[2] += self.velocidadTheta * dt
            self.s[0] += self.velLineal * math.cos(self.s[2]) * dt
            self.s[1] += self.velLineal * math.sin(self.s[2]) * dt
            self.s[2] = self.normalize_angle(self.s[2]) # Normalize angle
            self.get_logger().info(
                f"Simple Dead Reckoning: x={self.s[0]:.3f}, y={self.s[1]:.3f}, "
                f"θ={math.degrees(self.s[2]):.2f}°"
            )

        # Update self.posX, self.posY, self.theta from the EKF's state vector
        # This ensures consistency for publishing and TF broadcasting
        self.posX = self.s[0]
        self.posY = self.s[1]
        self.theta = self.s[2] # Already normalized by EKF steps or direct update

        current_time_ros_msg = current_time_ros.to_msg() # Convert ROS time to message format
        # Create and publish Odometry message
        odom_msg = self.create_odometry_message(current_time_ros_msg)
        self.pub_odometry.publish(odom_msg)

        # Publish TF transform from world to base_footprint
        self.publish_world_to_base_transform(current_time_ros_msg)

    def create_odometry_message(self, timestamp):
        """
        Creates and populates an Odometry message with the current state and covariance.
        """
        odom_msg = Odometry()

        # Fill odometry message header
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = self.odom_frame  # Parent frame (e.g., 'world')
        odom_msg.child_frame_id = self.base_frame   # Child frame (e.g., 'base_footprint')

        # Set position in world frame from the EKF state
        odom_msg.pose.pose.position.x = self.posX
        odom_msg.pose.pose.position.y = self.posY
        odom_msg.pose.pose.position.z = 0.0 # Assuming 2D motion

        # Convert theta (yaw) to quaternion for orientation
        quat = transforms3d.euler.euler2quat(0, 0, self.theta)
        odom_msg.pose.pose.orientation.w = quat[0]
        odom_msg.pose.pose.orientation.x = quat[1]
        odom_msg.pose.pose.orientation.y = quat[2]
        odom_msg.pose.pose.orientation.z = quat[3]

        # Set velocity information in base_footprint frame
        odom_msg.twist.twist.linear.x = self.velLineal      # Forward velocity
        odom_msg.twist.twist.linear.y = 0.0                 # No sideways motion
        odom_msg.twist.twist.linear.z = 0.0                 # No vertical motion
        odom_msg.twist.twist.angular.x = 0.0                # No roll rate
        odom_msg.twist.twist.angular.y = 0.0                # No pitch rate
        odom_msg.twist.twist.angular.z = self.velocidadTheta # Yaw rate

        # Set covariance matrices if using EKF
        if self.use_linear_model:
            # Pose covariance (6x6 matrix flattened to 36 elements)
            # Map the 3x3 state covariance (x, y, theta) to the 6x6 Odometry covariance
            # The order is: x, y, z, roll, pitch, yaw
            odom_msg.pose.covariance = [
                self.Sigma[0, 0], self.Sigma[0, 1], 0.0, 0.0, 0.0, self.Sigma[0, 2], # Row for x
                self.Sigma[1, 0], self.Sigma[1, 1], 0.0, 0.0, 0.0, self.Sigma[1, 2], # Row for y
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                                        # Row for z (zero for 2D)
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                                        # Row for roll (zero for 2D)
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                                        # Row for pitch (zero for 2D)
                self.Sigma[2, 0], self.Sigma[2, 1], 0.0, 0.0, 0.0, self.Sigma[2, 2]  # Row for yaw
            ]

            # Twist covariance (6x6 matrix flattened to 36 elements)
            # This represents the uncertainty in the linear and angular velocities.
            # For a differential drive, linear.x and angular.z are the primary components.
            # You might need to tune these values or derive them more rigorously.
            odom_msg.twist.covariance = [
                0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  # Variance for linear.x (forward velocity)
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Variance for linear.y (zero for differential drive)
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Variance for linear.z
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Variance for angular.x (roll rate)
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Variance for angular.y (pitch rate)
                0.0, 0.0, 0.0, 0.0, 0.0, 0.01  # Variance for angular.z (yaw rate)
            ]
            # A more advanced approach would involve calculating this from wheel encoder noise and kinematics.
            # For now, small constant values are used.

        return odom_msg

    def publish_world_to_base_transform(self, timestamp):
        """
        Publish the transform from world frame to base_footprint frame.
        This transform is crucial for your TF tree, positioning the robot in the world.
        """
        transform = TransformStamped()

        # Set up transform header
        transform.header.stamp = timestamp
        transform.header.frame_id = self.odom_frame    # Parent frame (e.g., 'world')
        transform.child_frame_id = self.base_frame     # Child frame (e.g., 'base_footprint')

        # Set translation (robot's position in world frame from EKF state)
        transform.transform.translation.x = self.posX
        transform.transform.translation.y = self.posY
        transform.transform.translation.z = 0.0 # Assuming 2D motion

        # Set rotation (robot's orientation in world frame from EKF state)
        quat = transforms3d.euler.euler2quat(0, 0, self.theta)
        transform.transform.rotation.w = quat[0]
        transform.transform.rotation.x = quat[1]
        transform.transform.rotation.y = quat[2]
        transform.transform.rotation.z = quat[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

    def signal_callback_left(self, msg):
        """Process left wheel velocity data from encoder."""
        if msg is not None:
            # Filter out very small values that might be noise or zero
            self.vel_left = msg.data if abs(msg.data) > 1e-3 else 0.0

    def signal_callback_right(self, msg):
        """Process right wheel velocity data from encoder."""
        if msg is not None:
            # Filter out very small values that might be noise or zero
            self.vel_right = msg.data if abs(msg.data) > 1e-3 else 0.0

    def normalize_angle(self, theta):
        """Normalize angle to [-π, π] range."""
        return (theta + math.pi) % (2 * math.pi) - math.pi

    def noise(self, v, w):
        """
        Computes the motion noise covariance matrix Q based on robot velocities.
        This follows the model from Probabilistic Robotics, Chapter 5.
        """
        # Linear velocity variance depends on both linear and angular velocities
        var_v = (self.alpha[0] * v**2) + (self.alpha[1] * w**2)
        # Angular velocity variance depends on both linear and angular velocities
        var_w = (self.alpha[2] * v**2) + (self.alpha[3] * w**2)
        # Cross-correlation term (often assumed zero or small)
        var_vw = self.alpha[4] * v * w

        # Construct the 3x3 covariance matrix for the motion model
        # This matrix represents the uncertainty in (x, y, theta) due to motion
        return np.array([
            [var_v, 0, var_vw],
            [0, var_v, var_vw],
            [var_vw, var_vw, var_w]
        ])

    def aruco_callback(self, msg):
        """
        Process ArUco marker data from the ArUco detector node.
        Message format: [marker_id, range, bearing, marker_x_world, marker_y_world]

        Args:
            msg: Float32MultiArray containing ArUco detection data
        """
        if len(msg.data) != 5:
            self.get_logger().error(f"Invalid ArUco data format. Expected 5 values, got {len(msg.data)}")
            return

        # Extract data from message
        marker_id = int(msg.data[0])
        range_to_marker = msg.data[1]      # Range in meters (from robot's base_link)
        bearing_to_marker = msg.data[2]    # Bearing in radians (from robot's base_link)
        marker_x_world = msg.data[3]       # Marker X position in world frame
        marker_y_world = msg.data[4]       # Marker Y position in world frame

        # Validate measurement ranges for safety and plausibility
        if range_to_marker <= 0 or range_to_marker > 10.0:  # Example reasonable range limits
            self.get_logger().warn(f"Invalid range measurement: {range_to_marker:.2f}m. Skipping.")
            return

        if abs(bearing_to_marker) > math.pi + 0.1:  # Bearing should be within approx [-π, π]
            self.get_logger().warn(f"Invalid bearing measurement: {math.degrees(bearing_to_marker):.1f}°. Skipping.")
            return

        # Store the measurement for use in the Kalman filter's correction step
        self.latest_aruco_measurement = np.array([range_to_marker, bearing_to_marker])
        self.latest_landmark_pos = np.array([marker_x_world, marker_y_world])
        self.latest_aruco_id = marker_id
        self.have_new_aruco = True # Set flag to trigger correction in timer_callback

        # Log received ArUco data for debugging (using debug level)
        self.get_logger().debug(
            f"Received ArUco marker {marker_id}: range={range_to_marker:.2f}m, "
            f"bearing={math.degrees(bearing_to_marker):.1f}°, "
            f"world_position=({marker_x_world:.2f}, {marker_y_world:.2f})"
        )

    def linearized_state_update(self, s_prev, u, dt):
        """
        EKF Prediction Step: Updates state and covariance based on motion model.

        Args:
            s_prev (np.array): Previous state vector [x, y, theta]
            u (np.array): Control input vector [linear_velocity, angular_velocity]
            dt (float): Time elapsed since last update

        Returns:
            tuple: (s_new, Sigma_new) - predicted state and covariance
        """
        x, y, theta = s_prev[0], s_prev[1], s_prev[2]
        v, w = u[0], u[1]

        # Predict new state using the nonlinear motion model
        # This is the f(s_prev, u) part of the EKF
        if abs(w) > 1e-6: # Avoid division by zero if angular velocity is zero
            s_new = np.array([
                x + (-v/w) * math.sin(theta) + (v/w) * math.sin(theta + w * dt),
                y + (v/w) * math.cos(theta) - (v/w) * math.cos(theta + w * dt),
                theta + w * dt
            ])
        else: # Straight line motion
            s_new = np.array([
                x + v * math.cos(theta) * dt,
                y + v * math.sin(theta) * dt,
                theta # No change in theta
            ])

        # Normalize heading angle to [-π, π]
        s_new[2] = self.normalize_angle(s_new[2])

        # Compute Jacobian matrices for linearization (A_k and B_k)
        # A_k (F_x) represents how the state changes with respect to the previous state
        # Derivatives of f(s_prev, u) with respect to s_prev
        if abs(w) > 1e-6:
            A_k = np.array([
                [1, 0, (-v/w) * math.cos(theta) + (v/w) * math.cos(theta + w * dt)],
                [0, 1, (-v/w) * math.sin(theta) + (v/w) * math.sin(theta + w * dt)],
                [0, 0, 1]
            ])
        else: # Straight line motion
            A_k = np.array([
                [1, 0, -v * math.sin(theta) * dt],
                [0, 1,  v * math.cos(theta) * dt],
                [0, 0, 1]
            ])

        # B_k (F_u) represents how the state changes with respect to the control input
        # Derivatives of f(s_prev, u) with respect to u
        # This is usually used if the noise is applied to the control inputs,
        # but here we're using the alpha model for process noise Q.
        # For the alpha model, Q is directly added, so B_k is not explicitly used for noise propagation
        # but is useful if you were to transform input noise.
        # For simplicity, we'll use a direct Q based on alpha.

        # Compute process noise covariance matrix Q
        if self.motion_noise:
            Q = self.noise(v, w) # Motion-dependent noise
        else:
            Q = self.Q_constant # Constant base noise

        # Predict covariance using linearized covariance propagation
        # Sigma_new = A_k @ Sigma_prev @ A_k.T + Q
        Sigma_new = A_k @ self.Sigma @ A_k.T + Q

        return s_new, Sigma_new

    def kalman_filter(self, z, landmark_pos):
        """
        EKF Update Step: Corrects state and covariance using ArUco measurements.

        Args:
            z (np.array): Measurement vector [range, bearing] from camera (base_link frame)
            landmark_pos (np.array): Landmark position [x, y] in world frame
        """
        # Current estimated robot pose from the state vector
        x_r, y_r, theta_r = self.s[0], self.s[1], self.s[2]

        # Compute relative position from robot to landmark in world frame
        dx = landmark_pos[0] - x_r  # Difference in X (world frame)
        dy = landmark_pos[1] - y_r  # Difference in Y (world frame)

        # Avoid division by zero in range calculation if robot is exactly on landmark
        eps = 1e-6
        squared_dist = dx**2 + dy**2
        dist = np.sqrt(max(squared_dist, eps)) # Distance to landmark

        # Compute expected measurement (z_hat) based on current state estimate
        # This is the h(s) part of the EKF
        expected_range = dist
        expected_bearing = self.normalize_angle(np.arctan2(dy, dx) - theta_r)
        z_hat = np.array([expected_range, expected_bearing])

        # Compute measurement Jacobian (H_k or G)
        # This linearizes the nonlinear measurement model around the current state
        # Derivatives of h(s) with respect to s = [x, y, theta]
        G = np.array([
            [-dx / dist, -dy / dist, 0],  # Derivative of range w.r.t. x, y, theta
            [ dy / squared_dist, -dx / squared_dist, -1]   # Derivative of bearing w.r.t. x, y, theta
        ])

        # Compute innovation covariance (S)
        # S = G @ Sigma @ G.T + R
        S = G @ self.Sigma @ G.T + self.R

        # Compute Kalman gain (K)
        # K = Sigma @ G.T @ S_inverse
        K = self.Sigma @ G.T @ np.linalg.inv(S)

        # Compute innovation (y_k) (difference between actual and expected measurement)
        innovation = z - z_hat
        innovation[1] = self.normalize_angle(innovation[1])  # Normalize bearing innovation

        # Calculate Normalized Innovation Squared (NIS) for filter monitoring
        try:
            nis = innovation.T @ np.linalg.inv(S) @ innovation
            self.nis_values.append(nis)
            if len(self.nis_values) > self.nis_window_size:
                self.nis_values.pop(0) # Remove oldest NIS value
            self.get_logger().debug(f"NIS: {nis:.3f}")
        except np.linalg.LinAlgError:
            self.get_logger().warn("Could not compute NIS - singular innovation covariance (S)")

        # Monitor filter consistency using NIS
        self.monitor_consistency()

        # Update state estimate using Kalman gain and innovation
        # s_new = s_prev + K @ innovation
        self.s += K @ innovation
        # Normalize heading angle after update
        self.s[2] = self.normalize_angle(self.s[2])

        # Update covariance using Joseph form (numerically stable)
        # P_new = (I - K @ G) @ P_prev @ (I - K @ G).T + K @ R @ K.T
        I = np.eye(3) # Identity matrix
        self.Sigma = (I - K @ G) @ self.Sigma @ (I - K @ G).T + K @ self.R @ K.T

    def monitor_consistency(self):
        """
        Monitors the filter consistency using the Normalized Innovation Squared (NIS) values.
        Logs warnings if the filter appears overconfident or underconfident.
        """
        if len(self.nis_values) < self.nis_window_size:
            return # Not enough data yet

        # Count NIS values outside expected bounds (based on chi-squared distribution)
        too_high = sum(1 for nis in self.nis_values if nis > self.chi2_upper)
        too_low = sum(1 for nis in self.nis_values if nis < self.chi2_lower)

        high_percent = too_high / len(self.nis_values) * 100
        low_percent = too_low / len(self.nis_values) * 100

        # Log warnings if filter consistency is poor
        if high_percent > 15:  # More than 15% of NIS values above upper bound
            self.get_logger().warn(
                f"Filter appears overconfident (NIS too high): {high_percent:.1f}% > 15% of values outside upper bound."
            )
        elif low_percent > 15:  # More than 15% of NIS values below lower bound
            self.get_logger().warn(
                f"Filter appears underconfident (NIS too low): {low_percent:.1f}% > 15% of values outside lower bound."
            )

def main(args=None):
    """Main function to run the enhanced odometry node."""
    rclpy.init(args=args)
    node = Odometry_Node()

    try:
        rclpy.spin(node) # Keep node alive until Ctrl+C is pressed
    except KeyboardInterrupt:
        pass # Handle Ctrl+C gracefully
    finally:
        node.destroy_node() # Clean up node resources
        rclpy.shutdown() # Shut down ROS2 client library

if __name__ == '__main__':
    main()