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
        self.declare_parameter('odom_frame', 'world')  # Changed to 'world' to match your TF chain
        self.declare_parameter("use_linear_model", True)
        self.declare_parameter('motion_model_noise', False)
        self.declare_parameter('base_frame', 'base_footprint')  # Added base frame parameter

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
        # This data contains marker positions in world frame coordinates (your odom frame)
        self.subscription_aruco = self.create_subscription(
            Float32MultiArray,
            'aruco_range_bearing',
            self.aruco_callback,
            10)
        
        # Publishers
        self.pub_odometry = self.create_publisher(Odometry, 'odometria', 1000)
        
        # TF2 broadcaster for publishing world -> base_footprint transform
        # This is the critical transform that positions your robot in the world frame
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
            [3.02e-03, 0],           # Variance for range measurement (m^2)
            [0, 4.78e-06]  # Variance for bearing measurement (rad^2)
        ])

        # Kalman filter variables for ArUco measurements
        self.have_new_aruco = False
        self.latest_aruco_measurement = np.zeros(2)  # [range, bearing]
        self.latest_landmark_pos = np.zeros(2)       # [x, y] in world frame
        self.latest_aruco_id = -1

        # Motion model noise parameters (Probabilistic Robotics, Chapter 5)
        # These parameters control how much noise to add based on robot motion
        self.alpha = [0.05, 0.001, 0.05, 0.01, 0.01]
        
        # Normalized Innovation Squared (NIS) monitoring for filter consistency
        self.nis_values = []
        self.nis_window_size = 30
        self.chi2_lower = 0.103  # chi2 lower bound for 95% confidence, 2 DoF
        self.chi2_upper = 5.991  # chi2 upper bound for 95% confidence, 2 DoF

        # Robot state variables (position and orientation in world frame)
        init_x = self.get_parameter('init_pose_x').value
        init_y = self.get_parameter('init_pose_y').value
        init_theta = self.get_parameter('init_pose_yaw').value
        
        # Get configuration parameters
        self.use_linear_model = self.get_parameter('use_linear_model').get_parameter_value().bool_value
        self.motion_noise = self.get_parameter('motion_model_noise').get_parameter_value().bool_value
        # Get frame names from parameters (properly handle namespaces)
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # Initialize state vector for Kalman filter - this is now the single source of truth
        self.s = np.array([init_x, init_y, init_theta])
        
        # Handle namespace prefixing for frame names
        # This ensures frame names are consistent across your TF tree
        ns = self.get_namespace().strip('/')
        if ns:
            # Add namespace prefix if we're in a namespace
            self.odom_frame = f'{ns}/{self.odom_frame}' if not self.odom_frame.startswith(ns) else self.odom_frame
            self.base_frame = f'{ns}/{self.base_frame}' if not self.base_frame.startswith(ns) else self.base_frame
        
        self.get_logger().info(f'Enhanced Odometry node initialized')
    
    # Property accessors to maintain backward compatibility while using state vector
    @property
    def posX(self):
        return self.s[0]
    
    @property
    def posY(self):
        return self.s[1]
    
    @property
    def theta(self):
        return self.s[2]

    def timer_callback(self):
        # Compute robot velocities from differential drive kinematics
        self.velocidadTheta = self.radius * ((self.vel_right - self.vel_left) / self.lenght)
        self.velLineal = self.radius * ((self.vel_right + self.vel_left) / 2)
        
        if self.use_linear_model:
            # Use linearized Extended Kalman Filter (EKF) approach
            u = np.array([self.velLineal, self.velocidadTheta])  # Control input

            # Predict step: Update state and covariance using motion model
            self.s, self.Sigma = self.linearized_state_update(self.s, u)
            
            # Correction step: Apply ArUco measurement if available
            if self.have_new_aruco:
                z = self.latest_aruco_measurement  # [range, bearing]
                landmark = self.latest_landmark_pos # [x, y] in world frame
                
                # Apply Kalman filter correction using ArUco measurement
                self.kalman_filter(z, landmark)
                self.have_new_aruco = False
                
                self.get_logger().info(
                    f"After Kalman correction: x={self.s[0]:.3f}, y={self.s[1]:.3f}, "
                    f"θ={math.degrees(self.s[2]):.2f}°"
                )
            else:
                # Log state without ArUco measurement
                self.get_logger().info(
                    f"EKF prediction only: x={self.s[0]:.3f}, y={self.s[1]:.3f}, "
                    f"θ={math.degrees(self.s[2]):.2f}°"
                )
        else:
            # Use simple dead reckoning without Kalman filter
            # Update robot orientation
            self.s[2] += self.velocidadTheta * self.timer_period
                    
            # Update robot position using kinematic model
            self.s[0] += self.velLineal * math.cos(self.s[2]) * self.timer_period
            self.s[1] += self.velLineal * math.sin(self.s[2]) * self.timer_period
            
            self.get_logger().info(
                f"Dead reckoning: x={self.s[0]:.3f}, y={self.s[1]:.3f}, "
                f"θ={math.degrees(self.s[2]):.2f}°"
            )
        
        # Normalize theta to [-π, π] range
        self.s[2] = self.normalize_angle(self.s[2])

        # Create and publish Odometry message
        # Create current timestamp for consistent use across messages
        current_time = self.get_clock().now().to_msg()
        odom_msg = self.create_odometry_message(current_time)
        self.pub_odometry.publish(odom_msg)
        
        # Publish TF transform from world to base_footprint
        # This is the critical transform that positions your robot in the world
        self.publish_world_to_base_transform(current_time)

    def create_odometry_message(self, timestamp):
        odom_msg = Odometry()
        
        # Fill odometry message header
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = self.odom_frame  # Parent frame (world)
        odom_msg.child_frame_id = self.base_frame   # Child frame (base_footprint)

        # Set position in world frame - now using state vector consistently
        odom_msg.pose.pose.position.x = self.s[0]
        odom_msg.pose.pose.position.y = self.s[1]
        odom_msg.pose.pose.position.z = 0.0

        # Convert theta (yaw) to quaternion for orientation
        quat = transforms3d.euler.euler2quat(0, 0, self.s[2])
        odom_msg.pose.pose.orientation.w = quat[0]  # w component first
        odom_msg.pose.pose.orientation.x = quat[1]
        odom_msg.pose.pose.orientation.y = quat[2]
        odom_msg.pose.pose.orientation.z = quat[3]

        # Set velocity information in base_footprint frame
        odom_msg.twist.twist.linear.x = self.velLineal      # Forward velocity
        odom_msg.twist.twist.linear.y = 0.0                 # No sideways motion
        odom_msg.twist.twist.linear.z = 0.0                 # No vertical motion
        odom_msg.twist.twist.angular.x = 0.0                # No roll
        odom_msg.twist.twist.angular.y = 0.0                # No pitch
        odom_msg.twist.twist.angular.z = self.velocidadTheta # Yaw rate

        # Set covariance matrices if using EKF
        if self.use_linear_model:
            # Pose covariance (6x6 matrix flattened to 36 elements)
            odom_msg.pose.covariance = [
                self.Sigma[0, 0], self.Sigma[0, 1], 0.0, 0.0, 0.0, self.Sigma[0, 2], # x row
                self.Sigma[1, 0], self.Sigma[1, 1], 0.0, 0.0, 0.0, self.Sigma[1, 2], # y row
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                                        # z row
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                                        # roll row
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                                        # pitch row
                self.Sigma[2, 0], self.Sigma[2, 1], 0.0, 0.0, 0.0, self.Sigma[2, 2]  # yaw row
            ]

        
        return odom_msg

    def publish_world_to_base_transform(self, timestamp):
        """
        Publish the transform from world frame to base_footprint frame.
        
        This transform is crucial for your TF tree. It tells ROS where the robot
        is positioned and oriented within the world coordinate system.
        
        Your complete TF chain will be:
        world -> base_footprint -> base_link -> bracket_base_link -> camera_link -> camera_link_optical
        
        This function publishes the first transform in that chain.
        
        Args:
            timestamp: Current timestamp for the transform
        """
        transform = TransformStamped()
        
        # Set up transform header
        transform.header.stamp = timestamp
        transform.header.frame_id = self.odom_frame    # Parent frame (world)
        transform.child_frame_id = self.base_frame     # Child frame (base_footprint)
        
        # Set translation (robot's position in world frame) - using state vector
        transform.transform.translation.x = self.s[0]
        transform.transform.translation.y = self.s[1]
        transform.transform.translation.z = 0.0
        
        # Set rotation (robot's orientation in world frame)
        quat = transforms3d.euler.euler2quat(0, 0, self.s[2])
        transform.transform.rotation.w = quat[0]  # w component first
        transform.transform.rotation.x = quat[1]
        transform.transform.rotation.y = quat[2]
        transform.transform.rotation.z = quat[3]
        
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
        Message format: [marker_id, range, bearing, marker_x_world, marker_y_world]
        
        Args:
            msg: Float32MultiArray containing ArUco detection data
        """
        if len(msg.data) != 5:
            self.get_logger().error(f"Invalid ArUco data format. Expected 5 values, got {len(msg.data)}")
            return
            
        # Extract data from message
        marker_id = int(msg.data[0])
        range_to_marker = msg.data[1]      # Range in meters (base_link frame)
        bearing_to_marker = msg.data[2]    # Bearing in radians (base_link frame)
        marker_x_world = msg.data[3]       # Marker X position in world frame
        marker_y_world = msg.data[4]       # Marker Y position in world frame
        
        # Validate measurement ranges for safety
        if range_to_marker <= 0 or range_to_marker > 3.0:  # Reasonable range limits
            self.get_logger().warn(f"Invalid range measurement: {range_to_marker:.2f}m")
            return
            
        if abs(bearing_to_marker) > math.pi:  # Bearing should be within [-π, π]
            self.get_logger().warn(f"Invalid bearing measurement: {math.degrees(bearing_to_marker):.1f}°")
            return
        
        # Store the measurement for use in the Kalman filter
        # All coordinates are now properly aligned in their respective frames
        self.latest_aruco_measurement = np.array([range_to_marker, bearing_to_marker])
        self.latest_landmark_pos = np.array([marker_x_world, marker_y_world])
        self.latest_aruco_id = marker_id
        self.have_new_aruco = True
        
        # Log received ArUco data for debugging
        self.get_logger().debug(
            f"Received ArUco marker {marker_id}: range={range_to_marker:.2f}m, "
            f"bearing={math.degrees(bearing_to_marker):.1f}°, "
            f"world_position=({marker_x_world:.2f}, {marker_y_world:.2f})"
        )
    
    def linearized_state_update(self, s, u):
        # Compute Jacobian matrices for linearization
        # A_k represents how the state changes with respect to the previous state
        A_k = np.array([
            [1, 0, -u[0] * math.sin(s[2]) * self.timer_period],  
            [0, 1,  u[0] * math.cos(s[2]) * self.timer_period],  
            [0, 0, 1]
        ])

        # B_k represents how the state changes with respect to the control input
        B_k = np.array([
            [math.cos(s[2]) * self.timer_period, 0],  
            [math.sin(s[2]) * self.timer_period, 0],  
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
        
        The key aspect: all measurements and positions are now in consistent
        coordinate frames (world frame for positions, base_link frame for measurements).
        
        Args:
            z (np.array): Measurement vector [range, bearing] from camera (base_link frame)
            landmark_pos (np.array): Landmark position [x, y] in world frame
        """
        # Compute relative position from robot to landmark in world frame
        dx = landmark_pos[0] - self.s[0]  # Difference in X (world frame)
        dy = landmark_pos[1] - self.s[1]  # Difference in Y (world frame)
        theta = self.s[2]                 # Robot's current heading in world frame
        
        # Avoid division by zero in range calculation
        eps = 1e-6
        p = max(dx**2 + dy**2, eps)  # Squared distance to landmark


        # Compute expected measurement based on current state estimate
        # This represents what we would expect to measure if our state estimate is correct
        z_hat = np.array([
            np.sqrt(p),                           # Expected range to landmark
            np.arctan2(dy, dx) - theta           # Expected bearing (relative to robot heading)
        ])
        z_hat[1] = self.normalize_angle(z_hat[1])

        # Compute measurement Jacobian (how measurement changes with state)
        # This linearizes the nonlinear measurement model around current state
        G = np.array([
            [-dx / np.sqrt(p), -dy / np.sqrt(p), 0],  # Range derivatives w.r.t. x, y, θ
            [ dy / p,          -dx / p,         -1]   # Bearing derivatives w.r.t. x, y, θ
        ])

        self.get_logger().info(
            f"\nKalman Filter Values:"
            f"\n  dx: {dx:.3f}m"
            f"\n  dy: {dy:.3f}m"
            f"\n  theta: {math.degrees(theta):.2f}°"
            f"\n  Measured distance: {z[0]:.3f}m"
            f"\n  Measured bearing: {math.degrees(z[1]):.2f}°"
            f"\n  Expected distance: {z_hat[0]:.3f}m"
            f"\n  Expected bearing: {math.degrees(z_hat[1]):.2f}°"
            f"\n  Distance error: {z[0] - z_hat[0]:.3f}m"
            f"\n  Bearing error: {math.degrees(self.normalize_angle(z[1] - z_hat[1])):.2f}°"
        )
        # Compute innovation covariance (uncertainty in innovation)
        S = G @ self.Sigma @ G.T + self.R

        # Check for numerical issues with innovation covariance
        try:
            # Compute Kalman gain (optimal weighting between prediction and measurement)
            K = self.Sigma @ G.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.get_logger().error("Singular innovation covariance matrix - skipping update")
            return

        # Compute innovation (difference between actual and expected measurement)
        innovation = z - z_hat
        innovation[1] = self.normalize_angle(innovation[1])  # Normalize bearing innovation

        # Calculate Normalized Innovation Squared (NIS) for filter monitoring
        try:
            nis = innovation.T @ np.linalg.inv(S) @ innovation
            self.nis_values.append(nis)
            if len(self.nis_values) > self.nis_window_size:
                self.nis_values.pop(0)
        except np.linalg.LinAlgError:
            self.get_logger().warn("Could not compute NIS - singular covariance")

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