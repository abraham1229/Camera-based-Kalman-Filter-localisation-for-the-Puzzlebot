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
        self.declare_parameter('odom_frame', 'world')
        self.declare_parameter("use_linear_model", True)
        self.declare_parameter('motion_model_noise', False)
        self.declare_parameter('base_frame', 'base_footprint')
        # Add confidence parameters
        self.declare_parameter('high_freq_prediction', True)
        self.declare_parameter('adaptive_noise', True)

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
        self.subscription_aruco = self.create_subscription(
            Float32MultiArray,
            'aruco_range_bearing',
            self.aruco_callback,
            10)
        
        # Publishers
        self.pub_odometry = self.create_publisher(Odometry, 'odometria', 1000)
        
        # TF2 broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for main odometry loop at 100Hz
        self.timer_period = 0.01 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Physical robot parameters
        self.radius = 0.05    # Wheel radius in meters
        self.lenght = 0.19    # Distance between wheels in meters

        # Velocity variables from wheel encoders
        self.vel_left = 0.0   
        self.vel_right = 0.0  
        
        # Previous velocities for change detection
        self.prev_vel_left = 0.0
        self.prev_vel_right = 0.0
        
        # Initialize covariance matrix with better initial values
        self.Sigma = np.diag([0.01, 0.01, 0.001])  # Better initial uncertainty

        # Base process noise covariance matrix (time-independent)
        self.Q_base = np.array([
            [0.001, 0.0,   0.0],
            [0.0,   0.001, 0.0],
            [0.0,   0.0,   0.0001]
        ])

        # Measurement noise covariance matrix for ArUco observations
        self.R_base = np.array([
            [0.01, 0],     # Range measurement noise (1cm std)
            [0, 0.05]      # Bearing measurement noise (3 degree std)
        ])
        
        # Kalman filter variables for ArUco measurements
        self.have_new_aruco = False
        self.latest_aruco_measurement = np.zeros(2)
        self.latest_landmark_pos = np.zeros(2)
        self.latest_aruco_id = -1
        self.aruco_timestamp = None

        # Motion model noise parameters
        self.alpha = [0.01, 0.001, 0.01, 0.001, 0.001]  # Reduced noise
        
        # NIS monitoring
        self.nis_values = []
        self.nis_window_size = 30
        self.chi2_lower = 0.103
        self.chi2_upper = 5.991

        # Get parameters
        self.posX = self.get_parameter('init_pose_x').value
        self.posY = self.get_parameter('init_pose_y').value
        self.theta = self.get_parameter('init_pose_yaw').value
        self.use_linear_model = self.get_parameter('use_linear_model').get_parameter_value().bool_value
        self.motion_noise = self.get_parameter('motion_model_noise').get_parameter_value().bool_value
        self.high_freq_prediction = self.get_parameter('high_freq_prediction').get_parameter_value().bool_value
        self.adaptive_noise = self.get_parameter('adaptive_noise').get_parameter_value().bool_value
        
        # Frame names
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # Initialize state vector for Kalman filter
        self.s = np.array([self.posX, self.posY, self.theta])
        
        # Handle namespace prefixing for frame names
        ns = self.get_namespace().strip('/')
        if ns:
            self.odom_frame = f'{ns}/{self.odom_frame}' if not self.odom_frame.startswith(ns) else self.odom_frame
            self.base_frame = f'{ns}/{self.base_frame}' if not self.base_frame.startswith(ns) else self.base_frame
        
        # Outlier detection
        self.measurement_validation_threshold = 3.0  # Mahalanobis distance threshold
        
        self.get_logger().info(f'Enhanced Odometry node initialized')
    
    def timer_callback(self):
        """Main odometry loop with improved EKF integration"""
        # Compute robot velocities from differential drive kinematics
        v_linear = self.radius * ((self.vel_right + self.vel_left) / 2)
        v_angular = self.radius * ((self.vel_right - self.vel_left) / self.lenght)
        
        if self.use_linear_model:
            # Use EKF for all state updates
            u = np.array([v_linear, v_angular])
            
            # Predict step: Update state and covariance using motion model
            self.predict_step(u)
            
            # Correction step: Apply ArUco measurement if available
            if self.have_new_aruco:
                z = self.latest_aruco_measurement
                landmark = self.latest_landmark_pos
                
                R = self.get_adaptive_measurement_noise(z)
                self.kalman_filter(z, landmark, R)
                # Validate measurement before applying
            
                self.have_new_aruco = False
            
            # Update pose from EKF state (single source of truth)
            self.posX, self.posY, self.theta = self.s
            
        else:
            # Simple odometry without EKF
            self.theta += v_angular * self.timer_period
            self.posX += v_linear * math.cos(self.theta) * self.timer_period
            self.posY += v_linear * math.sin(self.theta) * self.timer_period

        # Normalize theta
        self.theta = self.normalize_angle(self.theta)
        self.s[2] = self.theta  # Keep state consistent

        # Create and publish messages
        current_time = self.get_clock().now().to_msg()
        odom_msg = self.create_odometry_message(current_time, v_linear, v_angular)
        self.pub_odometry.publish(odom_msg)
        
        self.publish_world_to_base_transform(current_time)

    def predict_step(self, u):
        """EKF Prediction step with improved integration"""
        dt = self.timer_period
        v, w = u[0], u[1]
        
        # Use improved integration for better accuracy
        if abs(w) > 1e-6:  # Non-zero angular velocity
            # Use exact integration for circular motion
            theta_new = self.s[2] + w * dt
            theta_mid = self.s[2] + 0.5 * w * dt
            
            self.s[0] += (v / w) * (math.sin(theta_new) - math.sin(self.s[2]))
            self.s[1] += (v / w) * (math.cos(self.s[2]) - math.cos(theta_new))
            self.s[2] = theta_new
        else:
            # Linear motion (w ≈ 0)
            theta_mid = self.s[2]
            self.s[0] += v * math.cos(theta_mid) * dt
            self.s[1] += v * math.sin(theta_mid) * dt
            self.s[2] += w * dt
        
        # Normalize angle
        self.s[2] = self.normalize_angle(self.s[2])
        
        # Compute Jacobian for covariance update
        A_k = self.compute_motion_jacobian(u, dt)
        
        # Update covariance with time-scaled process noise
        Q = self.get_process_noise(u, dt)
        self.Sigma = A_k @ self.Sigma @ A_k.T + Q

    def compute_motion_jacobian(self, u, dt):
        """Compute motion model Jacobian"""
        v, w = u[0], u[1]
        theta = self.s[2]
        
        if abs(w) > 1e-6:
            # Nonlinear motion model Jacobian
            A_k = np.array([
                [1, 0, (v/w) * (math.cos(theta + w*dt) - math.cos(theta))],
                [0, 1, (v/w) * (math.sin(theta + w*dt) - math.sin(theta))],
                [0, 0, 1]
            ])
        else:
            # Linear motion model Jacobian
            A_k = np.array([
                [1, 0, -v * math.sin(theta) * dt],
                [0, 1,  v * math.cos(theta) * dt],
                [0, 0, 1]
            ])
        
        return A_k

    def get_process_noise(self, u, dt):
        """Get process noise matrix with proper time scaling"""
        v, w = u[0], u[1]
        
        if self.motion_noise and (abs(v) > 1e-3 or abs(w) > 1e-3):
            # Motion-dependent noise (from Probabilistic Robotics)
            var_v = (self.alpha[0] * v**2) + (self.alpha[1] * w**2)
            var_w = (self.alpha[2] * v**2) + (self.alpha[3] * w**2)
            
            # Scale with time and motion
            Q = np.array([
                [var_v * dt**2, 0, 0],
                [0, var_v * dt**2, 0],
                [0, 0, var_w * dt**2]
            ])
        else:
            # Constant process noise scaled by time
            Q = self.Q_base * dt
            
        # Reduce process noise when robot is stationary (increases confidence in dead reckoning)
        if abs(v) < 0.01 and abs(w) < 0.01:
            Q *= 0.1  # Much lower noise when stationary
        
        return Q

    def get_adaptive_measurement_noise(self, z):
        """Adapt measurement noise based on measurement quality"""
        if not self.adaptive_noise:
            return self.R_base.copy()
        
        R = self.R_base.copy()
        range_measurement = z[0]
        bearing_measurement = abs(z[1])
        
        # Increase noise for far measurements
        if range_measurement > 2.0:
            range_factor = 1.0 + (range_measurement - 2.0) / 3.0
            R[0, 0] *= range_factor
        
        # Increase noise for side measurements (bearing close to ±π/2)
        if bearing_measurement > math.pi/3:
            bearing_factor = 1.0 + (bearing_measurement - math.pi/3) / (math.pi/6)
            R[1, 1] *= bearing_factor
        
        return R


    def create_odometry_message(self, timestamp, v_linear, v_angular):
        """Create odometry message with current velocities"""
        odom_msg = Odometry()
        
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Set position
        odom_msg.pose.pose.position.x = self.posX
        odom_msg.pose.pose.position.y = self.posY
        odom_msg.pose.pose.position.z = 0.0

        # Set orientation
        quat = transforms3d.euler.euler2quat(0, 0, self.theta)
        odom_msg.pose.pose.orientation.w = quat[0]
        odom_msg.pose.pose.orientation.x = quat[1]
        odom_msg.pose.pose.orientation.y = quat[2]
        odom_msg.pose.pose.orientation.z = quat[3]

        # Set velocity (use actual computed velocities, not stored ones)
        odom_msg.twist.twist.linear.x = v_linear
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = v_angular

        # Set covariance matrices if using EKF
        if self.use_linear_model:
            odom_msg.pose.covariance = [
                self.Sigma[0, 0], self.Sigma[0, 1], 0.0, 0.0, 0.0, self.Sigma[0, 2],
                self.Sigma[1, 0], self.Sigma[1, 1], 0.0, 0.0, 0.0, self.Sigma[1, 2],
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                self.Sigma[2, 0], self.Sigma[2, 1], 0.0, 0.0, 0.0, self.Sigma[2, 2]
            ]

        return odom_msg

    def publish_world_to_base_transform(self, timestamp):
        """Publish transform from world to base_footprint"""
        transform = TransformStamped()
        
        transform.header.stamp = timestamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        
        transform.transform.translation.x = self.posX
        transform.transform.translation.y = self.posY
        transform.transform.translation.z = 0.0
        
        quat = transforms3d.euler.euler2quat(0, 0, self.theta)
        transform.transform.rotation.w = quat[0]
        transform.transform.rotation.x = quat[1]
        transform.transform.rotation.y = quat[2]
        transform.transform.rotation.z = quat[3]
        
        self.tf_broadcaster.sendTransform(transform)

    def signal_callback_left(self, msg):
        """Process left wheel velocity data from encoder."""
        if msg is not None:
            self.prev_vel_left = self.vel_left
            self.vel_left = msg.data if abs(msg.data) > 1e-3 else 0.0

    def signal_callback_right(self, msg):
        """Process right wheel velocity data from encoder."""
        if msg is not None:
            self.prev_vel_right = self.vel_right
            self.vel_right = msg.data if abs(msg.data) > 1e-3 else 0.0

    def normalize_angle(self, theta):
        """Normalize angle to [-π, π] range."""
        return (theta + math.pi) % (2 * math.pi) - math.pi
    
    def aruco_callback(self, msg):
        """Process ArUco marker data"""
        if len(msg.data) != 5:
            self.get_logger().error(f"Invalid ArUco data format. Expected 5 values, got {len(msg.data)}")
            return
            
        marker_id = int(msg.data[0])
        range_to_marker = msg.data[1]
        bearing_to_marker = msg.data[2]
        marker_x_world = msg.data[3]
        marker_y_world = msg.data[4]
        
        # Validate measurement ranges
        if range_to_marker <= 0 or range_to_marker > 5:
            self.get_logger().warn(f"Invalid range measurement: {range_to_marker:.2f}m")
            return
            
        if abs(bearing_to_marker) > math.pi:
            self.get_logger().warn(f"Invalid bearing measurement: {math.degrees(bearing_to_marker):.1f}°")
            return
        
        # Store measurement with timestamp
        self.latest_aruco_measurement = np.array([range_to_marker, bearing_to_marker])
        self.latest_landmark_pos = np.array([marker_x_world, marker_y_world])
        self.latest_aruco_id = marker_id
        self.aruco_timestamp = self.get_clock().now()
        self.have_new_aruco = True
        
        self.get_logger().debug(
            f"Received ArUco marker {marker_id}: range={range_to_marker:.2f}m, "
            f"bearing={math.degrees(bearing_to_marker):.1f}°, "
            f"world_position=({marker_x_world:.2f}, {marker_y_world:.2f})"
        )
    
    def kalman_filter(self, z, landmark_pos, R):
        """EKF update step with improved numerical stability"""
        # Compute relative position
        dx = landmark_pos[0] - self.s[0]
        dy = landmark_pos[1] - self.s[1]
        theta = self.s[2]
        
        eps = 1e-6
        p = max(dx**2 + dy**2, eps)
        sqrt_p = np.sqrt(p)

        # Expected measurement
        z_hat = np.array([
            sqrt_p,
            self.normalize_angle(np.arctan2(dy, dx) - theta)
        ])

        # Measurement Jacobian
        G = np.array([
            [-dx / sqrt_p, -dy / sqrt_p, 0],
            [ dy / p,      -dx / p,      -1]
        ])

        # Innovation covariance
        S = G @ self.Sigma @ G.T + R

        # Compute Kalman gain with numerical stability check
        try:
            K = self.Sigma @ G.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.get_logger().error("Singular innovation covariance matrix")
            return

        # Innovation
        innovation = z - z_hat
        innovation[1] = self.normalize_angle(innovation[1])

        # NIS calculation
        try:
            nis = innovation.T @ np.linalg.inv(S) @ innovation
            self.nis_values.append(nis)
            if len(self.nis_values) > self.nis_window_size:
                self.nis_values.pop(0)
        except np.linalg.LinAlgError:
            pass

        # State update
        self.s += K @ innovation
        self.s[2] = self.normalize_angle(self.s[2])

        # Covariance update using Joseph form for numerical stability
        I = np.eye(3)
        IKG = I - K @ G
        self.Sigma = IKG @ self.Sigma @ IKG.T + K @ R @ K.T

        # Monitor filter consistency
        self.monitor_consistency()

    def monitor_consistency(self):
        """Monitor filter consistency using NIS"""
        if len(self.nis_values) < self.nis_window_size:
            return

        too_high = sum(1 for nis in self.nis_values if nis > self.chi2_upper)
        too_low = sum(1 for nis in self.nis_values if nis < self.chi2_lower)

        high_percent = too_high / len(self.nis_values) * 100
        low_percent = too_low / len(self.nis_values) * 100

        if high_percent > 15:
            self.get_logger().warn(
                f"Filter overconfident (NIS too high): {high_percent:.1f}% > 15%"
            )
        elif low_percent > 15:
            self.get_logger().warn(
                f"Filter underconfident (NIS too low): {low_percent:.1f}% > 15%"
            )

def main(args=None):
    """Main function"""
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