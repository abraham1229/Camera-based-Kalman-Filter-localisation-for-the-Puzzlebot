import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import Quaternion
from aruco_msgs.msg import MarkerArray
import transforms3d
import math
import numpy as np
from tf2_ros import TransformBroadcaster

def euler_from_quaternion(quaternion):
    """Convert quaternion to euler angles (roll, pitch, yaw)"""
    w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation) 
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

def quaternion_from_euler(roll, pitch, yaw):
    """Convert euler angles to geometry_msgs.msg.Quaternion"""
    quat = transforms3d.euler.euler2quat(roll, pitch, yaw)
    q_msg = Quaternion()
    q_msg.w = quat[0]
    q_msg.x = quat[1]
    q_msg.y = quat[2]
    q_msg.z = quat[3]
    return q_msg

def wrap_to_pi(angle):
    """Normalize angle to [-π, π]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

class EnhancedOdometry(Node):   

    def __init__(self):
        super().__init__('odometry')

        # Declare parameters with default values
        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0) 
        self.declare_parameter('init_pose_yaw', 0.0)
        self.declare_parameter('odom_frame', 'world')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter("use_kalman_filter", True)

        # Get initial parameters
        init_x = self.get_parameter('init_pose_x').value
        init_y = self.get_parameter('init_pose_y').value
        init_theta = self.get_parameter('init_pose_yaw').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.use_kalman = self.get_parameter('use_kalman_filter').value

        # Handle namespace prefixing for frame names
        ns = self.get_namespace().strip('/')
        if ns:
            self.base_frame = f'{ns}/{self.base_frame}' if not self.base_frame.startswith(ns) else self.base_frame

        self.subscription_velocity_left = self.create_subscription(
            Float32,
            'VelocityEncL',
            self.encL_callback,
            rclpy.qos.qos_profile_sensor_data)
        
        self.subscription_velocity_right = self.create_subscription(
            Float32,
            'VelocityEncR', 
            self.encR_callback,
            rclpy.qos.qos_profile_sensor_data)
        
        self.sub_aruco = self.create_subscription(MarkerArray,'/marker_publisher/markers',self.aruco_callback,rclpy.qos_profile_sensor_data)
        
        # Publishers
        self.pub_odometry = self.create_publisher(Odometry, 'odometria', 10)
        
        # TF2 broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for main odometry loop at 50Hz
        self.run_dt = 0.02  # 50Hz
        self.timer = self.create_timer(self.run_dt, self.run_loop)
        
        # Physical robot parameters
        self.wheel_radius = 0.05    # Wheel radius in meters
        self.robot_width = 0.1875   # Distance between wheels in meters

        # State variables
        self.pose_x = init_x
        self.pose_y = init_y 
        self.pose_theta = init_theta
        
        # Velocity variables from wheel encoders
        self.velocityL = 0.0   # Left wheel velocity (rad/s)
        self.velocityR = 0.0   # Right wheel velocity (rad/s)
        
        # Computed robot velocities
        self.Vr = 0.0  # Linear velocity
        self.Wr = 0.0  # Angular velocity
        
        # Kalman filter variables (matching kalman.py)
        self.Sig = np.array([[0.02, 0.0, 0.0],
                            [0.0, 0.02, 0.0], 
                            [0.0, 0.0, 0.05]])
        
        # Process noise parameter (from kalman.py)
        self.sigma_squared = 0.32
        
        self.map = [
            # Landmarks with IDs, x, y, theta
            [0, 2.5, -0.5, 0],   
            [1, 2.75,  2.5, 1.57],   
            [2, 4, -4, -1.57],   
            [3, -0.5, -0.5, -1.57],  
            [4, 0.45, -3, -1.57]
        ]
        
        # Timing variables for proper dt calculation
        self.first_stamp = True
        self.prev_stamp = None
        self.dt = self.run_dt  # Default dt
        

    def encL_callback(self, msg):
        """Left wheel velocity callback (matching kalman.py)"""
        self.velocityL = msg.data

    def encR_callback(self, msg):
        """Right wheel velocity callback (matching kalman.py)"""  
        self.velocityR = msg.data

    def aruco_callback(self, msg):
        """Process ArUco detections using full Pose message (enhanced from kalman.py)"""
        if not self.use_kalman:
            return
            
        markers = []
        for aruco_marker in msg.markers:
            # Extract pose information
            roll, pitch, yaw = euler_from_quaternion(aruco_marker.pose.orientation)
            
            marker = [
                aruco_marker.marker_id,
                aruco_marker.pose.position.z,    # Forward distance (range)
                -aruco_marker.pose.position.x,   # Lateral offset  
                wrap_to_pi(-pitch)               # Relative orientation
            ]
            markers.append(marker)
            
        if markers:
            success = self.kalman_correction(markers, 0.1)  # 0.1 is measurement noise

    def run_loop(self):
        """Main execution loop (matching kalman.py structure)"""

        # Compute robot velocities from wheel encoders
        self.Vr = (self.velocityR + self.velocityL) * self.wheel_radius / 2
        self.Wr = (self.velocityR - self.velocityL) * self.wheel_radius / self.robot_width
        
        # Apply motion model
        if self.use_kalman:
            self.kalman_prediction()
        else:
            self.dead_reckoning()
        
        # Create and publish odometry message
        current_time = self.get_clock().now().to_msg()
        odom_msg = self.create_odometry_message(current_time)
        self.pub_odometry.publish(odom_msg)
        
        # Publish transform
        self.publish_transform(current_time)

    def dead_reckoning(self):
        """Simple dead reckoning integration"""
        dt = self.dt
        
        self.pose_x += dt * self.Vr * math.cos(self.pose_theta + dt * self.Wr / 2)
        self.pose_y += dt * self.Vr * math.sin(self.pose_theta + dt * self.Wr / 2)
        self.pose_theta = wrap_to_pi(self.pose_theta + dt * self.Wr)

    def kalman_prediction(self):
        """Kalman prediction step (copied from kalman.py with minor adaptations)"""
        dt = self.dt
        R = self.wheel_radius
        L = self.robot_width
        
        self.pose_x += dt * self.Vr * math.cos(self.pose_theta + dt * self.Wr / 2)
        self.pose_y += dt * self.Vr * math.sin(self.pose_theta + dt * self.Wr / 2) 
        self.pose_theta = wrap_to_pi(self.pose_theta + dt * self.Wr)
        
        # Compute Jacobian 
        H = np.array([[1, 0, -dt * self.Vr * math.sin(self.pose_theta)],
                      [0, 1,  dt * self.Vr * math.cos(self.pose_theta)],
                      [0, 0, 1]])
        
        # Compute process noise Jacobian 
        dH = np.array([[0.5 * dt * R * math.cos(self.pose_theta), 0.5 * dt * R * math.cos(self.pose_theta)],
                       [0.5 * dt * R * math.sin(self.pose_theta), 0.5 * dt * R * math.sin(self.pose_theta)],
                       [dt * R / L, -dt * R / L]])
        
        # Control input covariance 
        K = np.array([[self.sigma_squared * abs(self.velocityR), 0],
                      [0, self.sigma_squared * abs(self.velocityL)]])
        
        # Process noise covariance
        Q = dH @ K @ dH.T
        
        # Update covariance 
        self.Sig = H @ self.Sig @ H.T + Q

    def kalman_correction(self, markers, cov):
        """Kalman correction step"""
        try:
            S = [self.pose_x, self.pose_y, self.pose_theta]
            
            for marker in markers:
                found, M = self.get_landmark(marker[0])
                
                if found:
                    # Expected measurement
                    Z_hat = np.array([
                        (M[0] - S[0]) * math.cos(S[2]) + (M[1] - S[1]) * math.sin(S[2]),
                        -(M[0] - S[0]) * math.sin(S[2]) + (M[1] - S[1]) * math.cos(S[2]),
                        wrap_to_pi(M[2] - S[2])
                    ])
                    
                    # Measurement Jacobian 
                    G = np.array([
                        [-math.cos(S[2]), -math.sin(S[2]), -(M[0] - S[0]) * math.sin(S[2]) + (M[1] - S[1]) * math.cos(S[2])],
                        [math.sin(S[2]), -math.cos(S[2]), -(M[0] - S[0]) * math.cos(S[2]) - (M[1] - S[1]) * math.sin(S[2])],
                        [0, 0, -1]
                    ])
                    
                    # Measurement noise covariance 
                    R = np.array([[cov, 0, 0],
                                  [0, cov, 0],
                                  [0, 0, cov]])
                    
                    # Innovation
                    diff = np.array(marker[1:4]) - Z_hat
                    diff[2] = wrap_to_pi(diff[2])  
                    
                    # Innovation covariance
                    Z = G @ self.Sig @ G.T + R
                    
                    # Check for singular matrix
                    if np.linalg.det(Z) < 1e-10:
                        self.get_logger().warn(f"Singular innovation covariance for marker {marker[0]}")
                        continue
                    
                    # Kalman gain 
                    K = self.Sig @ G.T @ np.linalg.inv(Z)
                    
                    # Update pose mean 
                    S = S + K @ diff
                    
                    # Update covariance 
                    self.Sig = (np.eye(3) - K @ G) @ self.Sig
                    
                    self.get_logger().debug(f"Updated with marker {marker[0]}: innovation=({diff[0]:.3f}, {diff[1]:.3f}, {math.degrees(diff[2]):.1f}°)")
                else:
                    self.get_logger().warn(f"Landmark {marker[0]} not found in map")
            
            # Update pose
            self.pose_x = S[0]
            self.pose_y = S[1] 
            self.pose_theta = S[2]
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Kalman correction failed: {e}")
            return False

    def get_landmark(self, marker_id):
        """Get landmark from map """
        for mark in self.map:
            if mark[0] == marker_id:
                return True, mark[1:4]
        return False, [0, 0, 0]

    def create_odometry_message(self, timestamp):
        """Create odometry message"""
        odom = Odometry()
        
        odom.header.stamp = timestamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Position
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.position.z = 0.0

        # Orientation
        odom.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, self.pose_theta)

        # Velocities
        odom.twist.twist.linear.x = self.Vr
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.Wr

        # Covariance (matching kalman.py format)
        if self.use_kalman:
            odom.pose.covariance[0] = self.Sig[0][0]   # x-x
            odom.pose.covariance[1] = self.Sig[0][1]   # x-y  
            odom.pose.covariance[6] = self.Sig[1][0]   # y-x
            odom.pose.covariance[7] = self.Sig[1][1]   # y-y
            odom.pose.covariance[35] = self.Sig[2][2]  # θ-θ
        
        return odom

    def publish_transform(self, timestamp):
        """Publish TF transform"""
        t = TransformStamped()
        
        t.header.stamp = timestamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = self.pose_x
        t.transform.translation.y = self.pose_y
        t.transform.translation.z = 0.0
        
        t.transform.rotation = quaternion_from_euler(0.0, 0.0, self.pose_theta)
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedOdometry()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()