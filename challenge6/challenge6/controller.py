import rclpy
from rclpy.node import Node
from msgs_clase.msg import Goal   # type: ignore
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import transforms3d
import time

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')

        self.wait_for_ros_time()

        # Declare and get parameters
        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.declare_parameter('init_pose_yaw', 0.0)

        self.Posx = self.get_parameter('init_pose_x').value
        self.Posy = self.get_parameter('init_pose_y').value
        self.Postheta = self.get_parameter('init_pose_yaw').value
        self.use_sim = self.get_parameter('use_sim_time').value

        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1000)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.create_subscription(Odometry, 'odometria', self.callback_odometry, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Goal, 'path_generator', self.callback_path, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.callback_lidar, 10)

        self.Posx = 0.0
        self.Posy = 0.0
        self.Postheta = 0.0
        self.coordenadasMeta = []
        self.velL = 0.0
        self.velA = 0.0

        self.kpTheta = 0.8
        self.kpLineal = 0.4
        self.error_distancia = 0.0
        self.errorTheta = 0.0

        self.k_wall = 0.5

        self.trayectoria_finalizda = False
        self.state = 'GO_TO_GOAL'
        self.obstacle_threshold_enter = 0.5
        self.obstacle_threshold_exit = 0.6
        self.last_follow_wall_time = 0.0

    def timer_callback(self):
        if not self.coordenadasMeta:
            return

        if self.trayectoria_finalizda:
            self.stop_robot()
            return

        if self.state == 'GO_TO_GOAL':
            self.compute_errors()
            self.apply_control()
            self.limit_velocities()
            self.check_point_reached()
            self.publish_velocity_command()

        elif self.state == 'FOLLOW_WALL':
            self.wall_follow_logic()
            # Check if path to goal is clear
            if self.is_path_to_goal_clear() and (time.time() - self.last_follow_wall_time > 1.0):
                self.state = 'GO_TO_GOAL'
                self.get_logger().info('Path to goal clear, switching to GO_TO_GOAL')

    def stop_robot(self):
        twist = Twist()
        self.pub_cmd_vel.publish(twist)
        self.get_logger().warn('Meta alcanzada')

    def callback_odometry(self, msg):
        self.Posx = msg.pose.pose.position.x
        self.Posy = msg.pose.pose.position.y
        qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        _, _, yaw = transforms3d.euler.quat2euler([qw, qx, qy, qz])
        self.Postheta = yaw

    def callback_path(self, msg):
        self.coordenadasMeta = [msg.x_goal, msg.y_goal]

    def callback_lidar(self, msg):
        self.latest_ranges = msg.ranges
        self.lidar_msg = msg

        obstacle_in_front = any(self.get_range_from_lidar(angle) < self.obstacle_threshold_enter for angle in range(-15, 16))
        front_clear = all(self.get_range_from_lidar(angle) > self.obstacle_threshold_exit for angle in range(-15, 16))

        if self.state == 'GO_TO_GOAL' and obstacle_in_front:
            self.state = 'FOLLOW_WALL'
            self.last_follow_wall_time = time.time()
            self.get_logger().info('Cambio a FOLLOW_WALL')
        elif self.state == 'FOLLOW_WALL' and front_clear and (time.time() - self.last_follow_wall_time > 1.0):
            self.state = 'GO_TO_GOAL'
            self.get_logger().info('Regreso a GO_TO_GOAL')

    def compute_errors(self):
        goal_x, goal_y = self.coordenadasMeta[0], self.coordenadasMeta[1] 
        dx, dy = goal_x - self.Posx, goal_y - self.Posy
        self.error_distancia = math.hypot(dx, dy)
        target_theta = math.atan2(dy, dx)
        self.errorTheta = target_theta - self.Postheta
        self.errorTheta = (self.errorTheta + math.pi) % (2 * math.pi) - math.pi

    def apply_control(self):
        self.velA = self.kpTheta * self.errorTheta
        self.velL = self.kpLineal * self.error_distancia

    def limit_velocities(self):
        self.velA = max(min(self.velA, 0.25), -0.25)
        if abs(self.errorTheta) > 0.05:
            self.velL = 0.0
        else:
            self.velL = min(self.velL, 0.2)

    def check_point_reached(self):
        if abs(self.errorTheta) < 0.05 and self.error_distancia < 0.05:
            self.trayectoria_finalizda = True

    def publish_velocity_command(self):
        twist = Twist()
        twist.linear.x = self.velL
        twist.angular.z = self.velA
        self.pub_cmd_vel.publish(twist)

    def wall_follow_logic(self):
        desired_distance = 0.7
        max_linear = 0.2
        max_angular = 1.0

        front = self.get_range_from_lidar(0)
        left = self.get_range_from_lidar(90)
        left_corner = self.get_range_from_lidar(45)

        twist = Twist()

        if front < 0.8:
            self.get_logger().info("Obstáculo al frente. Girando a la derecha")
            twist.linear.x = 0.0
            twist.angular.z = -max_angular
        elif left < desired_distance:
            if left_corner < desired_distance / 2:
                self.get_logger().info("Muy cerca de la pared. Corrigiendo a la derecha")
                #twist.linear.x = max_linear
                #twist.angular.z = -max_angular / 2
                error = desired_distance - left
                twist.linear.x = max_linear
                twist.angular.z = np.clip(error * self.k_wall, -max_angular, max_angular)
            else:
                self.get_logger().info("Pared detectada a la izquierda. Avanzando recto")
                twist.linear.x = max_linear
                twist.angular.z = 0.0
        else:
            self.get_logger().info("Sin pared. Girando a la izquierda")
            twist.linear.x = max_linear / 2
            twist.angular.z = max_angular / 2

        self.pub_cmd_vel.publish(twist)
    
    def is_path_to_goal_clear(self):
        if not hasattr(self, 'lidar_msg'):
            return False
        goal_x, goal_y = self.coordenadasMeta
        dx, dy = goal_x - self.Posx, goal_y - self.Posy
        current_dist = math.hypot(dx, dy)
        goal_angle = math.atan2(dy, dx)
        robot_angle = self.Postheta
        rel_angle = (goal_angle - robot_angle + math.pi) % (2 * math.pi) - math.pi
        rel_angle_deg = np.degrees(rel_angle)
        # Check a small window around the goal direction
        for offset in range(-5, 6):
            angle = rel_angle_deg + offset
            dist = self.get_range_from_lidar(angle)
            if dist < self.error_distancia:  # Obstacle between robot and goal
                return False
        return True

    def get_range_from_lidar(self, angle_deg):
        msg = self.lidar_msg
        ranges = np.array(self.latest_ranges)
        angle_rad = np.radians(angle_deg)
        index = int((angle_rad - msg.angle_min) / msg.angle_increment)
        if 0 <= index < len(ranges):
            val = ranges[index]
            return val if not (np.isnan(val) or np.isinf(val)) else float('inf')
        return float('inf')

    def stop(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(twist_msg)
        self.get_logger().info('Stopping robot due to obstacle detection.')

    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            now = self.get_clock().now()
            if now.nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'ROS time is active! Start time: {now.nanoseconds * 1e-9:.2f}s')

    def stop_handler(self, signum, frame):
        """Handles Ctrl+C (SIGINT)."""
        self.get_logger().info("Interrupt received! Stopping node...")
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
