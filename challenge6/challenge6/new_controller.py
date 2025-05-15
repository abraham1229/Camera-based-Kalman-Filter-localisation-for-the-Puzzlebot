import rclpy
from rclpy.node import Node
from msgs_clase.msg import Goal   # type: ignore
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import smach
import numpy as np
import transforms3d

# --- Define States ---
class GoToGoal(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['obstacle_detected','goal_reached', 'continue'])
        self.node = node

    def execute(self, userdata):
        node = self.node
        node.get_logger().info('State: GO_TO_GOAL')

        # Compute distance and heading error
        dx = node.coordenadasMeta[0] - node.Posx
        dy = node.coordenadasMeta[1] - node.Posy
        dist = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        err_theta = self._normalize_angle(heading - node.Postheta)

        # Check for obstacle
        front_clear = node.is_path_to_goal_clear()
        if dist < node.goal_tolerance:
            return 'goal_reached'
        if not front_clear:
            return 'obstacle_detected'

        # Publish velocities
        cmd = node.create_twist(linear=node.kpLineal * dist,
                                angular=node.kpTheta * err_theta)
        node.pub_cmd_vel.publish(cmd)
        return 'continue'

    def _normalize_angle(self, angle):
        while angle > math.pi: angle -= 2*math.pi
        while angle < -math.pi: angle += 2*math.pi
        return angle

class FollowWall(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['path_clear','still_following'])
        self.node = node

    def execute(self, userdata):
        node = self.node
        node.get_logger().info('State: FOLLOW_WALL')
        linear, angular = node.wall_follow_control()
        node.pub_cmd_vel.publish(node.create_twist(linear, angular))

        if node.is_path_to_goal_clear():
            return 'path_clear'
        return 'still_following'

# --- Controller Node ---
class Bug0Controller(Node):
    def __init__(self):
        super().__init__('bug0_with_smach')

        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.declare_parameter('init_pose_yaw', 0.0)
        self.declare_parameter('kp_theta', 0.8)
        self.declare_parameter('kp_lineal', 0.5)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('use_sim_time', False)

        self.Posx = self.get_parameter('init_pose_x').value
        self.Posy = self.get_parameter('init_pose_y').value
        self.Postheta = self.get_parameter('init_pose_yaw').value
        self.kpTheta = self.get_parameter('kp_theta').value
        self.kpLineal = self.get_parameter('kp_lineal').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.use_sim = self.get_parameter('use_sim_time').value

        self.coordenadasMeta = [5.0, 5.0]
        self.latest_ranges = []
        self.lidar_msg = None
        self.error_distancia = 0.5

        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1000)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.create_subscription(Odometry, 'odometria', self.callback_odometry, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Goal, 'path_generator', self.callback_path, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.callback_lidar, 10)

    def run(self):
        sm = smach.StateMachine(outcomes=['DONE'])
        with sm:
            smach.StateMachine.add('GO_TO_GOAL', GoToGoal(self),
                                   transitions={'obstacle_detected':'FOLLOW_WALL',
                                                'goal_reached':'DONE',
                                                'continue':'GO_TO_GOAL'})
            smach.StateMachine.add('FOLLOW_WALL', FollowWall(self),
                                   transitions={'path_clear':'GO_TO_GOAL',
                                                'still_following':'FOLLOW_WALL'})

        outcome = sm.execute()
        self.get_logger().info(f'State machine finished with outcome: {outcome}')

    def callback_odometry(self, msg: Odometry):
        self.Posx = msg.pose.pose.position.x
        self.Posy = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        _, _, yaw = transforms3d.euler.quat2euler([qw, qx, qy, qz])
        self.Postheta = yaw

    def callback_path(self, msg: Goal):
        self.coordenadasMeta = [msg.x_goal, msg.y_goal]

    def callback_lidar(self, msg: LaserScan):
        self.latest_ranges = msg.ranges
        self.lidar_msg = msg

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
        for offset in range(-5, 6):
            angle = rel_angle_deg + offset
            dist = self.get_range_from_lidar(angle)
            if dist < self.error_distancia:
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

    def wall_follow_control(self):
        side_distance = self.get_range(90)
        error = side_distance - 0.5
        angular = -1.0 * error
        linear = 0.15
        return linear, angular

    def create_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def get_range(self, angle):
        if not self.latest_ranges:
            return float('inf')
        angle = angle % 360
        index = int(angle / 360.0 * len(self.latest_ranges))
        return self.latest_ranges[index] if 0 <= index < len(self.latest_ranges) else float('inf')

    def timer_callback(self):
        pass

# --- Entry Point ---
def main(args=None):
    rclpy.init(args=args)
    node = Bug0Controller()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
