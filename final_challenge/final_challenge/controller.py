import rclpy
from rclpy.node import Node
from msgs_clase.msg import Goal          # type: ignore
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math
import transforms3d
import numpy as np

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # Parámetros de control PD
        self.kp_linear = 0.8
        self.kd_linear = 0.2
        self.kp_angular = 1.5
        self.kd_angular = 0.3

        # Estado del path follower
        self.goal = None
        self.Posx = self.Posy = self.Postheta = 0.0
        self.prev_error_dist = 0.0
        self.prev_error_theta = 0.0
        self.prev_Posx = self.prev_Posy = 0.0
        self.goal_idx = 0
        self.final_goal_reached = False
        self.last_goal_time = self.get_clock().now()

        # Variables para Bug algorithm
        self.lidar_msg = None
        self.kp = 1.0
        self.wall_desired = 0.5      # distancia deseada a la pared derecha
        self.max_linear = 0.3
        self.max_angular = 0.5
        self.threshold_front = 0.4   # si algo está más cerca, se considera obstáculo

        # Bug 2
        self.initial_point_x = 0.0
        self.initial_point_y = 0.0
        self.mline_slope = None
        self.mline_intercept = None
        self.last_state_change_time = self.get_clock().now()
        self.min_state_duration = 0.05 # segundos
        self.default_distance = 2.5
        self.danger_distance = 0.17

        # Estado de la trayectoria
        self.state = 'GO_TO_GOAL'


        # Publicadores y suscriptores
        qos = rclpy.qos.qos_profile_sensor_data
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_next_goal = self.create_publisher(Bool, 'next_goal', 10)
        self.create_subscription(Odometry, 'odometria', self.callback_odometry, qos)
        self.create_subscription(Goal, 'path_generator', self.callback_goal, qos)
        self.create_subscription(LaserScan, 'scan', self.callback_lidar, qos)

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('Controller node initialized')

    def callback_odometry(self, msg: Odometry):
        ''' Callback which updates the robot's position and orientation from odometry messages. '''
        if msg is None:
            return
        self.Posx = msg.pose.pose.position.x
        self.Posy = msg.pose.pose.position.y
        qx, qy, qz, qw = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, yaw = transforms3d.euler.quat2euler([qw, qx, qy, qz])
        self.Postheta = yaw

    def callback_goal(self, msg: Goal):
        ''' Callback which updates the goal position from the path generator messages. '''
        # Detectar mensaje especial de fin (inf)
        if math.isinf(msg.x_goal) or math.isinf(msg.y_goal):
            self.final_goal_reached = True
            self.print_success('¡Meta final alcanzada!')
            return
        
        new_goal = (msg.x_goal, msg.y_goal)
        if new_goal == self.goal:
            return

        # Asignar nuevo goal
        self.goal = new_goal
        self.print_success(f'x {msg.x_goal}  y {msg.y_goal} ')

        # Evitar recalcular M-line si el nuevo goal está demasiado cerca del robot
        dist_to_goal = math.hypot(self.goal[0] - self.Posx, self.goal[1] - self.Posy)
        if dist_to_goal < 0.1:
            self.get_logger().info("Meta demasiado cercana, no se recalcula M-line")
            return

        # Guardar punto inicial y M-line
        self.initial_point_x = self.Posx
        self.initial_point_y = self.Posy
        x1, y1 = self.initial_point_x, self.initial_point_y
        x2, y2 = self.goal

        if abs(x2 - x1) > 1e-6:
            self.mline_slope = (y2 - y1) / (x2 - x1)
            self.mline_intercept = y1 - self.mline_slope * x1
        else:
            self.mline_slope = float('inf')
            self.mline_intercept = x1  
    
    def callback_lidar(self, msg: LaserScan):
        ''' Callback which updates the LiDAR message and changes the state based on wall distance. '''
        self.lidar_msg = msg

        dist_wall = self.get_distance_at_angle_range(15, -5)

        previous_state = self.state  # Store the previous state

        if dist_wall < self.threshold_front:
            self.state = 'FOLLOW_WALL'
        else:
            self.state = 'GO_TO_GOAL'

        # Only log/publish when the state changes
        if self.state != previous_state:
            self.last_state_change_time = self.get_clock().now()
            self.get_logger().info(f"estado: {self.state}")
        

    def timer_callback(self):
        ''' Timer callback which executes the control logic based on the current state. '''
        if self.final_goal_reached or self.goal is None:
            return
        if self.state == 'GO_TO_GOAL':
            self.go_to_goal()
        elif self.state == 'FOLLOW_WALL':
            self.wall_follower()
        

    def go_to_goal(self):
        ''' Control logic to move towards the current goal using PD control. '''
        gx, gy = self.goal
        dx = gx - self.Posx
        dy = gy - self.Posy
        error_dist = math.hypot(dx, dy)
        target_theta = math.atan2(dy, dx)
        error_theta = (target_theta - self.Postheta + math.pi) % (2 * math.pi) - math.pi

        # Derivadas
        d_error_dist = (error_dist - self.prev_error_dist) / self.timer_period
        d_error_theta = (error_theta - self.prev_error_theta) / self.timer_period

        # Control PD
        v = self.kp_linear * error_dist + self.kd_linear * d_error_dist
        w = self.kp_angular * error_theta + self.kd_angular * d_error_theta

        # Limitar velocidades
        v = max(min(v, self.max_linear), -self.max_linear)
        w = max(min(w, self.max_angular), -self.max_angular)

        # --- Producto escalar para detectar cruce del objetivo ---
    
        dx_prev = gx - self.prev_Posx
        dy_prev = gy - self.prev_Posy
        dx_now = gx - self.Posx
        dy_now = gy - self.Posy
        dot_product = dx_prev * dx_now + dy_prev * dy_now

        now = self.get_clock().now()
        time_since_last_goal = (now - self.last_goal_time).nanoseconds / 1e9
        reached = False
        if error_dist < 0.05 and time_since_last_goal > 0.5:
            reached = True
        elif dot_product < 0 and time_since_last_goal > 0.5:
            reached = True
            self.get_logger().info('Objetivo cruzado por producto escalar')
        if reached:
            v = 0.0
            w = 0.0
            self.goal_idx += 1
            self.prev_error_dist = 0.0
            self.prev_error_theta = 0.0
            self.print_success(f'Punto {self.goal_idx} alcanzado')
            # Publicar señal para avanzar al siguiente objetivo
            msg = Bool()
            msg.data = True
            self.pub_next_goal.publish(msg)
            self.last_goal_time = now

        if abs(error_theta) > 0.4: #to make sure it will reach the point
            v = 0.0

        # Publicar comando
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.pub_cmd_vel.publish(twist)

        # Guardar errores previos
        self.prev_error_dist = error_dist
        self.prev_error_theta = error_theta
        self.prev_Posx = self.Posx
        self.prev_Posy = self.Posy

    
    def wall_follower(self):
        ''' Control logic to follow the wall on the right side using Bug 0 algorithm. '''
        dist_right = self.get_distance_at_angle(-90)
        dist_right_45 = self.get_distance_at_angle(-45)
        dist_right_side = np.mean([dist_right, dist_right_45])
        dist_front_mean = self.get_distance_at_angle_range(-25,25)
        dist_all_front = self.get_distance_at_angle_range(-90,90)

        twist = Twist()

        if not hasattr(self, 'was_frente'):
            self.was_frente = False

        if dist_all_front < self.danger_distance:
            twist.linear.x = 0.0
            twist.angular.z = self.max_angular

        elif dist_front_mean > self.threshold_front:
            error = dist_right_side - self.wall_desired
            turn_rate = -error * self.kp
            twist.linear.x = self.max_linear
            twist.angular.z = turn_rate
            if abs(error) > 0.9: 
                twist.linear.x = self.max_linear / 5 

        else:
            # Obstáculo al frente
            if not self.was_frente:
                pass
                #self.get_logger().info("Frente")
            twist.linear.x = 0.0
            twist.angular.z = self.max_angular

        # Update was_frente state
        self.was_frente = (dist_all_front >= self.danger_distance and dist_front_mean <= self.threshold_front)

        # Limitar velocidades
        twist.linear.x = max(min(twist.linear.x, self.max_linear), -self.max_linear)
        twist.angular.z = max(min(twist.angular.z, self.max_angular), -self.max_angular)

        # Publicar comando final
        self.pub_cmd_vel.publish(twist)

    def get_distance_at_angle(self, angle_deg):
        ''' Get the distance from the LiDAR at a specific angle in degrees. '''
        if self.lidar_msg is None:
            return self.default_distance

        # Convertir a radianes
        angle_rad_input = math.radians(angle_deg)

        # Detectar si el LiDAR va de -pi a pi o de 0 a 2pi
        lidar_min = self.lidar_msg.angle_min
        lidar_max = self.lidar_msg.angle_max

        if lidar_min < 0 and lidar_max <= math.pi:  
            angle_rad = ((angle_rad_input + math.pi) % (2 * math.pi)) - math.pi
        else:  
            angle_rad = angle_rad_input % (2 * math.pi)

        # Calcular el índice correspondiente
        index = int((angle_rad - lidar_min) / self.lidar_msg.angle_increment)
        if 0 <= index < len(self.lidar_msg.ranges):
            value = self.lidar_msg.ranges[index]
            if not math.isnan(value) and not math.isinf(value):
                return value
        return self.default_distance
    
    def get_distance_at_angle_range(self, angle_start_deg, angle_end_deg):
        ''' Get the minimum distance from the LiDAR within a specific angle range in degrees. '''
        if self.lidar_msg is None:
            return self.default_distance
        
        # Se distingue si es el puzzle o simulacion
        if self.lidar_msg.angle_min < 0 and self.lidar_msg.angle_max <= math.pi:
            # Rango tipo (-π, π)
            angle_start_rad = ((math.radians(angle_start_deg) + math.pi) % (2 * math.pi)) - math.pi
            angle_end_rad = ((math.radians(angle_end_deg) + math.pi) % (2 * math.pi)) - math.pi
        else:
            # Rango tipo (0, 2π)
            angle_start_rad = math.radians(angle_start_deg) % (2 * math.pi)
            angle_end_rad = math.radians(angle_end_deg) % (2 * math.pi)

        # Asegurar que el inicio sea menor que el fin en ángulos
        if angle_end_rad < angle_start_rad:
            angle_end_rad += 2 * math.pi

        min_dist = float('inf')

        angle = self.lidar_msg.angle_min
        for i, r in enumerate(self.lidar_msg.ranges):
            if not math.isnan(r) and not math.isinf(r):
                current_angle = angle % (2 * math.pi)
                if angle_start_rad <= current_angle <= angle_end_rad:
                    min_dist = min(min_dist, r)
            angle += self.lidar_msg.angle_increment

        return min_dist if min_dist != float('inf') else self.default_distance

    
    def is_on_mline(self, tolerance=0.4):
        ''' Check if the robot is on the M-line defined by the initial point and the goal. '''
        if self.mline_slope is None:
            return False

        if self.mline_slope == float('inf'):
            error = abs(self.Posx - self.mline_intercept)
        elif self.mline_slope == 0:
            error = abs(self.Posy - self.mline_intercept)
        else:
            expected_y = self.mline_slope * self.Posx + self.mline_intercept
            error = abs(self.Posy - expected_y)

        return error < tolerance
    
    def can_change_state(self):
        ''' Check if enough time has passed since the last state change to allow a new state change. '''
        now = self.get_clock().now()
        elapsed = (now - self.last_state_change_time).nanoseconds / 1e9
        return elapsed > self.min_state_duration



    def print_success(self, msg):
        GREEN = '\033[92m'
        RESET = '\033[0m'
        self.get_logger().info(f'{GREEN}{msg}{RESET}')
        
def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()