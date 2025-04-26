import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float32
from msgs_clase.msg import Vector   # type: ignore
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import JointState
import transforms3d
import numpy as np

class DronePublisher(Node):

    def __init__(self):
        super().__init__('frame_publisher')

        self.namespace = self.get_namespace().rstrip('/')

        # Declare the parameter with a default value
        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.declare_parameter('init_pose_z', 0.0)
        self.declare_parameter('init_pose_yaw', np.pi/2)
        self.declare_parameter('init_pose_pitch', 0.0)
        self.declare_parameter('init_pose_roll', 0.0)
        self.declare_parameter('odom_frame', 'odom')

        # Retrieve the parameter value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value.strip('/')

        # Crear publisher de velocidad de las llantas
        self.pub_r_vel = self.create_publisher(Float32, 'VelocityEncR', 1000)
        self.pub_l_vel = self.create_publisher(Float32, 'VelocityEncL', 1000)


        #Se hacen las suscripciones pertinentes
        self.subscription_odometry = self.create_subscription(
            Vector,
            'odometria',
            self.callback_odometry,
            rclpy.qos.qos_profile_sensor_data ) #Se debe de incluir la lectura de datos
        
        #Se hacen las suscripciones pertinentes
        self.subscription_odometry = self.create_subscription(
            Twist,
            'cmd_vel',
            self.callback_cmd_vel,
            rclpy.qos.qos_profile_sensor_data ) #Se debe de incluir la lectura de datos
        
    
        #Puzzlebot Initial Pose
        self.intial_pos_x = self.get_parameter('init_pose_x').value
        self.intial_pos_y = self.get_parameter('init_pose_y').value
        self.intial_pos_z = self.get_parameter('init_pose_z').value
        self.intial_pos_yaw = self.get_parameter('init_pose_yaw').value
        self.intial_pos_pitch = self.get_parameter('init_pose_pitch').value
        self.intial_pos_roll = self.get_parameter('init_pose_roll').value

        # Puzzlebot odometry
        self.Posx = self.intial_pos_x
        self.Posy = self.intial_pos_y
        self.Postheta = self.intial_pos_yaw


        #Angular velocity for wheels puzzlebot
        self.omega = 0.5
        self.cmd_vel_linear = 0.0
        self.cmd_vel_angular = 0.0

        #Variables físicas del robot
        #Valor del radio de la llanta
        self.radius = 0.05
        #Valor de la distancia entre llantas
        self.lenght = 0.19

        #Define Transformations
        self.define_TF()

        #initialise Message to be published
        self.ctrlJoints = JointState()
        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.name = ["wheel_right_joint","wheel_left_joint"]
        self.ctrlJoints.position = [0.0] * 2
        self.position_prev = [0.0] * 2
        self.ctrlJoints.velocity = [0.0] * 2
        self.ctrlJoints.effort = [0.0] * 2
        self.wheel_right_vel = 0.0
        self.wheel_left_vel = 0.0

        #Create Transform Boradcasters
        self.tf_br_base = TransformBroadcaster(self)
        self.tf_br_base_footprint = TransformBroadcaster(self)

        #Publisher
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        

        #Create a Timer
        self.timer_period = 0.01 #seconds
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        # Message 
        self.get_logger().info('Puzzlebot node initialized')



    #Timer Callback
    def timer_cb(self):
       
        self.base_footprint_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint_tf.transform.translation.x = self.Posx
        self.base_footprint_tf.transform.translation.y = self.Posy
        self.base_footprint_tf.transform.translation.z = 0.0
        q = transforms3d.euler.euler2quat(0, 0, self.Postheta)
        self.base_footprint_tf.transform.rotation.x = q[1]
        self.base_footprint_tf.transform.rotation.y = q[2]
        self.base_footprint_tf.transform.rotation.z = q[3]
        self.base_footprint_tf.transform.rotation.w = q[0]

        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()

        
        # Velocidad desde cmd_vel

        right_distance = (self.cmd_vel_linear + (self.lenght / 2) * self.cmd_vel_angular) / self.radius * self.timer_period
        left_distance = (self.cmd_vel_linear - (self.lenght / 2) * self.cmd_vel_angular) / self.radius * self.timer_period

        self.ctrlJoints.position[0] += right_distance
        self.ctrlJoints.position[1] += left_distance


        # Velocidad para encoders
        self.wheel_right_vel = (self.ctrlJoints.position[0] - self.position_prev[0])/self.timer_period
        self.wheel_left_vel = (self.ctrlJoints.position[1] - self.position_prev[1])/self.timer_period

        # Se actualiza posicion anterior
        self.position_prev[0] = self.ctrlJoints.position[0]
        self.position_prev[1] = self.ctrlJoints.position[1]



        self.tf_br_base.sendTransform(self.base_footprint_tf)

        # Se publican valores

        right_wheel = Float32()
        left_wheel = Float32()
        right_wheel.data = self.wheel_right_vel
        left_wheel.data = self.wheel_left_vel

        self.pub_r_vel.publish(right_wheel)
        self.pub_l_vel.publish(left_wheel)

        self.publisher.publish(self.ctrlJoints)


    #Lee los datos del nodo de la llanta derecha
    def callback_odometry(self, msg):
        if msg is not None:
            self.Posx = msg.x
            self.Posy = msg.y
            self.Postheta = msg.theta

    def callback_cmd_vel(self, msg):
        if msg is not None:
            self.cmd_vel_linear = msg.linear.x
            self.cmd_vel_angular = msg.angular.z

    def define_TF(self):

        #Create Transform Messages
        self.base_footprint_tf = TransformStamped()
        self.base_footprint_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint_tf.header.frame_id = self.odom_frame
        self.base_footprint_tf.child_frame_id = f"{self.namespace}/base_footprint"
        self.base_footprint_tf.transform.translation.x = self.intial_pos_x
        self.base_footprint_tf.transform.translation.y = self.intial_pos_y
        self.base_footprint_tf.transform.translation.z = 0.0
        q_foot = transforms3d.euler.euler2quat(self.intial_pos_roll, self.intial_pos_pitch, self.intial_pos_yaw)       
        self.base_footprint_tf.transform.rotation.x = q_foot[1]
        self.base_footprint_tf.transform.rotation.y = q_foot[2]
        self.base_footprint_tf.transform.rotation.z = q_foot[3]
        self.base_footprint_tf.transform.rotation.w = q_foot[0]



def main(args=None):
    rclpy.init(args=args)

    node = DronePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()