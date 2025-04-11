import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import transforms3d
import numpy as np

class DronePublisher(Node):

    def __init__(self):
        super().__init__('frame_publisher')

        #Puzzlebot Initial Pose
        self.intial_pos_x = 1.0
        self.intial_pos_y = 1.0
        self.intial_pos_z = 0.0
        self.intial_pos_yaw = np.pi/2
        self.intial_pos_pitch = 0.0
        self.intial_pos_roll = 0.0


        #Angular velocity for wheels puzzlebot
        self.omega = 0.5

        #Define Transformations
        self.define_TF()

        #initialise Message to be published
        self.ctrlJoints = JointState()
        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.name = ["wheel_right_joint","wheel_left_joint"]
        self.ctrlJoints.position = [0.0] * 2
        self.ctrlJoints.velocity = [0.0] * 2
        self.ctrlJoints.effort = [0.0] * 2

        #Create Transform Boradcasters
        self.tf_br_base = TransformBroadcaster(self)
        self.tf_br_base_footprint = TransformBroadcaster(self)

        #Publisher
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        

        #Create a Timer
        timer_period = 0.01 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)


    #Timer Callback
    def timer_cb(self):

        time = self.get_clock().now().nanoseconds/1e9

       
        self.base_footprint_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint_tf.transform.translation.x = self.intial_pos_x + 0.5*np.cos(self.omega*time)
        self.base_footprint_tf.transform.translation.y = self.intial_pos_y + 0.5*np.sin(self.omega*time)
        self.base_footprint_tf.transform.translation.z = 0.0
        q = transforms3d.euler.euler2quat(0, 0, self.intial_pos_yaw+self.omega*time)
        self.base_footprint_tf.transform.rotation.x = q[1]
        self.base_footprint_tf.transform.rotation.y = q[2]
        self.base_footprint_tf.transform.rotation.z = q[3]
        self.base_footprint_tf.transform.rotation.w = q[0]

        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.position[0] = self.omega*time
        self.ctrlJoints.position[1] = self.omega*time

        self.tf_br_base.sendTransform(self.base_footprint_tf)

        self.publisher.publish(self.ctrlJoints)

    def define_TF(self):

        #Create Transform Messages
        self.base_footprint_tf = TransformStamped()
        self.base_footprint_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint_tf.header.frame_id = 'odom'
        self.base_footprint_tf.child_frame_id = 'base_footprint'
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