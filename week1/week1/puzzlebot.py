import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import transforms3d
import numpy as np

class FramePublisher(Node):

    def __init__(self):
        super().__init__('frame_publisher_ri', namespace='robo_inges')
        
        # Static frames
        self.map = StaticTransformBroadcaster(self)
        self.odom = StaticTransformBroadcaster(self)
        self.base_link = StaticTransformBroadcaster(self)
        self.caster_link = StaticTransformBroadcaster(self)

        # # Map values
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'map'
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        q = transforms3d.euler.euler2quat(0, 0, 0)      #input euler2quat(roll, pitch, yaw) , output q=[w, x, y, z] 
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]

        # # Odom values
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'map'
        t2.child_frame_id = 'odom'
        t2.transform.translation.x = 1.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        q = transforms3d.euler.euler2quat(0, 0, 0)      #input euler2quat(roll, pitch, yaw) , output q=[w, x, y, z] 
        t2.transform.rotation.x = q[1]
        t2.transform.rotation.y = q[2]
        t2.transform.rotation.z = q[3]
        t2.transform.rotation.w = q[0]

        # Base link values
        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = 'base_footprint'
        t3.child_frame_id = 'base_link'
        t3.transform.translation.x = 0.0*5
        t3.transform.translation.y = 0.0*5
        t3.transform.translation.z = 0.5*5
        q = transforms3d.euler.euler2quat(0, 0, 0)      #input euler2quat(roll, pitch, yaw) , output q=[w, x, y, z] 
        t3.transform.rotation.x = q[1]
        t3.transform.rotation.y = q[2]
        t3.transform.rotation.z = q[3]
        t3.transform.rotation.w = q[0]

        # Caster link values
        t4 = TransformStamped()
        t4.header.stamp = self.get_clock().now().to_msg()
        t4.header.frame_id = 'base_link'
        t4.child_frame_id = 'caster_link'
        t4.transform.translation.x = -0.095*5
        t4.transform.translation.y = 0.0*5
        t4.transform.translation.z = -0.03*5
        q = transforms3d.euler.euler2quat(0, 0, 0)      #input euler2quat(roll, pitch, yaw) , output q=[w, x, y, z] 
        t4.transform.rotation.x = q[1]
        t4.transform.rotation.y = q[2]
        t4.transform.rotation.z = q[3]
        t4.transform.rotation.w = q[0]




        # Publish the static nodes
        self.map.sendTransform(t)
        self.odom.sendTransform(t2)
        self.base_link.sendTransform(t3)
        self.caster_link.sendTransform(t4)



        # Dynamic frames
        #Create Trasnform Messages
        self.base_footprint = TransformStamped()
        self.wheel_r_link = TransformStamped()
        self.wheel_l_link = TransformStamped()
        
        #Create Transform Boradcasters
        self.tf_base_footprint = TransformBroadcaster(self)
        self.tf_wheel_r_link = TransformBroadcaster(self)
        self.tf_wheel_l_link = TransformBroadcaster(self)

        #Create a Timer
        timer_period = 0.1 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)

        #Variables to be used
        self.start_time = self.get_clock().now()
        self.omega = 0.1

    #Timer Callback
    def timer_cb(self):

        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds/1e9

        # Base footprint data 
        self.base_footprint.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint.header.frame_id = 'odom'
        self.base_footprint.child_frame_id = 'base_footprint'
        self.base_footprint.transform.translation.x = 1.0 + elapsed_time*0.03
        self.base_footprint.transform.translation.y = 0.0
        self.base_footprint.transform.translation.z = 0.0
        q = transforms3d.euler.euler2quat(0, 0, 0)      #input euler2quat(roll, pitch, yaw) , output q=[w, x, y, z] 
        self.base_footprint.transform.rotation.x = q[1]
        self.base_footprint.transform.rotation.y = q[2]
        self.base_footprint.transform.rotation.z = q[3]
        self.base_footprint.transform.rotation.w = q[0]


        # Right wheel data 
        self.wheel_r_link.header.stamp = self.get_clock().now().to_msg()
        self.wheel_r_link.header.frame_id = 'base_link'
        self.wheel_r_link.child_frame_id = 'wheel_r_link'
        self.wheel_r_link.transform.translation.x = 0.052*3
        self.wheel_r_link.transform.translation.y = -0.095*3
        self.wheel_r_link.transform.translation.z = -0.0025*3
        q = transforms3d.euler.euler2quat(0, elapsed_time, 0)      #input euler2quat(roll, pitch, yaw) , output q=[w, x, y, z] 
        self.wheel_r_link.transform.rotation.x = q[1]
        self.wheel_r_link.transform.rotation.y = q[2]
        self.wheel_r_link.transform.rotation.z = q[3]
        self.wheel_r_link.transform.rotation.w = q[0]

        # Left wheel data 
        self.wheel_l_link.header.stamp = self.get_clock().now().to_msg()
        self.wheel_l_link.header.frame_id = 'base_link'
        self.wheel_l_link.child_frame_id = 'wheel_l_link'
        self.wheel_l_link.transform.translation.x = 0.052*3
        self.wheel_l_link.transform.translation.y = 0.095*3
        self.wheel_l_link.transform.translation.z = -0.0025*3
        q = transforms3d.euler.euler2quat(0, elapsed_time, 0)      #input euler2quat(roll, pitch, yaw) , output q=[w, x, y, z] 
        self.wheel_l_link.transform.rotation.x = q[1]
        self.wheel_l_link.transform.rotation.y = q[2]
        self.wheel_l_link.transform.rotation.z = q[3]
        self.wheel_l_link.transform.rotation.w = q[0]


        # Publish data
        self.tf_base_footprint.sendTransform(self.base_footprint)
        self.tf_wheel_r_link.sendTransform(self.wheel_r_link)
        self.tf_wheel_l_link.sendTransform(self.wheel_l_link)

def main(args=None):
    rclpy.init(args=args)

    node = FramePublisher()

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