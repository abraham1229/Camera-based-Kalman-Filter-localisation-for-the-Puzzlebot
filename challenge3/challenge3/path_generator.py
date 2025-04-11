import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from msgs_clase.msg import Path   # type: ignore

class My_Talker_Params(Node):
    def __init__(self):
        super().__init__('Path_generator')
                
        #Se hacen las suscripciones pertinentes
        self.subscription_odometria = self.create_subscription(
            Int32,
            'path_type',
            self.signal_callback_type,
            1000) #Se debe de incluir la lectura de datos
        
        self.pub = self.create_publisher(Path, 'path_generator', 1000)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Path generator node initialized')
        self.msg = Path()
        self.velocidaLineal = 0.0
        self.velocidaAngular = 0.0
        self.type  = 1

    # Callback para recibir la posición actual del robot
    def signal_callback_type(self, msg):
        if msg is not None:
            self.type = msg.data

    def timer_callback(self):
        if self.type == 1:
            # Triángulo
            self.msg.x1 = 0.5
            self.msg.y1 = 0.87
            self.msg.x2 = 1.0
            self.msg.y2 = 0.0
            self.msg.x3 = 0.0
            self.msg.y3 = 0.0
            self.msg.x4 = 0.0
            self.msg.y4 = 0.0
            self.msg.x5 = 0.0
            self.msg.y5 = 0.0
            self.msg.x6 = 0.0
            self.msg.y6 = 0.0
            self.msg.x7 = 0.0
            self.msg.y7 = 0.0
            self.msg.x8 = 0.0
            self.msg.y8 = 0.0

        if self.type == 2:
            # Cuadrado
            self.msg.x1 = 1.0
            self.msg.y1 = 0.0
            self.msg.x2 = 1.0
            self.msg.y2 = 1.0
            self.msg.x3 = 0.0
            self.msg.y3 = 1.0
            self.msg.x4 = 0.0
            self.msg.y4 = 0.0
            self.msg.x5 = 0.0
            self.msg.y5 = 0.0
            self.msg.x6 = 0.0
            self.msg.y6 = 0.0
            self.msg.x7 = 0.0
            self.msg.y7 = 0.0
            self.msg.x8 = 0.0
            self.msg.y8 = 0.0

        if self.type == 3:
            # Pentágono
            self.msg.x1 = 0.35
            self.msg.y1 = 0.02
            self.msg.x2 = 0.9
            self.msg.y2 = 0.21
            self.msg.x3 = 0.9
            self.msg.y3 = 0.59
            self.msg.x4 = 0.35
            self.msg.y4 = 0.98
            self.msg.x5 = 0.0
            self.msg.y5 = 0.5
            self.msg.x6 = 0.35
            self.msg.y6 = 0.02
            self.msg.x7 = 0.0
            self.msg.y7 = 0.0
            self.msg.x8 = 0.0
            self.msg.y8 = 0.0
            

        if self.type == 4:
            # Hexágono
            self.msg.x1 = 0.25
            self.msg.y1 = 0.07
            self.msg.x2 = 0.75
            self.msg.y2 = 0.07
            self.msg.x3 = 1.0
            self.msg.y3 = 0.5
            self.msg.x4 = 0.75
            self.msg.y4 = 0.93
            self.msg.x5 = 0.25
            self.msg.y5 = 0.93
            self.msg.x6 = 0.0
            self.msg.y6 = 0.05
            self.msg.x7 = 0.25
            self.msg.y7 = 0.07
            self.msg.x8 = 0.0
            self.msg.y8 = 0.0

        

        self.pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    m_t_p = My_Talker_Params()
    rclpy.spin(m_t_p)
    m_t_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
