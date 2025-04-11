import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from msgs_clase.msg import Vector   # type: ignore
import math

#Se crea el nodo My_Talker_Params tomando objeto de Node.
class Odometry_Node(Node):   

    #Se inicializa el nodo
    def __init__(self):
        #Se crear el nodo que será encargado de publicar la señal
        super().__init__('Odometria')

        #Se hacen las suscripciones pertinentes
        self.subscription_velocity_left = self.create_subscription(
            Float32,
            'VelocityEncL',
            self.signal_callback_left,
            rclpy.qos.qos_profile_sensor_data ) #Se debe de incluir la lectura de datos
        
        self.subscription_velocity_right = self.create_subscription(
            Float32,
            'VelocityEncR',
            self.signal_callback_right,
            rclpy.qos.qos_profile_sensor_data)
        
        #Se crea el publicador que mandará mensaje personalizado
        self.pub_odometry = self.create_publisher(Vector, 'odometria', 1000)
        # Período de temporizador para 10Hz
        self.timer_period = 0.01 
        #Se declara el timer que llamará al callback
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        #DECLARACIÓN DE VARIABLES

        #Variables físicas del robot
        #Valor del radio de la llanta
        self.radius = 0.05
        #Valor de la distancia entre llantas
        self.lenght = 0.19

        #Variables de velocidad
        self.vel_left = 0.0 #Variable de lectura izquierda de la llanta
        self.vel_right = 0.0 #Variable de lectura derecha de la llanta
        
        #Obtención de velocidades
        self.velocidadTheta = 0.0 #Velocidad thetha
        self.velLineal = 0.0 #Variable para obtener las velocidades del robot

        #Variables de odometría
        self.theta = 0.0
        self.posX = 0.0
        self.posY = 0.0

        #Se declara mensaje personalizado
        self.msgDato = Vector()

        #Se despliega en pantalla que se ha inicializado el nodo
        self.get_logger().info('Odometry node initialized')

        
    #Lee los datos del nodo de la llanta izquierda
    def signal_callback_left(self, msg):
        if msg is not None:
            self.vel_left = msg.data

    #Lee los datos del nodo de la llanta derecha
    def signal_callback_right(self, msg):
        if msg is not None:
            self.vel_right = msg.data

    
    #Se hace callback en el que se calcula la posición en x, y y theta
    def timer_callback(self):

        #Se calcula theta punto
        self.velocidadTheta = self.radius*((self.vel_right-self.vel_left)/self.lenght)
        #Se calcula argumento velocidad
        self.velLineal = self.radius*((self.vel_right+self.vel_left)/2) * 1.09
        
        # Se hace cambio de signo del ángulo si es necesario.
        if self.theta >= math.pi:
            self.theta -= 2 * math.pi
        elif self.theta <= -math.pi:
            self.theta += 2 * math.pi
        
        # self.theta += self.velocidadTheta * self.timer_period * 1.069
        self.theta += self.velocidadTheta * self.timer_period * 1.1722
        
                
        #Se calcula posición en x y y
        self.posX += self.velLineal*math.cos(self.theta) *self.timer_period
        self.posY += self.velLineal*math.sin(self.theta) *self.timer_period
        
        #Se declara el mensaje costumizado
        self.msgDato.x = self.posX
        self.msgDato.y = self.posY
        self.msgDato.theta = self.theta
        
        #Se publica el dato
        self.pub_odometry.publish(self.msgDato)

        

#La función que será llamada según nuestro setup
def main(args=None):
    #Se inicializa el entorno de ros
    rclpy.init(args=args)
    #Se crea una instancia de la clase creada previamente.
    m_t_p = Odometry_Node()
    #Creación del bucle de eventos
    rclpy.spin(m_t_p)
    #Se liberan recursos
    m_t_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
