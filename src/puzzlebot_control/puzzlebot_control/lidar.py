# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from std_msgs.msg import Float32
# import numpy as np
# from rclpy import qos
# import transforms3d

# class ObstacleAvoider(Node):
#     def __init__(self):
#         super().__init__('obstacle_avoider')
#         self.sub = self.create_subscription(LaserScan, 'scan', self.lidar_cb, 10)
#         self.sub = self.create_subscription(Odometry,'odom',self.odom_cb,qos.qos_profile_sensor_data)
#         self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.ranges = None  #LiDAR
#         self.safe_distance = 0.5  # meters
#         self.current_pose = None
#         self.state = 0
#         self.side_count = 0
#         self.min_dist = None
#         #self.goal_pose = None
#         self.Kp_linear = 0.1
#         self.Ki_linear = 0.01
#         self.Kd_linear = 0.001
#         self.Kp_ang = 1.0
#         self.Kd_ang = 0.05
#         self.Ki_linear = 0.2
#         self.integral_linear = 0.0
#         self.integral_ang = 0.0
#         self.prev_error_linear = 0.0
#         self.prev_error_ang = 0.0
#         self.position_threshold = 0.05
#         self.angle_threshold = 0.05
#         self.timer_period = 0.1
#         self.timer = self.create_timer(self.timer_period,self.control_loop)
        
#     def odom_cb(self,msg):
#         self.current_pose = msg.pose.pose
#         # if self.goal_pose is None and self.current_pose is not None:
#         #     self.set_new_goal()
        
#     def lidar_cb(self, msg):
#         self.ranges = [r for r in msg.ranges if r > 0.0]
#         if not self.ranges:
#             return
#         self.min_dist = min(self.ranges)
            
            
#     def control_loop(self):
#         cmd = Twist()
#         if self.min_dist < self.safe_distance:
#             self.get_logger().info(f'Obstacle detected at {self.min_dist:.2f} m - turning')
#             cmd.angular.z = 0.5
#         else:
#             cmd.linear.x = 0.2

#         self.pub.publish(cmd)
        
# def main(args=None):
#     rclpy.init(args=args)
#     node = ObstacleAvoider()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if rclpy.ok():
#             rclpy.shutdown()
#         node.destroy_node()

# if __name__=='__main__':
#     main()
            

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from rclpy import qos
import transforms3d
import math

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        # Suscriptores
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.lidar_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_cb, qos.qos_profile_sensor_data)
        
        # Publicador
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Variables de estado
        self.ranges = None
        self.angle_increment = 0.0
        self.angle_min = 0.0
        self.safe_distance = 0.4  # metros
        self.current_pose = None
        self.min_dist = float('inf')
        self.frontal_range = 30  # grados a cada lado (60° total de visión frontal)
        
        # Máquina de estados
        self.state = "FORWARD"  # Estados: FORWARD, BACKING, TURNING
        self.turn_direction = 1  # 1 para derecha, -1 para izquierda (alterna)
        self.backup_distance = 0.1  # metros a retroceder
        self.turn_angle = math.pi/2  # 90 grados en radianes
        
        # Variables para control de movimiento
        self.start_backup_pose = None
        self.start_turn_pose = None
        self.distance_backed = 0.0
        self.angle_turned = 0.0
        
        # Temporizador
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
    def odom_cb(self, msg):
        # Guardamos la pose actual
        prev_pose = self.current_pose
        self.current_pose = msg.pose.pose
        
        # Calculamos distancia recorrida desde el inicio del retroceso
        if self.state == "BACKING" and self.start_backup_pose:
            dx = self.current_pose.position.x - self.start_backup_pose.position.x
            dy = self.current_pose.position.y - self.start_backup_pose.position.y
            self.distance_backed = math.sqrt(dx**2 + dy**2)
        
        # Calculamos ángulo girado desde el inicio del giro
        if self.state == "TURNING" and self.start_turn_pose and prev_pose:
            # Obtenemos la orientación actual y la inicial
            current_quat = [
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            ]
            start_quat = [
                self.start_turn_pose.orientation.x,
                self.start_turn_pose.orientation.y,
                self.start_turn_pose.orientation.z,
                self.start_turn_pose.orientation.w
            ]
            
            # Calculamos la diferencia de ángulo
            _, _, current_yaw = transforms3d.euler.quat2euler(current_quat)
            _, _, start_yaw = transforms3d.euler.quat2euler(start_quat)
            self.angle_turned = abs(self.wrap_to_Pi(current_yaw - start_yaw))
        
    def lidar_cb(self, msg):
        self.ranges = msg.ranges
        self.angle_increment = msg.angle_increment
        self.angle_min = msg.angle_min
        
        # Calcular distancia mínima en el rango frontal
        if self.ranges:
            num_ranges = len(self.ranges)
            self.min_dist = float('inf')
            
            # Convertir ángulos a índices
            center_idx = num_ranges // 2
            range_start = center_idx - int(self.frontal_range * math.pi / 180 / self.angle_increment)
            range_end = center_idx + int(self.frontal_range * math.pi / 180 / self.angle_increment)
            
            # Asegurarse de que los índices estén dentro del rango
            range_start = max(0, range_start)
            range_end = min(num_ranges - 1, range_end)
            
            # Buscar la distancia mínima en el rango frontal
            for i in range(range_start, range_end):
                if 0 < self.ranges[i] < self.min_dist:
                    self.min_dist = self.ranges[i]
            
            # Si no hay obstáculos en el rango frontal, resetear min_dist
            if self.min_dist == float('inf'):
                self.min_dist = None
    
    def is_obstacle_in_front(self):
        """Verifica si hay un obstáculo en el rango frontal"""
        return self.min_dist is not None and self.min_dist < self.safe_distance
    
    def control_loop(self):
        cmd = Twist()
        
        # Máquina de estados
        if self.state == "FORWARD":
            if self.is_obstacle_in_front():
                self.get_logger().info(f'Obstáculo detectado a {self.min_dist:.2f} m - retrocediendo')
                self.state = "BACKING"
                self.start_backup_pose = self.current_pose
                self.distance_backed = 0.0
                # Alternar dirección de giro para el próximo giro
                self.turn_direction *= -1
            else:
                # Avanzar normalmente
                cmd.linear.x = 0.2
                cmd.angular.z = 0.0
        
        elif self.state == "BACKING":
            # Retroceder hasta completar la distancia
            if self.distance_backed < self.backup_distance:
                cmd.linear.x = -0.1  # Velocidad negativa para retroceder
                cmd.angular.z = 0.0
            else:
                self.get_logger().info('Retroceso completado - comenzando giro')
                self.state = "TURNING"
                self.start_turn_pose = self.current_pose
                self.angle_turned = 0.0
        
        elif self.state == "TURNING":
            # Girar hasta completar 90 grados
            if self.angle_turned < self.turn_angle:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5 * self.turn_direction  # Girar en la dirección determinada
            else:
                self.get_logger().info('Giro completado - volviendo a avanzar')
                self.state = "FORWARD"
        
        # Publicar el comando
        self.pub_cmd.publish(cmd)
    
    def wrap_to_Pi(self, theta):
        """Ajustar ángulo al rango [-π, π]"""
        result = np.fmod((theta + np.pi), (2 * np.pi))
        if result < 0:
            result += 2 * np.pi
        return result - np.pi

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()