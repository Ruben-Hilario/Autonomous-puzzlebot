import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import yaml
import cv2
import os

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher_node')

        # Carga el YAML
        with open('map.yaml', 'r') as f:
            map_config = yaml.safe_load(f)

        self.frame_id = "map"
        resolution = map_config['resolution']
        origin = map_config['origin']
        image_path = map_config['image']
        image_path = os.path.join(os.path.dirname(__file__), image_path)

        # Cargar imagen como mapa de ocupación
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f"No se pudo leer la imagen {image_path}")
            exit(1)

        height, width = img.shape
        data = []
        ancho_metros = width * resolution
        alto_metros = height * resolution
        self.get_logger().info(f"Tamaño del mapa: {ancho_metros:.2f} m de ancho x {alto_metros:.2f} m de alto")

        for y in range(height):
            for x in range(width):
                value = img[y, x]
                if value == 205:   # color gris intermedio (valor desconocido)
                    data.append(-1)
                elif value > 250:  # blanco = libre
                    data.append(0)
                else:              # negro = ocupado
                    data.append(100)

        # Crear OccupancyGrid
        self.map_msg = OccupancyGrid()
        self.map_msg.header = Header()
        self.map_msg.header.frame_id = self.frame_id
        self.map_msg.info.resolution = resolution
        self.map_msg.info.width = width
        self.map_msg.info.height = height
        self.map_msg.info.origin.position.x = origin[0]
        self.map_msg.info.origin.position.y = origin[1]
        self.map_msg.info.origin.position.z = origin[2]
        self.map_msg.info.origin.orientation.w = 1.0
        self.map_msg.data = data

        self.publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info("Publicando mapa en /map...")

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.map_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
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
