import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
from algorithm import astar
from collections import deque
import cv2

class SimplePlanner(Node):
    def __init__(self):
        super().__init__('simple_path_planner')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.map = None
    from collections import deque

    def find_nearest_free_cell(self,data, start):
        """
        Busca la celda libre más cercana a `start` en una matriz de ocupación `data`.

        Parámetros:
        - data: numpy.ndarray (valores de ocupación, -1, 0, 100)
        - start: tupla (fila, columna)

        Retorna:
        - tupla (fila, columna) de una celda libre más cercana, o None si no hay ninguna
        """
        height, width = data.shape
        visited = set()
        queue = deque()
        queue.append(start)

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                    (-1, -1), (-1, 1), (1, -1), (1, 1)]  # 8 direcciones

        while queue:
            current = queue.popleft()
            y, x = current

            if (0 <= y < height and 0 <= x < width and current not in visited):
                visited.add(current)
                if data[y, x] == 0:  # celda libre
                    return current
                for dy, dx in directions:
                    queue.append((y + dy, x + dx))

        return None  # No se encontró ninguna celda libre

    def map_callback(self, msg):
        self.map = msg
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))
        # Inflar obstáculos para mantener distancia mínima (10 celdas)
        obstacles = (data == 100).astype(np.uint8)  # mapa binario de obstáculos
        kernel_size = 2 * 6 + 1  # tamaño del kernel para inflar 10 celdas alrededor
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        inflated_obstacles = cv2.dilate(obstacles, kernel)

        # Marcar las celdas infladas como ocupadas en data
        data[inflated_obstacles == 1] = 100

        resolution = msg.info.resolution
        #data[data == -1] = 0
        origin = msg.info.origin
        # Define start and goal in grid coords
        start = (35, 50)
        goal = (110, 100)

        if data[start[0], start[1]] != 0:
            self.get_logger().warn('Start cell is not free! Buscando celda libre cercana...')
            new_start = self.find_nearest_free_cell(data, start)
            if new_start:
                self.get_logger().info(f'Nuevo start: {new_start}')
                start = new_start
            else:
                self.get_logger().error('No se encontró celda libre cercana al punto de inicio')
                return

        if data[goal[0], goal[1]] != 0:
            self.get_logger().warn('Goal cell is not free! Buscando celda libre cercana...')
            new_goal = self.find_nearest_free_cell(data, goal)
            if new_goal:
                self.get_logger().info(f'Nuevo goal: {new_goal}')
                goal = new_goal
            else:
                self.get_logger().error('No se encontró celda libre cercana al punto de destino')
                return


        path = astar(data, start, goal)
        print(f"Start cell value: {data[start[0], start[1]]}")
        print(f"Goal cell value: {data[goal[0], goal[1]]}")
        if path:
            ros_path = Path()
            ros_path.header = Header()
            ros_path.header.stamp = self.get_clock().now().to_msg()
            ros_path.header.frame_id = msg.header.frame_id
            # for x, y in path:
            #     pose = PoseStamped()
            #     pose.header = ros_path.header
            #     pose.pose.position.x = x * resolution + origin.position.x
            #     pose.pose.position.y = y * resolution + origin.position.y
            #     pose.pose.orientation.w = 1.0
            #     ros_path.poses.append(pose)
            for x, y in path:  # Recuerda: y = fila, x = columna
                if data[y, x] != 0:
                    corrected = self.find_nearest_free_cell(data, (y, x))
                    if corrected:
                        y, x = corrected
                        self.get_logger().warn(f'Celda ocupada en ruta, corrigiendo a ({y}, {x})')
                    else:
                        self.get_logger().error(f'No se pudo corregir celda en ({y}, {x})')
                        continue  # O puedes hacer return si prefieres abortar

                pose = PoseStamped()
                pose.header = ros_path.header
                pose.pose.position.x = x * resolution + origin.position.x
                pose.pose.position.y = y * resolution + origin.position.y
                pose.pose.orientation.w = 1.0
                ros_path.poses.append(pose)

            
            self.path_pub.publish(ros_path)
            self.get_logger().info(f'Path published with {len(path)} points.')
        else:
            self.get_logger().warn('No path found!')
    

def main(args=None):
    rclpy.init(args=args)
    node = SimplePlanner()
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
