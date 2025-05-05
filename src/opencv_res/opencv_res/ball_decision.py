import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class GreenBallTrackerNode(Node):
    def __init__(self):
        super().__init__('color_tracker_node')
        self.get_logger().info('Color Tracker Node Started')

        self.state_pub = self.create_publisher(Float32, 'state', 10)
        self.camera_sub = self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)
        self.bridge = CvBridge()

        self.colors = {
            'green': (np.array([40, 70, 70]), np.array([80, 255, 255])),
            # 'blue': (np.array([100, 150, 0]), np.array([140, 255, 255])),
            # 'orange': (np.array([5, 150, 150]), np.array([15, 255, 255])),
            'yellow': (np.array([20, 100, 100]), np.array([40, 255, 255])),
            'red': (np.array([0, 100, 100]), np.array([10, 255, 255]))
        }

        self.state_map = {'green': 0, 'yellow': 1, 'red': 2}
        self.state = 0

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

            for color_name, (lower, upper) in self.colors.items():
                mask = cv.inRange(hsv, lower, upper)
                mask = cv.erode(mask, None, iterations=2)
                mask = cv.dilate(mask, None, iterations=2)

                contours, _ = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
                if contours:
                    self.state = self.state_map[color_name]
                    self.state_pub.publish(Float32(data=float(self.state)))
                    self.get_logger().info(f"Detected {color_name}, published state {self.state}")
                    break  # Stop at first detected color

        except Exception as e:
            self.get_logger().error(f"{e}")

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GreenBallTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
