import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class GreenBallTrackerNode(Node):
    def __init__(self):
        super().__init__('green_ball_tracker_node')
        self.get_logger().info('Green Ball Tracker Node Started')
        #Parámetros
        self.publisher = self.create_publisher(Point, 'ball_pos', 10)
        self.camera_sub = self.create_subscription(Image,'/video_source/raw',self.image_callback,10)
        self.bridge = CvBridge() 

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        lower_green = np.array([40, 70, 70])
        upper_green = np.array([80, 255, 255])
        mask = cv.inRange(hsv, lower_green, upper_green)
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=2)

        contours, _ = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv.contourArea)
            ((x, y), radius) = cv.minEnclosingCircle(c)
            msg = Point()
            msg.x = float(x)
            msg.y = float(y)
            msg.z = float(radius)
            self.publisher.publish(msg)
            self.get_logger().info(f"Ball at: ({msg.x:.1f}, {msg.y:.1f}), r={msg.z:.1f}")

    def destroy_node(self):
        self.cap.release()
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
