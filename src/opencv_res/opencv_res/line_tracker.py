import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import opencv_res.filterss as fl

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        self.get_logger().info('Line Follower Node Started')
        #Parámetros
        self.publisher = self.create_publisher(Point, 'line_offset', 10)
        self.camera_sub = self.create_subscription(Image,'/video_source/raw',self.image_cb,10)
        self.bridge = CvBridge()
        self.frame = None
        self.refined_img = None
        self.target_width, self.target_height = 640, 480
        
        # self.timer_period = 0.1  # 10 Hz
        # self.timer = self.create_timer(self.timer_period, self.main)

    def image_cb(self,msg):
        #  Extract image
        self.frame = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        #Resize if needed
        height, width = self.frame.shape[:2]
        if (width,height) != (self.target_width,self.target_height):
            self.frame = cv.resize(self.frame,(self.target_width,self.target_height))
            self.get_logger().info("Image rezied to 640x480")
        pos = Point()
        #pos.x,pos.y = fl.line_detection_moments(fl.sharpened_filter(self.frame))
        #pos.x,pos.y = float(pos.x), float(pos.y)
        pos.x, pos.y = map(float, fl.line_detection_moments(fl.sharpened_filter(self.frame)))
        self.get_logger().info(f'Publishing offset: ({pos.x:.2f}, {pos.y:.2f})')
        pos.z = 0.0
        self.publisher.publish(pos)

        
    def destroy_node(self):
        #self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
