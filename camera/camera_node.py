import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.image_path = '/root/ros2_ws/src/my_cobot_pro/dataset/black line detection.v2i.yolov8-obb/test/images/rotated_rect_043_png.rf.81854ff1ff965736fcb46f26ba54f74d.jpg'  # Replace with your image path

    def timer_callback(self):
        cv_image = cv2.imread(self.image_path)
        msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('Published image.')

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
