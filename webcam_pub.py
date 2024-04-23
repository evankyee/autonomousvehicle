import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw/uncompressed', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Load an image from file
        print("Current Working Directory:", os.getcwd())
        self.image_path = 'image.png'  # Replace with the path to your image
        self.image = cv2.imread(self.image_path)
        self.br = CvBridge()

    def timer_callback(self):
        if self.image is not None:
            # Publish the image
            self.publisher_.publish(self.br.cv2_to_imgmsg(self.image)) # , "bgr8" and do the same on subscriber expected format?
            self.get_logger().info('Publishing image from file')
        else:
            self.get_logger().error('Failed to read image from file')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

