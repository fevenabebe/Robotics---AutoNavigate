import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import random

class CameraDriver(Node):
    def __init__(self):
        super().__init__('camera_driver')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()

        try:
            cv2.namedWindow("Camera Driver Output", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Camera Driver Output", 640, 480)
        except Exception as e:
            self.get_logger().warn(f"Failed to create Camera Driver Output window: {e}. Running without GUI.")

        self.get_logger().info("Synthetic lane image generator Initialized")

    def timer_callback(self):
        image_to_publish = np.zeros((480, 640, 3), dtype=np.uint8)

        points = [
            ((150, 450), (300, 300)),
            ((450, 300), (550, 450)),
            ((300, 300), (450, 300))
        ]

        current_offset = 0
        if random.random() > 0.5:
            current_offset = random.randint(-15, 15)

        for p1, p2 in points:
            p1_offset = (p1[0] + current_offset, p1[1])
            p2_offset = (p2[0] + current_offset, p2[1])
            cv2.line(image_to_publish, p1_offset, p2_offset, (0, 255, 255), 20)

        # IMPORTANT: Ensure noise standard deviation is low for testing consistent detection
        noise = np.random.normal(0, 1, image_to_publish.shape).astype(np.int16) # Set std_dev to 1
        image_to_publish = cv2.add(image_to_publish, noise, dtype=cv2.CV_8U)

        try:
            cv2.imshow("Camera Driver Output", image_to_publish)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"Error displaying Camera Driver Output: {str(e)}")

        try:
            ros_image = self.bridge.cv2_to_imgmsg(image_to_publish, "bgr8")
            self.publisher.publish(ros_image)
            self.get_logger().info("Publishing image", throttle_duration_sec=0.5)
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Camera Driver stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        
if __name__ == '__main__':
    main()