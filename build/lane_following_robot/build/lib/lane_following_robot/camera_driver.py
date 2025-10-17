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

        self.get_logger().info("🚗 Enhanced Synthetic Lane Image Generator Initialized")

    def timer_callback(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)

        # Simulate better-structured lanes
        lane_color = (0, 255, 255)  # Bright yellow
        thickness = 15
        offset = random.randint(-10, 10)  # small lateral jitter

        left_lane_start = (200 + offset, 480)
        left_lane_end = (300 + offset, 0)

        right_lane_start = (440 + offset, 480)
        right_lane_end = (340 + offset, 0)

        cv2.line(image, left_lane_start, left_lane_end, lane_color, thickness)
        cv2.line(image, right_lane_start, right_lane_end, lane_color, thickness)

        # Optional: Add central lane
        # cv2.line(image, (320 + offset, 480), (320 + offset, 0), (255, 255, 0), 10)

        # Add low noise only
        noise = np.random.normal(0, 0.5, image.shape).astype(np.int16)
        image = cv2.add(image, noise, dtype=cv2.CV_8U)

        try:
            cv2.imshow("Camera Driver Output", image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"Error displaying Camera Driver Output: {str(e)}")

        try:
            ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.publisher.publish(ros_image)
            self.get_logger().info("Publishing enhanced image", throttle_duration_sec=0.5)
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
