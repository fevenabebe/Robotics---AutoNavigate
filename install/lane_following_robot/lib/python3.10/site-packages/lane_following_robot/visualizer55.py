import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer')
        self.subscription = self.create_subscription(
            Image,
            '/processed_image',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # Create named window here to ensure it exists before imshow is called
        try:
            cv2.namedWindow("Processed Image", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Processed Image", 800, 600) # Optional: resize for better viewing
        except Exception as e:
            self.get_logger().warn(f"Failed to create display window: {e}. Ensure you have a graphical environment.")
        
        self.get_logger().info("📊 Visualizer Initialized")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Processed Image", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            # Catch exceptions during imshow, which can happen if window isn't created or display is lost
            self.get_logger().error(f"Error displaying image in Visualizer: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # Ensure windows are destroyed even if an error occurs or on shutdown
        cv2.destroyAllWindows() 

if __name__ == '__main__':
    main()