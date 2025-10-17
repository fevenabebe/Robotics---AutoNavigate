import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32 # Import Float32 to receive lane error
from geometry_msgs.msg import Twist
import cv2 # Still needed for potential GUI/debug, but not for image processing
import numpy as np

class LaneController(Node):
    def __init__(self):
        super().__init__('lane_controller')

        # Subscribe to the numerical lane error from image_processor
        self.error_subscription = self.create_subscription(Float32, '/lane_error', self.error_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # We can still optionally subscribe to /processed_image for display only
        # self.image_subscription = self.create_subscription(Image, '/processed_image', self.image_display_callback, 10)
        # from cv_bridge import CvBridge # Uncomment if using image_display_callback
        # self.bridge = CvBridge() # Uncomment if using image_display_callback

        self.Kp = 0.035
        self.Ki = 0.0001
        self.Kd = 0.02
        self.base_speed = 0.25

        self.prev_error = 0.0
        self.integral = 0.0
        self.lost_count = 0
        self.search_direction = 1 # 1 for right, -1 for left
        self.lane_detected_flag = False # Flag to indicate if a valid error was received

        try:
            cv2.namedWindow("Controller View", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Controller View", 800, 600)
        except Exception as e:
            self.get_logger().warn(f"Failed to create Controller View window: {e}. Running without GUI.")

        self.get_logger().info("🚦 Controller initialized")

    # New callback to receive the numerical error
    def error_callback(self, msg):
        current_error = msg.data
        self.lane_detected_flag = True # A valid error means lanes were detected

        if current_error is not None: # Check if a meaningful error was received
            self.lost_count = 0 # Reset lost count if error is valid

            self.integral = np.clip(self.integral + current_error, -0.3, 0.3)
            derivative = current_error - self.prev_error

            steering = self.Kp * current_error + self.Ki * self.integral + self.Kd * derivative
            self.prev_error = current_error

            speed = self.base_speed * (1 - 0.4 * abs(steering))

            twist = Twist()
            twist.linear.x = float(speed)
            twist.angular.z = float(-steering * 1.3) # Scaling factor for angular velocity
            self.cmd_pub.publish(twist)

            self.get_logger().info(
                f"Tracking | Error: {current_error:.3f} | Speed: {speed:.2f}",
                throttle_duration_sec=0.5)
            
            # Update Controller View for debugging
            self.update_controller_view_gui(current_error, speed, steering)

        else:
            self.lane_detected_flag = False # No valid error, treat as lane lost
            self.handle_lost_lane()


    # Optional: Callback to display the processed image received (without re-processing)
    # Uncomment if you want to see the image in Controller View, but it's not needed for control
    # def image_display_callback(self, msg):
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #         # Only display, do not process for lanes
    #         cv2.imshow("Controller View", cv_image)
    #         cv2.waitKey(1)
    #     except Exception as e:
    #         self.get_logger().warn(f"Error displaying image in Controller View: {str(e)}")


    def update_controller_view_gui(self, error, speed, steering):
        # Create a blank image to draw current status, as we no longer process images here
        display_img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        cv2.putText(display_img, f"Speed: {speed:.2f}m/s", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(display_img, f"Steering: {steering:.3f}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(display_img, f"Lane Error: {error:.3f}", (10, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # Visualize error: draw a line or circle indicating steering direction
        center_x = 320 # Center of 640 width image
        target_indicator_x = int(center_x + error * (center_x * 0.8)) # Scale error to fit on image
        cv2.line(display_img, (center_x, 400), (target_indicator_x, 300), (0, 255, 255), 5)
        cv2.circle(display_img, (target_indicator_x, 300), 10, (255, 0, 0), -1)

        try:
            cv2.imshow("Controller View", display_img)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"Error displaying 'Controller View' image: {str(e)}")


    def handle_lost_lane(self):
        self.lost_count += 1
        self.integral = 0.0

        # Update GUI for lost lane
        display_img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(display_img, "LANE LOST!", (display_img.shape[1]//4, display_img.shape[0]//2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        try:
            cv2.imshow("Controller View", display_img)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"Error displaying 'Controller View' image: {str(e)}")

        if self.lost_count > 30:
            self.stop_robot()
            self.get_logger().error("Robot stopped due to prolonged lane loss.")
            return

        twist = Twist()
        twist.linear.x = 0.12
        twist.angular.z = 0.4 * self.search_direction
        self.cmd_pub.publish(twist)

        if self.lost_count % 15 == 0:
            self.search_direction *= -1

        self.get_logger().warn(
            f"Searching ({self.lost_count}/30) | Direction: {'Right' if self.search_direction == 1 else 'Left'}",
            throttle_duration_sec=0.3)

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    controller = LaneController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.stop_robot()
        controller.get_logger().info("Controller stopped by user.")
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()