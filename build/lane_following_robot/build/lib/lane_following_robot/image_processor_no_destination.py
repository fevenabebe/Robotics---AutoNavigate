import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class DualLaneDetector(Node):
    def __init__(self):
        super().__init__('dual_lane_detector')

        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/processed_image', 10)
        self.mask_pub = self.create_publisher(Image, '/debug/mask', 10)
        self.error_pub = self.create_publisher(Float32, '/lane_error', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()

        # HSV thresholds for green lane detection
        self.lower_green = np.array([45, 100, 100])
        self.upper_green = np.array([75, 255, 255])

        self.min_contour_area = 100
        self.max_lane_width = 200
        self.frame_counter = 0

        cv2.namedWindow("Dual Lane Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Dual Lane Detection", 800, 600)

        self.get_logger().info("🛣️ Dual Lane Detector Initialized (green lane support)")

    def image_callback(self, msg):
        start_time = time.time()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Optional ROI: Only process bottom half
            hsv_image[:hsv_image.shape[0] // 2, :] = 0

            mask = cv2.inRange(hsv_image, self.lower_green, self.upper_green)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            filtered = []
            for contour in contours:
                area = cv2.contourArea(contour)
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h if h != 0 else 0

                if self.min_contour_area < area and w < self.max_lane_width and 0.2 < aspect_ratio < 5.0:
                    filtered.append((area, contour))

            filtered.sort(key=lambda x: x[0], reverse=True)
            lane_contours = [cnt for _, cnt in filtered[:2]]

            output_img = cv_image.copy()
            current_error = 0.0

            if len(lane_contours) == 2:
                M1 = cv2.moments(lane_contours[0])
                M2 = cv2.moments(lane_contours[1])

                if M1["m00"] != 0 and M2["m00"] != 0:
                    cx1 = int(M1["m10"] / M1["m00"])
                    cx2 = int(M2["m10"] / M2["m00"])

                    cv2.drawContours(output_img, lane_contours, -1, (0, 255, 0), 2)
                    cv2.circle(output_img, (cx1, int(M1["m01"] / M1["m00"])), 5, (0, 0, 255), -1)
                    cv2.circle(output_img, (cx2, int(M2["m01"] / M2["m00"])), 5, (0, 0, 255), -1)

                    lane_center_x = (cx1 + cx2) / 2
                    image_center_x = cv_image.shape[1] / 2
                    current_error = lane_center_x - image_center_x

            elif len(lane_contours) == 1:
                M = cv2.moments(lane_contours[0])
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.drawContours(output_img, [lane_contours[0]], -1, (0, 255, 255), 2)
                    cv2.circle(output_img, (cx, cy), 5, (0, 0, 255), -1)

                    image_center_x = cv_image.shape[1] / 2
                    current_error = cx - image_center_x

            else:
                self.get_logger().warn("Detected 0 or >2 lanes.", throttle_duration_sec=0.5)
                current_error = 0.0

            # Annotate image
            cv2.putText(output_img, f"Error: {current_error:.3f}", (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(output_img, f"Lanes: {len(lane_contours)}/2", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            if self.frame_counter % 5 == 0:
                cv2.imshow("Dual Lane Detection", output_img)
                cv2.waitKey(1)
            self.frame_counter += 1

            self.publisher.publish(self.bridge.cv2_to_imgmsg(output_img, "bgr8"))
            self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))

            error_msg = Float32()
            error_msg.data = current_error
            self.error_pub.publish(error_msg)

            # Control the robot
            self.send_velocity_command(current_error)

            end_time = time.time()
            self.get_logger().info(
                f"Detected {len(lane_contours)} lanes | Error: {current_error:.3f} | Time: {(end_time - start_time) * 1000:.2f}ms",
                throttle_duration_sec=1.0)

        except Exception as e:
            end_time = time.time()
            self.get_logger().error(f"Detection error: {str(e)} | Time: {(end_time - start_time) * 1000:.2f}ms")

    def send_velocity_command(self, error):
        twist = Twist()
        kp = 0.005  # Proportional gain (tune if needed)
        twist.linear.x = 0.15  # Constant forward speed
        twist.angular.z = -kp * error  # Negative because error is right-positive
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DualLaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()