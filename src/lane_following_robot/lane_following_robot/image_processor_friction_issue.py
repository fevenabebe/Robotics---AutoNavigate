import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class DualLaneDetector(Node):
    def __init__(self):
        super().__init__('dual_lane_detector')

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/processed_image', 10)
        self.mask_pub = self.create_publisher(Image, '/debug/mask', 10)
        self.error_pub = self.create_publisher(Float32, '/lane_error', 10)

        self.bridge = CvBridge()

        # HSV thresholds for green lane detection
        self.lower_green = np.array([45, 100, 100])
        self.upper_green = np.array([75, 255, 255])

        # HSV thresholds for red box detection
        self.lower_red1 = np.array([0, 120, 70])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 70])
        self.upper_red2 = np.array([180, 255, 255])

        self.min_contour_area = 100
        self.max_lane_width = 300  # Max lane width to detect solid lane
        self.frame_counter = 0

        self.prev_error = 0.0
        self.alpha = 0.3  # smoothing factor for exponential moving average

        cv2.namedWindow("Dual Lane Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Dual Lane Detection", 800, 600)

        self.get_logger().info("🛣️ Dual Lane Detector Initialized (green lane + red box detection)")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Mask top half of image (ignore irrelevant parts)
            hsv_image[:hsv_image.shape[0] // 2, :] = 0

            # Green lane mask with morphology to reduce noise
            green_mask = cv2.inRange(hsv_image, self.lower_green, self.upper_green)
            green_mask = cv2.erode(green_mask, None, iterations=2)
            green_mask = cv2.dilate(green_mask, None, iterations=2)

            # Red box mask for stop detection
            red_mask1 = cv2.inRange(hsv_image, self.lower_red1, self.upper_red1)
            red_mask2 = cv2.inRange(hsv_image, self.lower_red2, self.upper_red2)
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            red_mask = cv2.erode(red_mask, None, iterations=2)
            red_mask = cv2.dilate(red_mask, None, iterations=2)

            # Detect red box presence
            red_detected = False
            contours_red, _ = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours_red:
                area = cv2.contourArea(cnt)
                if area > 3000:
                    red_detected = True
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 3)
                    cv2.putText(cv_image, "STOP (Red Box)", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    break  # stop after first large red box found

            current_error = 0.0
            lane_count = 0

            if red_detected:
                current_error = -999.0
                self.get_logger().info("🟥 Red flag detected, publishing error -999.0")
            else:
                # Normal lane detection when no red flag
                contours, _ = cv2.findContours(green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                filtered = []
                for contour in contours:
                    area = cv2.contourArea(contour)
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = float(w) / h if h != 0 else 0
                    # Filter contours by size and shape
                    if self.min_contour_area < area < 10000 and w < self.max_lane_width and 0.2 < aspect_ratio < 5.0:
                        filtered.append((area, contour))

                filtered.sort(key=lambda x: x[0], reverse=True)
                lane_contours = [cnt for _, cnt in filtered[:2]]
                lane_count = len(lane_contours)

                if lane_count == 2:
                    M1 = cv2.moments(lane_contours[0])
                    M2 = cv2.moments(lane_contours[1])
                    if M1["m00"] != 0 and M2["m00"] != 0:
                        cx1 = int(M1["m10"] / M1["m00"])
                        cx2 = int(M2["m10"] / M2["m00"])
                        lane_center_x = (cx1 + cx2) / 2
                        image_center_x = cv_image.shape[1] / 2
                        current_error = lane_center_x - image_center_x
                        cv2.drawContours(cv_image, [lane_contours[0]], -1, (0, 255, 0), 2)
                        cv2.drawContours(cv_image, [lane_contours[1]], -1, (0, 255, 0), 2)
                        cv2.circle(cv_image, (cx1, int(M1["m01"] / M1["m00"])), 5, (0, 0, 255), -1)
                        cv2.circle(cv_image, (cx2, int(M2["m01"] / M2["m00"])), 5, (0, 0, 255), -1)

                elif lane_count == 1:
                    cnt = lane_contours[0]
                    x, y, w, h = cv2.boundingRect(cnt)
                    aspect_ratio = float(w) / h if h != 0 else 0

                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        image_center_x = cv_image.shape[1] / 2

                        if w > self.max_lane_width or aspect_ratio > 2.0:
                            self.get_logger().info("🟩 Solid lane detected.")
                            cv2.drawContours(cv_image, [cnt], -1, (255, 255, 0), 2)
                        else:
                            cv2.drawContours(cv_image, [cnt], -1, (0, 255, 255), 2)

                        cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                        current_error = cx - image_center_x

                else:
                    self.get_logger().warn("⚠️ No valid lanes detected.")
                    current_error = -999.0  # Send error to trigger recovery

            # Exponential moving average smoothing
            smoothed_error = self.alpha * current_error + (1 - self.alpha) * self.prev_error
            self.prev_error = smoothed_error

            # Publish error
            error_msg = Float32()
            error_msg.data = smoothed_error
            self.error_pub.publish(error_msg)

            # Overlay info on image
            cv2.putText(cv_image, f"Error: {smoothed_error:.1f}", (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(cv_image, f"Lanes: {lane_count}/2", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # Show image every 5 frames to reduce CPU load
            if self.frame_counter % 5 == 0:
                cv2.imshow("Dual Lane Detection", cv_image)
                cv2.waitKey(1)
            self.frame_counter += 1

            # Publish processed images and masks
            self.publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            self.mask_pub.publish(self.bridge.cv2_to_imgmsg(green_mask, "mono8"))

            # Throttled debug log
            self.get_logger().info(
                f"📏 Error = {smoothed_error:.2f} | Lanes = {lane_count} | Red Detected = {red_detected}",
                throttle_duration_sec=1.0
            )

        except Exception as e:
            self.get_logger().error(f"❌ Detection Error: {e}")

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

if __name__ == '__main__':
    main()
