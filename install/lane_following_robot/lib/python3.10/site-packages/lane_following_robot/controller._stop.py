import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class LaneController(Node):
    def __init__(self):
        super().__init__('lane_controller')

        self.error_sub = self.create_subscription(Float32, '/lane_error', self.error_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.current_error = 0.0
        self.max_steering = 1.0
        self.max_speed = 0.25
        self.kp = 0.01

        self.robot_position = (0.0, 0.0)
        self.reached_destination = False
        self.rotated = False
        self.returning = False
        self.finished = False

        self.pause_start_time = None
        self.start_position = (0.0, 0.0)

    def error_callback(self, msg):
        self.current_error = msg.data

    def odom_callback(self, msg):
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def rotate_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.6  # Rotate in place
        self.cmd_pub.publish(twist)

    def control_loop(self):
        if self.finished:
            self.stop_robot()
            return

        # Step 1: Stop and wait for 1 minute
        if self.reached_destination and not self.rotated:
            if self.pause_start_time is None:
                self.get_logger().info("🛑 Destination reached. Pausing for 1 minute...")
                self.pause_start_time = time.time()
                self.stop_robot()
                return

            elapsed = time.time() - self.pause_start_time
            if elapsed < 60.0:
                self.stop_robot()
                return
            else:
                self.get_logger().info("🔄 Rotating 180 degrees...")
                self.rotate_start_time = time.time()
                self.rotated = True
                return

        # Step 2: Rotate for ~3 seconds
        if self.rotated and not self.returning:
            elapsed = time.time() - self.rotate_start_time
            if elapsed < 3.0:
                self.rotate_robot()
                return
            else:
                self.get_logger().info("🔙 Starting return to original position...")
                self.returning = True

        # Step 3: Stop at destination signal
        if self.current_error == -999.0 and not self.reached_destination:
            self.get_logger().info("🟥 Red box stop signal received.")
            self.reached_destination = True
            self.stop_robot()
            return

        # Step 4: Stop when close to starting point
        if self.returning:
            x, y = self.robot_position
            if abs(x - self.start_position[0]) < 0.3 and abs(y - self.start_position[1]) < 0.3:
                self.get_logger().info("✅ Returned to starting point. Stopping.")
                self.finished = True
                self.stop_robot()
                return

        # Normal lane following
        norm_error = self.current_error / 100.0
        norm_error = max(min(norm_error, 1.0), -1.0)

        steering = self.kp * self.current_error
        steering = max(min(steering, self.max_steering), -self.max_steering)

        speed = self.max_speed * (1 - abs(norm_error))
        speed = max(speed, 0.1)

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = steering

        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f"🚗 Driving | x = {self.robot_position[0]:.2f} | Error = {self.current_error:.1f} | Speed = {speed:.2f} | Steering = {steering:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = LaneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
