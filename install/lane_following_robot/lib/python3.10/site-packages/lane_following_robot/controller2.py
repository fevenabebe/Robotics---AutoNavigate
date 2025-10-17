import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np

class LaneController(Node):
    def __init__(self):
        super().__init__('lane_controller')

        self.error_sub = self.create_subscription(Float32, '/lane_error', self.error_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.current_error = 0.0
        self.current_pose = None
        self.reached_destination = False
        self.destination_pose = (25.0, 0.0)

        self.max_steering = 1.0
        self.max_speed = 0.25
        self.kp = 0.01

    def error_callback(self, msg):
        # Clamp very large error values to avoid instability
        self.current_error = np.clip(msg.data, -500.0, 500.0)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.current_pose = (position.x, position.y)

        dest_x, dest_y = self.destination_pose
        distance = np.sqrt((position.x - dest_x) ** 2 + (position.y - dest_y) ** 2)

        if distance < 0.1 and not self.reached_destination:  # Tighter threshold
            self.get_logger().info(f"✅ Reached destination at ({position.x:.2f}, {position.y:.2f})")
            self.reached_destination = True

    def control_loop(self):
        twist = Twist()

        if self.reached_destination:
            # Stop all motion
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        # Normalize and clamp error for stable control
        norm_error = np.clip(self.current_error / 100.0, -1.0, 1.0)

        # PID-like control (currently P only)
        steering = self.kp * self.current_error
        steering = np.clip(steering, -self.max_steering, self.max_steering)

        # Adjust speed based on steering error
        speed = self.max_speed * (1.0 - abs(norm_error))
        speed = max(speed, 0.08)  # Avoid full stop unless commanded

        twist.linear.x = speed
        twist.angular.z = steering

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LaneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
