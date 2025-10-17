import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class LaneController(Node):
    def __init__(self):
        super().__init__('lane_controller')

        self.error_sub = self.create_subscription(
            Float32,
            '/lane_detection_error',
            self.error_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.current_error = 0.0
        self.max_steering = 1.0
        self.max_speed = 0.25
        self.kp = 0.01  # Proportional gain

    def error_callback(self, msg):
        self.current_error = msg.data
        self.get_logger().info(f'Received lane error: {self.current_error:.3f}')

    def control_loop(self):
        twist = Twist()

        # Compute normalized error
        norm_error = self.current_error / 100.0  # scale error to [-1, 1]
        norm_error = max(min(norm_error, 1.0), -1.0)

        # Compute steering based on proportional control
        steering = self.kp * self.current_error
        steering = max(min(steering, self.max_steering), -self.max_steering)

        # Adjust speed (slower when error is high)
        speed = self.max_speed * (1 - abs(norm_error))
        speed = max(speed, 0.1)  # avoid stopping

        twist.linear.x = speed
        twist.angular.z = steering

        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f'Tracking | Error: {self.current_error:.1f} | Normalized: {norm_error:.2f} | '
            f'Speed: {speed:.2f} | Steering: {steering:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = LaneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()