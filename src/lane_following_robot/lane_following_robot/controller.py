import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from enum import Enum
import time

class RobotState(Enum):
    FOLLOWING_LANE = 1
    PAUSED_AT_DESTINATION = 2
    ROTATING = 3
    RETURNING = 4
    FINISHED = 5

class LaneController(Node):
    def __init__(self):
        super().__init__('lane_controller')

        self.error_sub = self.create_subscription(Float32, '/lane_error', self.error_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.current_error = 0.0
        self.max_speed = 0.25
        self.kp = 0.005
        self.kd = 0.0005
        self.integral = 0.0
        self.prev_error = 0.0

        self.state = RobotState.FOLLOWING_LANE
        self.robot_position = (0.0, 0.0)
        self.start_position = (0.0, 0.0)
        self.has_set_start = False
        self.max_distance_reached = 0.0
        self.pause_start_time = None
        self.rotate_start_time = None
        self.start_time = time.time()

    def error_callback(self, msg):
        self.current_error = msg.data

    def odom_callback(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.robot_position = (x, y)
        if not self.has_set_start:
            self.start_position = (x, y)
            self.has_set_start = True
            self.get_logger().info(f"📍 Start position set to ({x:.2f}, {y:.2f})")

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        for _ in range(3):
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
        self.get_logger().info("🛑 Robot STOPPED")

    def rotate_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.6
        self.cmd_pub.publish(twist)

    def control_loop(self):
        if time.time() - self.start_time < 2.0:
            self.get_logger().info("⏳ Waiting for camera to stabilize...")
            return

        self.get_logger().info(f"[ControlLoop] State={self.state.name} | Error={self.current_error:.2f}")

        if self.state == RobotState.FINISHED:
            self.stop_robot()
            self.get_logger().info("🏁 Robot has returned. All motion stopped.")
            self.timer.cancel()
            return

        # Handle returning to start position
        if self.state == RobotState.RETURNING:
            x, y = self.robot_position
            sx, sy = self.start_position
            dist = ((x - sx)**2 + (y - sy)**2)**0.5
            self.max_distance_reached = max(self.max_distance_reached, dist)

            self.get_logger().info(f"[RETURNING] Pos=({x:.2f},{y:.2f}) | Start=({sx:.2f},{sy:.2f}) | Dist={dist:.2f}")

            # ✅ Stop permanently when near starting point
            if self.max_distance_reached > 1.0 and dist < 0.3:
                self.get_logger().info("✅ Returned to start → FINISHED")
                self.state = RobotState.FINISHED
                self.stop_robot()
                return

            # PID control during RETURNING
            error = -self.current_error
            self.integral += error
            derivative = error - self.prev_error
            self.prev_error = error

            steering = self.kp * error + self.kd * derivative
            steering = max(min(steering, 0.15), -0.15)
            speed = max(self.max_speed * (1 - abs(error / 100.0)), 0.1)

            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = steering
            self.cmd_pub.publish(twist)
            return

        # Handle red box logic
        if abs(self.current_error + 999.0) < 1.0:
            if self.state == RobotState.FOLLOWING_LANE:
                self.state = RobotState.PAUSED_AT_DESTINATION
                self.pause_start_time = time.time()
                self.stop_robot()
                self.get_logger().info("🟥 Red box detected → Paused")
                return
            elif self.state == RobotState.PAUSED_AT_DESTINATION:
                if time.time() - self.pause_start_time < 60:
                    self.stop_robot()
                    self.get_logger().info("⏸️ Waiting at destination...")
                    return
                else:
                    self.state = RobotState.ROTATING
                    self.rotate_start_time = time.time()
                    self.get_logger().info("⏩ Done waiting. Start rotating.")
                    return

        # Handle ROTATING state
        if self.state == RobotState.ROTATING:
            if time.time() - self.rotate_start_time < 6.0:
                self.rotate_robot()
                self.get_logger().info("🔁 Rotating...")
                return
            else:
                self.get_logger().info("🔙 Rotation complete → RETURNING")
                self.state = RobotState.RETURNING
                return

        # FOLLOWING_LANE PID control
        error = -self.current_error
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error

        steering = self.kp * error + self.kd * derivative
        steering = max(min(steering, 0.15), -0.15)
        speed = max(self.max_speed * (1 - abs(error / 100.0)), 0.1)

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = steering
        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f"🚗 FOLLOWING: Speed={speed:.2f}, Steering={steering:.2f}, Error={error:.2f}, Pos=({self.robot_position[0]:.2f},{self.robot_position[1]:.2f})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = LaneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
