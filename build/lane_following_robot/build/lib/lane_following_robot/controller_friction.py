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
        self.returned_and_red_flag = False

    def error_callback(self, msg):
        self.current_error = msg.data
        if self.current_error < -900.0:
            self.get_logger().info("🟥 Red flag error received (-999.0)")
        else:
            self.get_logger().info(f"[ErrorCallback] Lane error: {self.current_error:.2f}")

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
        for _ in range(3):  # force multiple stop signals
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
        self.get_logger().info("🛑 Robot issued full stop (3x zero twist)")

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

        # Final stop
        if self.state == RobotState.FINISHED:
            self.stop_robot()
            self.get_logger().info("🏁 Robot finished task. Cancelling timer loop.")
            self.timer.cancel()
            return

        # Red flag during RETURNING triggers final FINISHED state
        if self.state == RobotState.RETURNING:
            x, y = self.robot_position
            sx, sy = self.start_position
            dist = ((x - sx)**2 + (y - sy)**2)**0.5
            self.max_distance_reached = max(self.max_distance_reached, dist)

            self.get_logger().info(f"[RETURNING] Pos=({x:.2f},{y:.2f}) | Start=({sx:.2f},{sy:.2f}) | Dist={dist:.2f}")

            if self.max_distance_reached > 1.0 and dist < 0.3 and self.current_error < -900.0:
                self.get_logger().info("✅ Returned to start AND saw red flag → FINISHED")
                self.state = RobotState.FINISHED
                self.stop_robot()
                return

        # Red box logic
        if abs(self.current_error + 999.0) < 1.0:
            if self.state == RobotState.FOLLOWING_LANE:
                self.get_logger().info("🟥 Red box detected → PAUSED_AT_DESTINATION")
                self.state = RobotState.PAUSED_AT_DESTINATION
                self.pause_start_time = time.time()
                self.stop_robot()
                return
            elif self.state == RobotState.PAUSED_AT_DESTINATION:
                if time.time() - self.pause_start_time < 60:
                    self.get_logger().info("⏸️ Pausing at destination...")
                    self.stop_robot()
                    return
                else:
                    self.get_logger().info("⏩ Done pausing. Begin rotating...")
                    self.rotate_start_time = time.time()
                    self.state = RobotState.ROTATING
                    return
            elif self.state == RobotState.RETURNING:
                self.get_logger().info("🔁 Red flag seen during RETURNING (for finish check)")

        if self.state == RobotState.ROTATING:
            if time.time() - self.rotate_start_time < 6.0:
                self.get_logger().info("🔁 Rotating...")
                self.rotate_robot()
                return
            else:
                self.get_logger().info("🔙 Rotation complete → RETURNING")
                self.state = RobotState.RETURNING
                return

        if self.state == RobotState.RETURNING:
            if abs(self.current_error + 999.0) < 1.0:
                self.get_logger().warn("⚠️ No lane detected while RETURNING. Moving straight slowly.")
                twist = Twist()
                twist.linear.x = -0.1
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                return

            # PID lane following while returning
            error = -self.current_error
            self.integral += error
            derivative = error - self.prev_error
            self.prev_error = error

            steering = self.kp * error + self.kd * derivative
            steering = max(min(steering, 0.15), -0.15)
            norm_error = error / 100.0
            speed = self.max_speed * (1 - abs(norm_error))
            speed = max(speed, 0.15 if abs(steering) > 0.05 else 0.1)

            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = steering
            self.cmd_pub.publish(twist)

            self.get_logger().info(
                f"🔙 RETURNING: Speed={speed:.2f}, Steering={steering:.2f}, Error={error:.2f}"
            )
            return

        # FOLLOWING_LANE PID
        error = -self.current_error
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error

        steering = self.kp * error + self.kd * derivative
        steering = max(min(steering, 0.15), -0.15)
        norm_error = error / 100.0
        speed = self.max_speed * (1 - abs(norm_error))
        speed = max(speed, 0.15 if abs(steering) > 0.05 else 0.1)

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = steering
        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f"🚗 CMD: Speed={speed:.2f}, Steering={steering:.2f}, Error={error:.2f}, Pos=({self.robot_position[0]:.2f},{self.robot_position[1]:.2f})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = LaneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
