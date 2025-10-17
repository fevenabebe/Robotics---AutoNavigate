import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity

class RobotSpawner(Node):
    def __init__(self):
        super().__init__('spawn_lane_bot')

        client = self.create_client(SpawnEntity, '/spawn_entity')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        urdf_path = os.path.join(
            get_package_share_directory('lane_following_robot'),
            'models/lane_bot/model.urdf'
        )

        with open(urdf_path, 'r') as file:
            robot_description = file.read()

        request = SpawnEntity.Request()
        request.name = 'lane_bot'
        request.xml = robot_description
        request.robot_namespace = ''
        request.initial_pose.position.x = 0.0
        request.initial_pose.position.y = 0.0
        request.initial_pose.position.z = 0.1

        # Rotate 180 degrees (pi radians) about Z axis
        request.initial_pose.orientation.x = 0.0
        request.initial_pose.orientation.y = 0.0
        request.initial_pose.orientation.z = 1.0
        request.initial_pose.orientation.w = 0.0

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Success: {future.result().status_message}')
        else:
            self.get_logger().error('Failed to spawn robot')

def main(args=None):
    rclpy.init(args=args)
    node = RobotSpawner()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
