import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from nav2_msgs.srv import SetInitialPose
from lifecycle_msgs.srv import GetState
from math import sin, cos


class ReadyInitialPoseClient(Node):
    def __init__(self, x, y, yaw):
        super().__init__('ready_init_pose_client')
        self.future = Future()
        # 1. Client to check AMCL lifecycle state
        self.lifecycle_client = self.create_client(GetState, 'amcl/get_state')
        # 2. Client to set the pose
        self.pose_client = self.create_client(SetInitialPose, 'set_initial_pose')
        self.x, self.y, self.yaw = x, y, yaw
        # Start the "Wait for Ready" loop
        self.timer = self.create_timer(1.0, self.check_amcl_ready)

    def get_future(self):
        return self.future

    def check_amcl_ready(self):
        if not self.lifecycle_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('Waiting for amcl/get_state service...')
            return

        future = self.lifecycle_client.call_async(GetState.Request())
        future.add_done_callback(self.lifecycle_callback)
        self.timer.cancel()

    def lifecycle_callback(self, future):
        try:
            state = future.result().current_state
            # ID 3 == Active
            if state.id == 3:
                self.get_logger().info('AMCL is ACTIVE. Sending pose...')
                self.send_pose()
            else:
                self.get_logger().warn(f'AMCL state is {state.label} ({state.id}). Waiting...')
                self.timer.reset()
        except Exception as e:
            self.get_logger().error(f'Lifecycle check failed: {e}')
            self.timer.reset()

    def send_pose(self):
        req = SetInitialPose.Request()
        req.pose.header.frame_id = 'map'
        req.pose.header.stamp = self.get_clock().now().to_msg()
        req.pose.pose.pose.position.x = self.x
        req.pose.pose.pose.position.y = self.y
        req.pose.pose.pose.orientation.z = sin(self.yaw / 2.0)
        req.pose.pose.pose.orientation.w = cos(self.yaw / 2.0)
        req.pose.pose.covariance = [0.1]*36

        self.pose_client.call_async(req).add_done_callback(self.done)

    def done(self, _):
        self.get_logger().info('Pose set. Shutting down.')
        self.future.set_result(True)


def main():
    rclpy.init()
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    node = ReadyInitialPoseClient(x, y, yaw)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    future = node.get_future()
    executor.spin_until_future_complete(future)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
