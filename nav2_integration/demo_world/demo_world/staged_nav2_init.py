"""Start a simulated robot's Nav2 stack in readiness order."""

from math import cos, sin
import sys

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState

from nav2_msgs.srv import ManageLifecycleNodes, SetInitialPose

from nav_msgs.msg import Odometry

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener


class StagedNav2Init(Node):
    """Activate Nav2 after the simulated robot and transforms are ready."""

    def __init__(self, x, y, yaw):
        """Initialize the coordinator."""
        super().__init__('staged_nav2_init')
        self.x = x
        self.y = y
        self.yaw = yaw
        self.done = Future()
        self.stage = 'waiting_for_robot'
        self.pending = False
        self.odom_received = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, 'odom', self.receive_odometry, 10)
        self.localization_manager = self.create_client(
            ManageLifecycleNodes,
            'lifecycle_manager_localization/manage_nodes',
        )
        self.navigation_manager = self.create_client(
            ManageLifecycleNodes,
            'lifecycle_manager_navigation/manage_nodes',
        )
        self.amcl_state = self.create_client(GetState, 'amcl/get_state')
        self.bt_state = self.create_client(GetState, 'bt_navigator/get_state')
        self.pose_client = self.create_client(
            SetInitialPose,
            'set_initial_pose',
        )
        self.timer = self.create_timer(0.5, self.tick)

    def receive_odometry(self, _):
        """Record that Gazebo is publishing odometry."""
        self.odom_received = True

    def transform_ready(self, target, source):
        """Check whether a transform is available."""
        return self.tf_buffer.can_transform(target, source, Time())

    def start_manager(self, client, next_stage, label):
        """Request lifecycle startup."""
        if not client.service_is_ready():
            return
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        self.pending = True
        future = client.call_async(request)
        future.add_done_callback(
            lambda result: self.manager_started(
                result,
                next_stage,
                label,
            )
        )

    def manager_started(self, future, next_stage, label):
        """Handle a lifecycle startup response."""
        self.pending = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warning(f'Cannot start {label}: {error}')
            return
        if not response.success:
            self.get_logger().warning(f'{label} startup failed; retrying')
            return
        self.stage = next_stage

    def wait_for_active(self, client, next_stage):
        """Poll a lifecycle node until it is active."""
        if not client.service_is_ready():
            return
        self.pending = True
        future = client.call_async(GetState.Request())
        future.add_done_callback(
            lambda result: self.state_received(result, next_stage)
        )

    def state_received(self, future, next_stage):
        """Handle a lifecycle state response."""
        self.pending = False
        try:
            state = future.result().current_state
        except Exception as error:
            self.get_logger().warning(f'Cannot read lifecycle state: {error}')
            return
        if state.id == State.PRIMARY_STATE_ACTIVE:
            self.stage = next_stage

    def send_pose(self):
        """Send the robot's initial pose."""
        if not self.pose_client.service_is_ready():
            return
        request = SetInitialPose.Request()
        request.pose.header.frame_id = 'map'
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.pose.position.x = self.x
        request.pose.pose.pose.position.y = self.y
        request.pose.pose.pose.orientation.z = sin(self.yaw / 2.0)
        request.pose.pose.pose.orientation.w = cos(self.yaw / 2.0)
        request.pose.pose.covariance = [0.1] * 36
        self.pending = True
        future = self.pose_client.call_async(request)
        future.add_done_callback(self.pose_sent)

    def pose_sent(self, future):
        """Continue after setting the initial pose."""
        self.pending = False
        try:
            future.result()
        except Exception as error:
            self.get_logger().warning(f'Cannot set initial pose: {error}')
            return
        self.stage = 'waiting_for_map_transform'

    def tick(self):
        """Advance the startup sequence."""
        if self.pending:
            return

        if self.stage == 'waiting_for_robot':
            if (
                self.odom_received
                and self.transform_ready('odom', 'base_link')
            ):
                self.get_logger().info(
                    'Robot transforms are ready; starting localization'
                )
                self.stage = 'starting_localization'
            return

        if self.stage == 'starting_localization':
            self.start_manager(
                self.localization_manager,
                'waiting_for_amcl',
                'localization',
            )
            return

        if self.stage == 'waiting_for_amcl':
            self.wait_for_active(self.amcl_state, 'setting_pose')
            return

        if self.stage == 'setting_pose':
            self.send_pose()
            return

        if self.stage == 'waiting_for_map_transform':
            if self.transform_ready('map', 'base_link'):
                self.get_logger().info(
                    'Localization is ready; starting navigation'
                )
                self.stage = 'starting_navigation'
            return

        if self.stage == 'starting_navigation':
            self.start_manager(
                self.navigation_manager,
                'waiting_for_navigation',
                'navigation',
            )
            return

        if self.stage == 'waiting_for_navigation':
            self.wait_for_active(self.bt_state, 'ready')
            return

        if self.stage == 'ready':
            self.get_logger().info('Nav2 startup complete')
            self.timer.cancel()
            self.done.set_result(True)


def main():
    """Run the staged startup coordinator."""
    rclpy.init()
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    node = StagedNav2Init(x, y, yaw)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin_until_future_complete(node.done)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
