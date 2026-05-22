import unittest
import time

import launch
import launch_ros
import launch_testing
import pytest

import rclpy
from rmf_prototype_msgs.msg import Destination, Plan, ParticipantList, Participant, DestinationConstraints, TargetRegion, Region
from nav_msgs.msg import Odometry

@pytest.mark.launch_test
def generate_test_description():
    server_node = launch_ros.actions.Node(
        package='rmf_path_server',
        executable='rmf_path_server',
        output='screen'
    )

    return launch.LaunchDescription([
        server_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'server_node': server_node}

class TestPathServer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_path_server_node')

    def tearDown(self):
        self.node.destroy_node()

    def create_destination(self, session_id, x, y, size):
        msg = Destination()
        msg.session.uuid = [session_id] * 16
        constraint = DestinationConstraints()
        target_region = TargetRegion()
        target_region.region.hint = Region.HINT_AXIS_ALIGNED_RECTANGLE
        target_region.region.points = [float(x), float(y), float(x + size), float(y + size)]
        constraint.regions.append(target_region)
        msg.constraints = constraint
        return msg

    def create_odom(self, x, y):
        msg = Odometry()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        return msg

    def test_plan_generation_and_publication(self):
        robot_name = 'robot_1'
        received_plans = []

        self.node.create_subscription(
            Plan,
            f'{robot_name}/plan',
            lambda msg: received_plans.append(msg),
            10
        )

        dest_pub = self.node.create_publisher(
            Destination,
            f'{robot_name}/destination',
            10
        )

        odom_pub = self.node.create_publisher(
            Odometry,
            f'{robot_name}/odom',
            10
        )

        discovery_pub = self.node.create_publisher(
            ParticipantList,
            '/destination/discovery',
            10
        )

        # Wait for discovery and server to be ready
        time.sleep(2.0)

        # 1. Advertise the participant
        parts = ParticipantList()
        p = Participant()
        p.name = robot_name
        parts.participants.append(p)
        discovery_pub.publish(parts)

        time.sleep(0.5)

        # 2. Publish destination and odom
        dest = self.create_destination(1, 10.0, 10.0, 1.0)
        odom = self.create_odom(0.0, 0.0)

        start_time = time.time()
        timeout = 8.0
        while time.time() - start_time < timeout:
            discovery_pub.publish(parts)
            dest_pub.publish(dest)
            odom_pub.publish(odom)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if received_plans:
                break
            time.sleep(0.5)

        self.assertTrue(len(received_plans) > 0, "Did not receive Plan message on robot_1/plan")
        plan = received_plans[0]
        self.assertEqual(list(plan.plan_id.destination_session.uuid), list(dest.session.uuid))
        self.assertTrue(len(plan.waypoints) > 0)
        print(f"Received plan with {len(plan.waypoints)} waypoints.")
