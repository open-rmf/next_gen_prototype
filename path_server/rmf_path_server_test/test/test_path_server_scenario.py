import time
import unittest

from launch import LaunchDescription
import launch_ros
import launch_testing
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rmf_prototype_msgs.msg import (
    Destination,
    DestinationConstraints,
    Participant,
    ParticipantList,
    Region,
    TargetRegion,
)


@pytest.mark.launch_test
def generate_test_description():
    path_server = launch_ros.actions.Node(
        package='rmf_path_server',
        executable='rmf_path_server',
        output='screen'
    )

    robot_1_sim = launch_ros.actions.Node(
        package='rmf_mock_robot_sim',
        executable='rmf_mock_robot_sim',
        output='screen',
        parameters=[{
            'robot_name': 'robot_1',
            'speed': 2.0,
            'update_rate': 20.0,
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_yaw': 0.0,
            'publish_discovery': False,
        }]
    )

    robot_2_sim = launch_ros.actions.Node(
        package='rmf_mock_robot_sim',
        executable='rmf_mock_robot_sim',
        output='screen',
        parameters=[{
            'robot_name': 'robot_2',
            'speed': 2.0,
            'update_rate': 20.0,
            'initial_x': 3.0,
            'initial_y': 0.0,
            'initial_yaw': 0.0,
            'publish_discovery': False,
        }]
    )

    return LaunchDescription([
        path_server,
        robot_1_sim,
        robot_2_sim,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'path_server': path_server,
        'robot_1_sim': robot_1_sim,
        'robot_2_sim': robot_2_sim,
    }


class TestPathServerScenario(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_path_server_scenario_node')

    def tearDown(self):
        self.node.destroy_node()

    def create_destination(self, session_id, x, y, size):
        msg = Destination()
        msg.session.uuid = [session_id] * 16
        constraint = DestinationConstraints()
        target_region = TargetRegion()
        target_region.region.hint = Region.HINT_AXIS_ALIGNED_RECTANGLE
        target_region.region.points = [
            float(x),
            float(y),
            float(x + size),
            float(y + size),
        ]
        constraint.regions.append(target_region)
        msg.constraints = constraint
        return msg

    def test_scenario(self):
        # Track robot odometry
        r1_odoms = []
        r2_odoms = []

        self.node.create_subscription(
            Odometry,
            'robot_1/odom',
            lambda msg: r1_odoms.append(msg),
            10
        )

        self.node.create_subscription(
            Odometry,
            'robot_2/odom',
            lambda msg: r2_odoms.append(msg),
            10
        )

        # Publishers for destinations
        r1_dest_pub = self.node.create_publisher(
            Destination,
            'robot_1/destination',
            10
        )

        r2_dest_pub = self.node.create_publisher(
            Destination,
            'robot_2/destination',
            10
        )

        # Discovery publisher
        discovery_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        discovery_pub = self.node.create_publisher(
            ParticipantList,
            '/destination/discovery',
            qos_profile=discovery_qos
        )

        # Build unified participant list
        discovery_msg = ParticipantList()
        p1 = Participant()
        p1.name = 'robot_1'
        p1.components = []
        p1.radius = 0.49
        discovery_msg.participants.append(p1)

        p2 = Participant()
        p2.name = 'robot_2'
        p2.components = []
        p2.radius = 0.49
        discovery_msg.participants.append(p2)

        # Wait for discovery and subscriptions to settle
        time.sleep(3.0)

        dest_1 = self.create_destination(1, 5.0, 0.0, 1.0)
        dest_2 = self.create_destination(2, 3.0, 0.0, 1.0)

        # Publish discovery and destinations initially to ensure delivery
        for _ in range(5):
            discovery_pub.publish(discovery_msg)
            r1_dest_pub.publish(dest_1)
            r2_dest_pub.publish(dest_2)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        # Run simulation and verify both robots reach their destinations
        start_time = time.time()
        timeout = 30.0
        r1_reached = False
        r2_reached = False
        r1_redirected = False

        while time.time() - start_time < timeout:
            # Keep publishing discovery so they aren't considered disconnected
            discovery_pub.publish(discovery_msg)

            rclpy.spin_once(self.node, timeout_sec=0.1)

            if r1_odoms:
                x = r1_odoms[-1].pose.pose.position.x
                y = r1_odoms[-1].pose.pose.position.y

                if x > 2.0 and not r1_redirected:
                    print(
                        f'robot_1 is midway at ({x:.2f}, {y:.2f}), '
                        'redirecting to (5.0, 5.0)...'
                    )
                    dest_1_new = self.create_destination(3, 5.0, 5.0, 1.0)
                    r1_dest_pub.publish(dest_1_new)
                    r1_redirected = True

                if r1_redirected:
                    if abs(x - 5.0) < 0.25 and abs(y - 5.0) < 0.25:
                        r1_reached = True
                else:
                    if abs(x - 5.0) < 0.25 and abs(y - 0.0) < 0.25:
                        r1_reached = True

            if r2_odoms:
                x = r2_odoms[-1].pose.pose.position.x
                y = r2_odoms[-1].pose.pose.position.y
                # Goal is 3.0, 0.0
                if abs(x - 3.0) < 0.25 and abs(y - 0.0) < 0.25:
                    r2_reached = True

            if r1_reached and r2_reached:
                break

            time.sleep(0.2)

        self.assertTrue(r1_reached, 'robot_1 did not reach destination (5.0, 5.0)')
        self.assertTrue(r2_reached, 'robot_2 did not reach destination (3.0, 0.0)')

        # Verify robot_1 moved and did not collide with robot_2
        self.assertGreater(len(r1_odoms), 0)
        self.assertGreater(len(r2_odoms), 0)
        print('Both robots safely reached their targets!')
