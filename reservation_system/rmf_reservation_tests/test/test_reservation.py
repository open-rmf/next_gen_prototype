import unittest
import time

import launch
import launch_ros
import launch_testing
import pytest

import rclpy
from rmf_prototype_msgs.msg import DestinationGoal, Destination, DestinationConstraints, TargetRegion, Region, ParticipantList, Participant

@pytest.mark.launch_test
def generate_test_description():
    server_node = launch_ros.actions.Node(
        package='rmf_simple_destination_server',
        executable='rmf_simple_destination_server',
        output='screen'
    )

    return launch.LaunchDescription([
        server_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'server_node': server_node}

class TestReservation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_reservation_node')

    def tearDown(self):
        self.node.destroy_node()

    def create_goal(self, session_id, x, y, size):
        msg = DestinationGoal()
        msg.session.uuid = [session_id] * 16
        constraint = DestinationConstraints()
        target_region = TargetRegion()
        target_region.region.hint = Region.HINT_AXIS_ALIGNED_RECTANGLE
        target_region.region.points = [float(x), float(y), float(x + size), float(y + size)]
        constraint.regions.append(target_region)
        msg.one_of.append(constraint)
        return msg

    def test_single_reservation(self):
        robot_name = 'robot_1'
        received_dest = []
        
        self.node.create_subscription(
            Destination,
            f'{robot_name}/destination',
            lambda msg: received_dest.append(msg),
            10
        )
        
        pub = self.node.create_publisher(
            DestinationGoal,
            f'{robot_name}/destination/goal',
            10
        )

        discovery_pub = self.node.create_publisher(
            ParticipantList,
            '/destination/discovery',
            10
        )
        
        # Wait for discovery and server to be ready
        time.sleep(2.0)

        parts = ParticipantList()
        p = Participant()
        p.name = robot_name
        parts.participants.append(p)
        discovery_pub.publish(parts)
        
        time.sleep(0.5)
        
        goal = self.create_goal(1, 0.0, 0.0, 1.0)
        
        start_time = time.time()
        timeout = 5.0
        while time.time() - start_time < timeout:
            discovery_pub.publish(parts)
            pub.publish(goal)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if received_dest:
                break
            time.sleep(0.5)
        
        self.assertTrue(len(received_dest) > 0, "Did not receive Destination message")
