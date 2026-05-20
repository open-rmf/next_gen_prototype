import unittest
import time

import launch
import launch_ros
import launch_testing
import pytest

import rclpy
from rmf_prototype_msgs.msg import DestinationGoal, Destination, DestinationError, DestinationConstraints, TargetRegion, Region

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

class TestOverlap(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_overlap_node')

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

    def test_overlapping_reservation(self):
        # Use robot_1 and robot_2 as they are supported by the server
        r1_name = 'robot_1'
        r2_name = 'robot_2'
        
        r1_received = []
        r2_errors = []
        
        self.node.create_subscription(Destination, f'{r1_name}/destination', lambda msg: r1_received.append(msg), 10)
        self.node.create_subscription(DestinationError, f'{r2_name}/destination/error', lambda msg: r2_errors.append(msg), 10)
        
        pub1 = self.node.create_publisher(DestinationGoal, f'{r1_name}/destination/goal', 10)
        pub2 = self.node.create_publisher(DestinationGoal, f'{r2_name}/destination/goal', 10)
        
        time.sleep(2.0)
        
        # Robot 1 goal
        goal1 = self.create_goal(10, 10.0, 10.0, 1.0)
        
        # Wait for robot 1 to succeed
        start_time = time.time()
        while time.time() - start_time < 5.0:
            pub1.publish(goal1)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if r1_received:
                break
            time.sleep(0.5)
        
        self.assertTrue(len(r1_received) > 0, "Robot 1 did not receive Destination message")
        
        # Robot 2 goal (overlapping with robot 1)
        goal2 = self.create_goal(11, 10.5, 10.5, 1.0)
        
        # Wait for robot 2 to fail
        start_time = time.time()
        while time.time() - start_time < 5.0:
            pub2.publish(goal2)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if r2_errors:
                break
            time.sleep(0.5)
        
        self.assertTrue(len(r2_errors) > 0, "Robot 2 did not receive DestinationError message")
