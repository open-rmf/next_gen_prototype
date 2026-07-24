from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.costmap_2d import PyCostmap2D
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, PoseStamped, PoseArray
from std_msgs.msg import String
from demo_world.goal_selector import AdaptiveGoalSelector
import time
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap

import numpy as np


"""
This is in charge of telling where the robot should head next.
It works by registering the robot to the /registration topic
and then it 
"""
class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.registration_pub = self.create_publisher(String, '/registration', 10)
        #self.subscription  # prevent unused variable warning
        self.get_logger().info('Pose subscriber node started')
        self.timer = self.create_timer(1.0, self.heartbeat)
        self.nav = BasicNavigator(namespace=self.get_namespace())
        #self.nav.waitUntilNav2Active()
        self.get_logger().info("NAV2 now ACTIVE!")
        qos_profile = QoSProfile(
            depth=1, # Keep last 10 messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.reentrant_group = ReentrantCallbackGroup()
        self.sub = self.create_subscription(Point, 'global_costmap/next_goal', self.next_goal_recv, qos_profile, callback_group=self.reentrant_group)
        self.prev_point = None
        self.sub
        self.sub1 = self.create_subscription(PoseArray, 'global_costmap/remaining_path', self.path_recv, qos_profile, callback_group=self.reentrant_group)
        self.sub1
        self.goal_selector = AdaptiveGoalSelector(self.nav)

    def heartbeat(self):
        ns = self.get_namespace()
        self.registration_pub.publish(String(data=ns))

    def path_recv(self, msg: PoseArray):
        print("Path received")
        goal = self.goal_selector.find_next_best_goal(msg)
        print(f"{goal}")
        #goal = PoseStamped()
        goal.header.frame_id = "map"
        #goal
        self.nav.goToPose(goal)

    def next_goal_recv(self, point: Point):
        self.get_logger().info(f"Received next goal as {point}")
        #gcp = PyCostmap2D(costmap_to_occupancy_grid(self.nav.getGlobalCostmap()))
        #print(f"{gcp}")
        #lcp = PyCostmap2D(costmap_to_occupancy_grid(self.nav.getLocalCostmap()))
        #print(lcp.header)
        #gx, gy = gcp.worldToMapValidated(point.x, point.y)
        if self.prev_point is None:
            self.get_logger().info("Setting new nav2 goal")
            pose = PoseStamped()
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.orientation.w = 1.0
            pose.header.frame_id = "map"
            #self.nav.goToPose(pose)
            self.prev_point = point
            return
        dist = (self.prev_point.x - point.x)**2 + (self.prev_point.y - point.y)**2

        if dist < 1:
            self.get_logger().info("Prev goal near current goal, doing nothing")
            return

        self.get_logger().info("Setting new nav2 goal")
        pose = PoseStamped()
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.orientation.w = 1.0
        pose.header.frame_id = "map"
        #self.nav.goToPose(pose)
        self.prev_point = point
        return

def main(args=None):
    time.sleep(30)
    rclpy.init(args=args)
    pose_subscriber = RobotController()
    # cant use waitUntilNav2Active cause it sets the wrong pose
    #pose_subscriber.nav.waitUntilNav2Active()
    executor = MultiThreadedExecutor()
    executor.add_node(pose_subscriber)
    executor.spin()
    pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()

