from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient


from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid

import numpy as np

import rclpy


def costmap_to_occupancy_grid(cost_msg: Costmap) -> OccupancyGrid:
    """
    Converts a nav2_msgs/Costmap message to a nav_msgs/OccupancyGrid message.
    
    :param cost_msg: The incoming nav2_msgs/msg/Costmap message
    :return: A standard nav_msgs/msg/OccupancyGrid message
    """
    grid = OccupancyGrid()

    # 1. Transfer Header
    grid.header = cost_msg.header

    # 2. Transfer Metadata
    # Nav2 uses 'size_x/y', OccupancyGrid uses 'width/height'
    grid.info.resolution = cost_msg.metadata.resolution
    grid.info.width = cost_msg.metadata.size_x
    grid.info.height = cost_msg.metadata.size_y
    grid.info.origin = cost_msg.metadata.origin
    
    # Nav2 CostmapMetaData has 'map_load_time' and 'update_time'
    # We typically use map_load_time for the static map info
    grid.info.map_load_time = cost_msg.metadata.map_load_time

    # 3. Convert Data
    # Nav2 Costmap (uint8): 0 (Free) ... 254 (Lethal) ... 255 (Unknown)
    # OccupancyGrid (int8): 0 (Free) ... 100 (Lethal) ... -1 (Unknown)
    
    # We use numpy for efficiency as maps can be large
    raw_data = np.array(cost_msg.data, dtype=np.uint8)
    
    # Initialize output array with 0 (Free)
    occ_data = np.zeros_like(raw_data, dtype=np.int8)

    # Logic:
    # 255 (NO_INFORMATION) -> -1
    # 254 (LETHAL_OBSTACLE) -> 100
    # 0-253 -> Scaled to 0-99
    
    # Create masks
    mask_unknown = (raw_data == 255)
    mask_lethal = (raw_data == 254)
    mask_cost = (raw_data < 254)

    # Apply conversions
    # Scale: value * 100 / 254 roughly maps 0-253 to 0-99
    # We use floating point division then cast to int
    occ_data[mask_cost] = (raw_data[mask_cost].astype(np.float32) * 100.0 / 254.0).astype(np.int8)
    
    occ_data[mask_lethal] = 100
    occ_data[mask_unknown] = -1

    # Flatten and convert to list for the message
    grid.data = occ_data.tolist()

    return grid


class AdaptiveGoalSelector:
    def __init__(self, navigator: BasicNavigator):
        self.nav = navigator
        self.lethal_threshold = 100

    def get_path_hack(self, start, end):
        #self.nav.clearPreviousState()
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = end
        goal_msg.planner_id = ""
        goal_msg.use_start = False

        self.nav.info('Getting path...')
        send_goal_future = self.nav.compute_path_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.nav, send_goal_future)
        self.nav.goal_handle = send_goal_future.result()

        if not self.nav.goal_handle or not self.nav.goal_handle.accepted:
            self.nav.error('Get path was rejected!')
            self.nav.status = self.nav.GoalStatus.STATUS_UNKNOWN
            result = ComputePathToPose.Result()
            result.error_code = ComputePathToPose.Result.UNKNOWN
            result.error_msg = 'Get path was rejected'
            return None
        self.nav.result_future = self.nav.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.nav, self.nav.result_future)
        if self.nav.result_future.result() is None:
            return None
        
        self.status = self.nav.result_future.result().status  # type: ignore[union-attr]

        return self.nav.result_future.result().result
        

    def find_next_best_goal(self, pose_array: PoseArray, interpolation_steps=5):
        """
        Processes PoseArray to find the furthest reachable pose within a window.
        """
        global_costmap = PyCostmap2D(costmap_to_occupancy_grid(self.nav.getGlobalCostmap()))
        local_costmap = PyCostmap2D(costmap_to_occupancy_grid(self.nav.getLocalCostmap()))
        # 1. Convert PoseArray to a list of PoseStamped for Nav2 compatibility
        full_path = self._pose_array_to_list(pose_array)

        my_pose = pose_array.poses[0]
        
        # 2. Interpolate the path to ensure no gaps in costmap checks
        interpolated_path = self._interpolate_path(full_path, interpolation_steps)
        
        # 3. Filter points that are physically within the costmap window
        valid_indices = [
            id for id, p in enumerate(interpolated_path) 
            if self._is_in_costmap_bounds(p, local_costmap, global_costmap)
        ]

        """"
        low = 0
        high = len(valid_indices) - 1
        best_idx = -1

        while low <= high:
            mid = (low + high) // 2
            candidate_pose = interpolated_path[valid_indices[mid]]

            # Quick check: Is the point itself in a wall?
            if self._get_cost(candidate_pose, local_costmap, global_costmap) >= self.lethal_threshold:
                # If point is lethal, everything beyond it is likely unreachable
                high = mid - 1
                continue

            # Heavy check: Can Nav2 actually plan a path to this point?
            path = self.get_path_hack(full_path[0], candidate_pose)
            print(path)
            if path :
                best_idx = mid  # This is reachable, try to find something further
                low = mid + 1
            else:
                high = mid - 1
                """

        # Farthest goal heuristic
        mx = my_pose.position.x
        my = my_pose.position.y
        max_idx = -1
        max_val = 0
        for index in valid_indices:
            kx = interpolated_path[index].pose.position.x
            ky = interpolated_path[index].pose.position.y

            dist = (kx - mx)**2 + (ky - my)**2
            if dist > max_val:
                max_val = dist
                max_idx = index

        return interpolated_path[max_idx]
    
            
    def _interpolate_path(self, path, steps):
        """Linear interpolation between PoseStamped points."""
        new_path = []
        for i in range(len(path) - 1):
            p1 = path[i].pose.position
            p2 = path[i+1].pose.position
            for j in range(steps):
                alpha = j / steps
                interp_pose = PoseStamped()
                interp_pose.header = path[0].header
                interp_pose.pose.position.x = p1.x + alpha * (p2.x - p1.x)
                interp_pose.pose.position.y = p1.y + alpha * (p2.y - p1.y)
                interp_pose.pose.orientation = path[i].pose.orientation # Keep orientation
                new_path.append(interp_pose)
        return new_path

    def _is_in_costmap_bounds(self, pose: PoseStamped, costmap, global_costmap):
        # Logic to check if pose.x/y is within costmap.info.origin and dimensions
        #ax, ay = costmap.worldToMapValidated(pose.x, pose.y)
        x = pose.pose.position.x
        y = pose.pose.position.y
        bx, by = global_costmap.worldToMapValidated(x, y)
        cost = global_costmap.getCostXY(bx,by)
        return cost < 90

    def _get_cost(self, pose, costmap, global_costmap):
        # Logic to extract cost from the OccupancyGrid data array
        x = pose.pose.position.x
        y = pose.pose.position.y
        bx, by = global_costmap.worldToMapValidated(x, y)
        cost = global_costmap.getCostXY(bx, by)
        lx, ly = costmap.worldToMapValidated(x, y)
        #cost2 = costmap.getCostXY(lx, ly)
        return cost #max(cost, cost2)

    def _pose_array_to_list(self, pose_array):
        path_list = []
        for p in pose_array.poses:
            ps = PoseStamped()
            ps.header = pose_array.header
            ps.pose = p
            path_list.append(ps)
        return path_list
