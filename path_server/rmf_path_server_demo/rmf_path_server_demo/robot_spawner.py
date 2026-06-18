# Copyright 2026 OSRA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import http.server
import json
import os
import queue
import socketserver
import subprocess
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rmf_prototype_msgs.msg import (
    Destination,
    DestinationConstraints,
    DestinationGoal,
    Participant,
    ParticipantList,
    Region,
    TargetRegion,
    Plan,
    Progress,
)


# Global node reference for the HTTP request handler to access
spawner_node = None


class ThreadingHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    allow_reuse_address = True


class DemoRequestHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        # Direct server logs to ROS logger instead of stderr
        if spawner_node:
            spawner_node.get_logger().info(f"HTTP: {format % args}")
        else:
            super().log_message(format, *args)

    def do_GET(self):
        global spawner_node

        if self.path.startswith('/stream'):
            # Server-Sent Events (SSE) endpoint
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            q = queue.Queue()
            if spawner_node:
                spawner_node.add_client(q)

            try:
                while True:
                    try:
                        data = q.get(timeout=5.0)
                        self.wfile.write(f"data: {data}\n\n".encode('utf-8'))
                        self.wfile.flush()
                    except queue.Empty:
                        # Send keep-alive comment to prevent connection timeouts
                        self.wfile.write(b": keep-alive\n\n")
                        self.wfile.flush()
            except Exception as e:
                if spawner_node:
                    spawner_node.get_logger().info(f"SSE Client disconnected: {e}")
            finally:
                if spawner_node:
                    spawner_node.remove_client(q)
            return

        elif self.path.startswith('/spawn'):
            # Parse query parameters
            from urllib.parse import urlparse, parse_qs
            query = parse_qs(urlparse(self.path).query)
            name = query.get('name', [''])[0]
            x = float(query.get('x', [0.0])[0])
            y = float(query.get('y', [0.0])[0])

            if spawner_node and name:
                spawner_node.spawn_robot(name, x, y)
                self.send_ok_response({"status": "spawned", "name": name, "radius": 0.49})
            else:
                self.send_error_response("Missing name or spawner node inactive")
            return

        elif self.path.startswith('/config'):
            use_dest = spawner_node.use_destination_server if spawner_node else False
            self.send_ok_response({"default_radius": 0.49, "use_destination_server": use_dest})
            return

        elif self.path.startswith('/destination'):
            from urllib.parse import urlparse, parse_qs
            query = parse_qs(urlparse(self.path).query)
            name = query.get('name', [''])[0]
            x_str = query.get('x', [''])[0]
            y_str = query.get('y', [''])[0]

            if spawner_node and name:
                xs = [float(val) for val in x_str.split(',')] if x_str else []
                ys = [float(val) for val in y_str.split(',')] if y_str else []
                spawner_node.set_goals(name, xs, ys)
                self.send_ok_response({"status": "goals_set", "name": name})
            else:
                self.send_error_response("Missing name or spawner node inactive")
            return

        elif self.path.startswith('/send_scenario'):
            if spawner_node:
                spawner_node.send_scenario()
                self.send_ok_response({"status": "scenario_sent"})
            else:
                self.send_error_response("Spawner node inactive")
            return

        elif self.path.startswith('/reset'):
            if spawner_node:
                spawner_node.reset_scenario()
                self.send_ok_response({"status": "reset"})
            else:
                self.send_error_response("Spawner node inactive")
            return

        # Otherwise serve static files
        super().do_GET()

    def send_ok_response(self, payload):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(payload).encode('utf-8'))

    def send_error_response(self, message):
        self.send_response(400)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps({"error": message}).encode('utf-8'))


class RobotSpawnerNode(Node):
    def __init__(self):
        super().__init__('robot_spawner')
        self.get_logger().info("Initializing Robot Spawner & HTTP Bridge Node...")

        # Parameters
        self.declare_parameter('use_destination_server', False)
        self.use_destination_server = self.get_parameter('use_destination_server').value

        self.active_processes = {}
        self.active_log_files = {}
        self.sse_clients = []
        self.sse_clients_lock = threading.Lock()
        self.robot_goals = {}
        self.odom_subs = {}
        self.plan_subs = {}
        self.progress_subs = {}
        self.destination_subs = {}
        self.discovery_timer = None
        self.discovery_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        self.reliable_transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Publishers for ROS 2 control plane
        self.discovery_pub = self.create_publisher(
            ParticipantList,
            '/destination/discovery',
            qos_profile=self.discovery_qos
        )
        self.dest_publishers = {}
        self.dest_goal_publishers = {}

        # Get the package share directory and resolve static files path
        try:
            share_dir = get_package_share_directory('rmf_path_server_demo')
            self.web_dir = os.path.join(share_dir, 'www')
        except Exception as e:
            self.get_logger().error(f"Failed to locate share directory: {e}")
            self.web_dir = "./www"

        # Start the HTTP server
        self.start_http_server(port=8080)

    def start_http_server(self, port):
        def serve():
            handler = lambda *args, **kwargs: DemoRequestHandler(
                *args, directory=self.web_dir, **kwargs
            )
            try:
                with ThreadingHTTPServer(("", port), handler) as httpd:
                    self.get_logger().info(
                        f"Web server running on http://localhost:{port}/ serving {self.web_dir}"
                    )
                    httpd.serve_forever()
            except Exception as e:
                self.get_logger().error(f"HTTP Server error: {e}")

        self.web_thread = threading.Thread(target=serve, daemon=True)
        self.web_thread.start()

    # Client subscription handling for SSE
    def add_client(self, q):
        with self.sse_clients_lock:
            self.sse_clients.append(q)
            self.get_logger().info(f"New SSE Client registered. Total clients: {len(self.sse_clients)}")

    def remove_client(self, q):
        with self.sse_clients_lock:
            if q in self.sse_clients:
                self.sse_clients.remove(q)
                self.get_logger().info(f"SSE Client removed. Total clients: {len(self.sse_clients)}")

    def broadcast(self, data):
        with self.sse_clients_lock:
            for q in self.sse_clients:
                q.put(data)

    # Spawns a mock simulator node
    def spawn_robot(self, name, x, y):
        if name in self.active_processes:
            poll = self.active_processes[name].poll()
            if poll is None:
                self.get_logger().info(f"Robot '{name}' is already running.")
                return

        # Ensure any orphaned mock robot simulation process for this name is killed first
        try:
            subprocess.run(['pkill', '-f', f'robot_name:={name}'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

        self.get_logger().info(f"Spawning robot '{name}' at ({x:.2f}, {y:.2f})")

        cmd = [
            'ros2', 'run', 'rmf_mock_robot_sim', 'rmf_mock_robot_sim',
            '--ros-args',
            '-p', f'robot_name:={name}',
            '-p', f'initial_x:={x}',
            '-p', f'initial_y:={y}',
            '-p', 'publish_discovery:=false'
        ]

        try:
            os.makedirs('mock_sim_logs', exist_ok=True)
            log_file = open(f'mock_sim_logs/{name}.log', 'w')
            self.active_log_files[name] = log_file

            proc = subprocess.Popen(
                cmd,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid if sys.platform != 'win32' else None
            )
            self.active_processes[name] = proc
            self.get_logger().info(f"Spawned process for '{name}' successfully (PID {proc.pid})")

            # Subscribe to this robot's topics dynamically
            self.subscribe_to_odom(name)
            self.subscribe_to_plan(name)
            self.subscribe_to_progress(name)
            self.subscribe_to_destination(name)

        except Exception as e:
            self.get_logger().error(f"Failed to spawn robot process: {e}")

    def subscribe_to_odom(self, name):
        if name in self.odom_subs:
            return

        self.get_logger().info(f"Subscribing to dynamic odom topic for '{name}'")

        # Capture the name closure correctly
        def odom_callback(msg, r_name=name):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            data = json.dumps({
                "type": "odom",
                "name": r_name,
                "x": x,
                "y": y
            })
            self.broadcast(data)

        self.odom_subs[name] = self.create_subscription(
            Odometry,
            f'{name}/odom',
            odom_callback,
            qos_profile=self.reliable_transient_qos
        )

    def subscribe_to_plan(self, name):
        if name in self.plan_subs:
            return

        self.get_logger().info(f"Subscribing to dynamic plan topic for '{name}'")

        def plan_callback(msg, r_name=name):
            wps = []
            for wp in msg.waypoints:
                blockers = []
                for blocker in wp.departure_blockers:
                    blockers.append({
                        "name": blocker.name,
                        "required_progress": float(blocker.required_progress)
                    })
                wps.append({
                    "x": float(wp.position[0]),
                    "y": float(wp.position[1]),
                    "progress": float(wp.progress),
                    "departure_blockers": blockers
                })
            data = json.dumps({
                "type": "plan",
                "name": r_name,
                "waypoints": wps
            })
            self.broadcast(data)

        self.plan_subs[name] = self.create_subscription(
            Plan,
            f'{name}/plan',
            plan_callback,
            qos_profile=self.reliable_transient_qos
        )

    def subscribe_to_progress(self, name):
        if name in self.progress_subs:
            return

        self.get_logger().info(f"Subscribing to dynamic progress topic for '{name}'")

        def progress_callback(msg, r_name=name):
            data = json.dumps({
                "type": "progress",
                "name": r_name,
                "progress": float(msg.progress)
            })
            self.broadcast(data)

        self.progress_subs[name] = self.create_subscription(
            Progress,
            f'{name}/plan/progress',
            progress_callback,
            qos_profile=self.reliable_transient_qos
        )

    def subscribe_to_destination(self, name):
        if name in self.destination_subs:
            return

        self.get_logger().info(f"Subscribing to dynamic destination topic for '{name}'")

        def destination_callback(msg, r_name=name):
            try:
                if msg.constraints.regions:
                    region = msg.constraints.regions[0].region
                    if region.hint == Region.HINT_AXIS_ALIGNED_RECTANGLE and len(region.points) >= 2:
                        gx = region.points[0]
                        gy = region.points[1]
                        data = json.dumps({
                            "type": "active_destination",
                            "name": r_name,
                            "x": gx,
                            "y": gy
                        })
                        self.broadcast(data)
            except Exception as e:
                self.get_logger().error(f"Error parsing destination message: {e}")

        self.destination_subs[name] = self.create_subscription(
            Destination,
            f'{name}/destination',
            destination_callback,
            qos_profile=self.reliable_transient_qos
        )

    def set_goal(self, name, x, y):
        self.set_goals(name, [x], [y])

    def set_goals(self, name, xs, ys):
        goals = list(zip(xs, ys))
        self.get_logger().info(f"Setting goals for '{name}' to {goals}")
        self.robot_goals[name] = goals
        if self.discovery_timer:
            self.publish_destination(name)

    def publish_destination(self, name):
        import uuid
        goals = self.robot_goals.get(name, [])
        if not goals:
            return

        if self.use_destination_server:
            if name not in self.dest_goal_publishers:
                self.dest_goal_publishers[name] = self.create_publisher(
                    DestinationGoal,
                    f'{name}/destination/goal',
                    qos_profile=self.reliable_transient_qos
                )

            goal_msg = DestinationGoal()
            goal_msg.session.uuid = list(uuid.uuid4().bytes)

            for gx, gy in goals:
                constraint = DestinationConstraints()
                target_region = TargetRegion()
                target_region.region.hint = Region.HINT_AXIS_ALIGNED_RECTANGLE
                target_region.region.points = [
                    float(gx),
                    float(gy),
                    float(gx + 1.0),
                    float(gy + 1.0)
                ]
                constraint.regions.append(target_region)
                goal_msg.one_of.append(constraint)

            goal_msg.cost_bias = [0.0] * len(goals)

            self.dest_goal_publishers[name].publish(goal_msg)
            self.get_logger().info(f"Published DestinationGoal for '{name}' with {len(goals)} options, session {goal_msg.session.uuid}")

        else:
            if name not in self.dest_publishers:
                self.dest_publishers[name] = self.create_publisher(
                    Destination,
                    f'{name}/destination',
                    qos_profile=self.reliable_transient_qos
                )

            gx, gy = goals[0]
            dest_msg = Destination()
            dest_msg.session.uuid = list(uuid.uuid4().bytes)

            constraint = DestinationConstraints()
            target_region = TargetRegion()
            target_region.region.hint = Region.HINT_AXIS_ALIGNED_RECTANGLE
            target_region.region.points = [
                float(gx),
                float(gy),
                float(gx + 1.0),
                float(gy + 1.0)
            ]
            constraint.regions.append(target_region)
            dest_msg.constraints = constraint

            self.dest_publishers[name].publish(dest_msg)
            self.get_logger().info(f"Published Destination for '{name}' to ({gx}, {gy}) with UUID {dest_msg.session.uuid}")

    def send_scenario(self):
        self.get_logger().info("Broadcasting scenario goals and discovery...")

        # 1. Publish discovery list containing all active robots
        discovery_msg = ParticipantList()
        for name in self.active_processes.keys():
            p = Participant()
            p.name = name
            p.components = []
            discovery_msg.participants.append(p)

        self.discovery_pub.publish(discovery_msg)

        # Set up a periodic timer in the ROS node to maintain the discovery heartbeats
        if self.discovery_timer:
            self.discovery_timer.cancel()

        def timer_callback():
            self.discovery_pub.publish(discovery_msg)

        self.discovery_timer = self.create_timer(1.0, timer_callback)

        # 2. Publish goals for all robots to their destination topics
        for name in self.robot_goals.keys():
            self.publish_destination(name)

    def reset_scenario(self):
        self.get_logger().info("Resetting scenario, shutting down all simulator processes...")
        
        # Stop discovery timer
        if self.discovery_timer:
            self.discovery_timer.cancel()
            self.discovery_timer = None

        # Kill all mock robot subprocesses
        for name, proc in self.active_processes.items():
            if proc.poll() is None:
                self.get_logger().info(f"Terminating '{name}' simulator process (PID {proc.pid})")
                proc.terminate()
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    proc.kill()

        # Also run a clean pkill to kill any orphaned or untracked mock robot sims
        try:
            subprocess.run(['pkill', '-f', 'rmf_mock_robot_sim'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

        self.active_processes.clear()
        for log_file in self.active_log_files.values():
            try:
                log_file.close()
            except Exception:
                pass
        self.active_log_files.clear()
        self.robot_goals.clear()

        # Destroy subscriptions and publishers
        for sub in self.odom_subs.values():
            self.destroy_subscription(sub)
        self.odom_subs.clear()

        for sub in self.plan_subs.values():
            self.destroy_subscription(sub)
        self.plan_subs.clear()

        for sub in self.progress_subs.values():
            self.destroy_subscription(sub)
        self.progress_subs.clear()

        for sub in self.destination_subs.values():
            self.destroy_subscription(sub)
        self.destination_subs.clear()

        for pub in self.dest_publishers.values():
            self.destroy_publisher(pub)
        self.dest_publishers.clear()

        for pub in self.dest_goal_publishers.values():
            self.destroy_publisher(pub)
        self.dest_goal_publishers.clear()

        self.get_logger().info("Reset complete.")

    def shutdown(self):
        self.reset_scenario()


def main(args=None):
    global spawner_node
    rclpy.init(args=args)
    spawner_node = RobotSpawnerNode()
    try:
        rclpy.spin(spawner_node)
    except KeyboardInterrupt:
        pass
    finally:
        spawner_node.shutdown()
        spawner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
