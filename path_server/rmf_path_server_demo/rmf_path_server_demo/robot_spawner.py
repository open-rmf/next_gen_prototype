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
from rmf_prototype_msgs.msg import (
    Destination,
    DestinationConstraints,
    Participant,
    ParticipantList,
    Region,
    TargetRegion,
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
                self.send_ok_response({"status": "spawned", "name": name})
            else:
                self.send_error_response("Missing name or spawner node inactive")
            return

        elif self.path.startswith('/destination'):
            from urllib.parse import urlparse, parse_qs
            query = parse_qs(urlparse(self.path).query)
            name = query.get('name', [''])[0]
            x = float(query.get('x', [0.0])[0])
            y = float(query.get('y', [0.0])[0])

            if spawner_node and name:
                spawner_node.set_goal(name, x, y)
                self.send_ok_response({"status": "goal_set", "name": name})
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

        self.active_processes = {}
        self.sse_clients = []
        self.sse_clients_lock = threading.Lock()
        self.robot_goals = {}
        self.odom_subs = {}
        self.discovery_timer = None

        # Publishers for ROS 2 control plane
        self.discovery_pub = self.create_publisher(
            ParticipantList,
            '/destination/discovery',
            10
        )
        self.dest_publishers = {}

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
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid if sys.platform != 'win32' else None
            )
            self.active_processes[name] = proc
            self.get_logger().info(f"Spawned process for '{name}' successfully (PID {proc.pid})")

            # Subscribe to this robot's odometry dynamically
            self.subscribe_to_odom(name)

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
            10
        )

    def set_goal(self, name, x, y):
        self.get_logger().info(f"Setting goal for '{name}' to ({x:.2f}, {y:.2f})")
        self.robot_goals[name] = (x, y)

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
        for name, (gx, gy) in self.robot_goals.items():
            if name not in self.dest_publishers:
                self.dest_publishers[name] = self.create_publisher(
                    Destination,
                    f'{name}/destination',
                    10
                )

            dest_msg = Destination()
            dest_msg.session.uuid = [len(self.dest_publishers)] * 16
            
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
            self.get_logger().info(f"Published goal destination for '{name}' to ({gx}, {gy})")

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

        self.active_processes.clear()
        self.robot_goals.clear()

        # Destroy subscriptions and publishers
        for sub in self.odom_subs.values():
            self.destroy_subscription(sub)
        self.odom_subs.clear()

        for pub in self.dest_publishers.values():
            self.destroy_publisher(pub)
        self.dest_publishers.clear()

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
