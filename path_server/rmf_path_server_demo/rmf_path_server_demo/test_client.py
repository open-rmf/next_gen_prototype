# Copyright 2026 Open Source Robotics Foundation
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

import json
import threading
import time
import urllib.parse
import urllib.request
from typing import Callable, Dict, List, Optional


class SpawnerTestClient:
    """Test client for interacting with the robot_spawner HTTP REST & SSE server."""

    def __init__(self, host: str = "http://localhost:8080", timeout: float = 5.0):
        self.host = host.rstrip("/")
        self.timeout = timeout
        self._sse_thread: Optional[threading.Thread] = None
        self._sse_stop_event = threading.Event()
        self.received_events: List[dict] = []
        self._events_lock = threading.Lock()

    def _get(self, endpoint: str, params: Optional[Dict[str, str]] = None) -> dict:
        url = f"{self.host}{endpoint}"
        if params:
            query_str = urllib.parse.urlencode(params)
            url = f"{url}?{query_str}"
        req = urllib.request.Request(url, headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=self.timeout) as response:
            body = response.read().decode("utf-8")
            return json.loads(body)

    def wait_until_ready(self, timeout: float = 10.0) -> bool:
        """Poll the spawner server until it is responsive."""
        start = time.time()
        while time.time() - start < timeout:
            try:
                res = self.get_config()
                if "default_radius" in res:
                    return True
            except Exception:
                pass
            time.sleep(0.2)
        return False

    def spawn_robot(self, name: str, x: float, y: float) -> dict:
        """Spawn a robot with initial position (x, y)."""
        return self._get("/spawn", {"name": name, "x": str(x), "y": str(y)})

    def set_destination(self, name: str, x: float, y: float) -> dict:
        """Set a single destination (x, y) for a robot."""
        return self.set_destinations(name, [x], [y])

    def set_destinations(self, name: str, xs: List[float], ys: List[float]) -> dict:
        """Set multiple candidate destinations for a robot."""
        x_str = ",".join(str(val) for val in xs)
        y_str = ",".join(str(val) for val in ys)
        return self._get("/destination", {"name": name, "x": x_str, "y": y_str})

    def send_scenario(self) -> dict:
        """Broadcast discovery and send scenario goals to all spawned robots."""
        return self._get("/send_scenario")

    def reset(self) -> dict:
        """Reset scenario and terminate spawned robot processes."""
        return self._get("/reset")

    def get_config(self) -> dict:
        """Get spawner configuration."""
        return self._get("/config")

    def get_map(self) -> dict:
        """Get occupancy grid map data."""
        return self._get("/map")

    def get_reservation_config(self) -> dict:
        """Get reservation system configuration."""
        return self._get("/reservation_config")

    def start_sse_listener(self, on_event: Optional[Callable[[dict], None]] = None):
        """Start listening to the /stream SSE endpoint in a background thread."""
        self._sse_stop_event.clear()

        def listen():
            url = f"{self.host}/stream"
            req = urllib.request.Request(url, headers={"Accept": "text/event-stream"})
            try:
                with urllib.request.urlopen(req, timeout=self.timeout) as response:
                    for line in response:
                        if self._sse_stop_event.is_set():
                            break
                        line_str = line.decode("utf-8").strip()
                        if line_str.startswith("data: "):
                            raw_json = line_str[6:]
                            try:
                                data = json.loads(raw_json)
                                with self._events_lock:
                                    self.received_events.append(data)
                                if on_event:
                                    on_event(data)
                            except json.JSONDecodeError:
                                pass
            except Exception:
                pass

        self._sse_thread = threading.Thread(target=listen, daemon=True)
        self._sse_thread.start()

    def stop_sse_listener(self):
        """Stop the background SSE listener thread."""
        self._sse_stop_event.set()
        if self._sse_thread and self._sse_thread.is_alive():
            self._sse_thread.join(timeout=2.0)


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Test client for robot_spawner demo")
    parser.add_argument("--host", default="http://localhost:8080", help="Spawner HTTP host")
    parser.add_argument("--reset", action="store_true", help="Reset scenario")
    args = parser.parse_args()

    client = SpawnerTestClient(host=args.host)
    if args.reset:
        print("Resetting scenario:", client.reset())
        return

    print("Checking server status...")
    if not client.wait_until_ready():
        print("Error: Server not ready")
        return

    print("Config:", client.get_config())
    print("Spawning robot_1 at (0.0, 0.0):", client.spawn_robot("robot_1", 0.0, 0.0))
    print("Spawning robot_2 at (3.0, 0.0):", client.spawn_robot("robot_2", 3.0, 0.0))
    print("Setting robot_1 destination (5.0, 0.0):", client.set_destination("robot_1", 5.0, 0.0))
    print("Setting robot_2 destination (8.0, 0.0):", client.set_destination("robot_2", 8.0, 0.0))
    print("Sending scenario:", client.send_scenario())
    print("Scenario sent successfully!")


if __name__ == "__main__":
    main()
