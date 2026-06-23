import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker

class PoseToMarkerConverter(Node):
    def __init__(self):
        super().__init__('pose_to_marker_converter')
        
        # Subscriber to your current "hacked" PoseArray
        self.subscription = self.create_subscription(
            PoseArray,
            'global_costmap/remaining_path', # Change this to your topic
            self.callback,
            10)
        
        # Publisher for the RViz Marker
        self.marker_pub = self.create_publisher(Marker, 'visual_path_marker', 10)

    def callback(self, msg):
        if not msg.poses:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "path_visualization"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set the thickness of the line
        marker.scale.x = 0.3  # Line width in meters
        
        # Set the color (RGBA)
        marker.color.r = 1.0
        marker.color.g = 0.0  # Bright Green
        marker.color.b = 0.0  # Cyan mix
        marker.color.a = 0.8  # Fully opaque

        # Convert PoseArray positions to Marker points
        for pose in msg.poses:
            marker.points.append(pose.position)

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToMarkerConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
