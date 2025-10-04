import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from robot_nav_assignment.path_smoothing import catmull_rom_spline
from robot_nav_assignment.trajectory_generation import generate_time_parameterized_trajectory
import json

class PathServer(Node):
    def __init__(self):
        super().__init__('path_server')

        # Declare parameters
        self.declare_parameter('waypoints', json.dumps([
            {'x': 0.0, 'y': 0.0},
            {'x': 1.0, 'y': 0.0},
            {'x': 2.0, 'y': 1.0},
            {'x': 3.0, 'y': 1.0},
            {'x': 4.0, 'y': 0.0}
        ]))
        self.declare_parameter('samples_per_segment', 20)
        self.declare_parameter('velocity', 0.2)

        # Parse waypoints from string
        try:
            waypoints_str = self.get_parameter('waypoints').get_parameter_value().string_value
            self.waypoints = [(float(p['x']), float(p['y'])) for p in json.loads(waypoints_str)]
        except Exception:
            self.get_logger().warn('Failed to parse waypoints parameter, using default.')
            self.waypoints = [
                (0.0, 0.0),
                (1.0, 0.0),
                (2.0, 1.0),
                (3.0, 1.0),
                (4.0, 0.0)
            ]

        self.samples_per_segment = self.get_parameter('samples_per_segment').value
        self.velocity = self.get_parameter('velocity').value

        # Publishers
        self.smooth_pub = self.create_publisher(Path, '/smoothed_path', 10)
        self.traj_pub = self.create_publisher(Path, '/trajectory', 10)

        # Timer to publish
        self.timer = self.create_timer(0.5, self.timer_cb)
        self.first_publish = True

    def timer_cb(self):
        # compute smoothed path
        smoothed = catmull_rom_spline(self.waypoints, samples_per_segment=self.samples_per_segment)

        # build Path message
        smoothed_path_msg = Path()
        now = self.get_clock().now()
        smoothed_path_msg.header.frame_id = 'odom'
        smoothed_path_msg.header.stamp = now.to_msg()
        for (x, y) in smoothed:
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.header.stamp = now.to_msg()
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            smoothed_path_msg.poses.append(ps)

        self.smooth_pub.publish(smoothed_path_msg)

        # generate and publish time-parameterized trajectory
        traj_msg = generate_time_parameterized_trajectory(self, smoothed, velocity=self.velocity)
        self.traj_pub.publish(traj_msg)

        if self.first_publish:
            self.get_logger().info('Published smoothed path and trajectory.')
            self.first_publish = False

def main(args=None):
    rclpy.init(args=args)
    node = PathServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
