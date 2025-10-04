# controller_node.py (with State-Based Hybrid Control)

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from robot_nav_assignment.utils import quaternion_to_yaw
import math
import json
from enum import Enum

class RobotState(Enum):
    FOLLOWING_PATH = 1
    AVOIDING_OBSTACLE = 2

class ObstacleAvoidingController(Node):
    def __init__(self):
        super().__init__('obstacle_avoiding_controller')
        self.state = RobotState.FOLLOWING_PATH

        # --- Parameters ---
        self.declare_parameter('nominal_velocity', 0.22)
        self.declare_parameter('lookahead_time', 2.0)
        self.declare_parameter('distance_tolerance', 0.15)
        self.declare_parameter('safety_bubble_radius', 0.5) # NEW: Radius to check for obstacles
        self.declare_parameter('max_omega', 1.0)

        # Get parameters
        self.nominal_v = self.get_parameter('nominal_velocity').value
        self.lookahead_time = self.get_parameter('lookahead_time').value
        self.dist_tol = self.get_parameter('distance_tolerance').value
        self.safety_radius = self.get_parameter('safety_bubble_radius').value
        self.max_omega = self.get_parameter('max_omega').value

        # --- Class variables ---
        self.trajectory, self.laser_scan = None, None
        self.robot_x, self.robot_y, self.robot_yaw = 0.0, 0.0, 0.0
        self.have_odom = False

        # --- Subscribers and Publisher ---
        self.create_subscription(Path, '/trajectory', self.trajectory_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.timer_cb)

    def trajectory_cb(self, msg: Path):
        self.trajectory, self.state = msg.poses, RobotState.FOLLOWING_PATH
        self.get_logger().info(f'New trajectory received. State set to FOLLOWING_PATH.')

    def odom_cb(self, msg: Odometry):
        self.robot_x, self.robot_y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.robot_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.have_odom = True

    def scan_cb(self, msg: LaserScan):
        self.laser_scan = msg

    def timer_cb(self):
        if self.trajectory is None or not self.have_odom or self.laser_scan is None:
            self.stop_robot()
            return

        if self.is_goal_reached():
            self.stop_robot()
            return

        # --- STATE MACHINE LOGIC ---
        obstacle_in_path = self.is_obstacle_in_path()

        if self.state == RobotState.FOLLOWING_PATH:
            if obstacle_in_path:
                self.get_logger().warn('Obstacle detected ahead! Switching to AVOIDING_OBSTACLE state.')
                self.state = RobotState.AVOIDING_OBSTACLE
            else:
                self.run_pure_pursuit()
        
        elif self.state == RobotState.AVOIDING_OBSTACLE:
            if not obstacle_in_path:
                self.get_logger().info('Path is clear. Switching back to FOLLOWING_PATH state.')
                self.state = RobotState.FOLLOWING_PATH
                self.run_pure_pursuit()
            else:
                self.run_obstacle_avoidance()
    
    def run_pure_pursuit(self):
        target_idx = self.find_lookahead_point()
        target_pose = self.trajectory[target_idx]
        
        dx = target_pose.pose.position.x - self.robot_x
        dy = target_pose.pose.position.y - self.robot_y
        alpha = self.normalize_angle(math.atan2(dy, dx) - self.robot_yaw)
        Ld = math.hypot(dx, dy)
        
        curvature = (2.0 * math.sin(alpha)) / Ld if Ld > 1e-6 else 0.0
        v = self.nominal_v
        omega = min(self.max_omega, max(-self.max_omega, v * curvature))
        
        self.publish_cmd(v, omega)

    def run_obstacle_avoidance(self):
        # A simple "turn away from the closest obstacle" logic
        min_dist = float('inf')
        closest_angle = 0.0
        for i, scan_range in enumerate(self.laser_scan.ranges):
            if scan_range < min_dist:
                min_dist = scan_range
                closest_angle = self.laser_scan.angle_min + i * self.laser_scan.angle_increment

        # If the closest obstacle is on the left (positive angle), turn right (negative omega)
        # If the closest obstacle is on the right (negative angle), turn left (positive omega)
        omega = -1.5 * closest_angle
        omega = min(self.max_omega, max(-self.max_omega, omega))
        
        # Slow down while turning
        v = self.nominal_v / 2.0
        
        self.publish_cmd(v, omega)

    def is_obstacle_in_path(self):
        # Checks a "safety bubble" in front of the robot
        for i, scan_range in enumerate(self.laser_scan.ranges):
            scan_angle = self.laser_scan.angle_min + i * self.laser_scan.angle_increment
            # Only check a cone in front of the robot (e.g., +/- 45 degrees)
            if abs(scan_angle) < (math.pi / 4.0):
                if scan_range < self.safety_radius:
                    return True
        return False

    def find_lookahead_point(self):
        lookahead_dist = self.nominal_v * self.lookahead_time
        min_dist, closest_idx = float('inf'), 0
        for i, pose in enumerate(self.trajectory):
            dist = math.hypot(pose.pose.position.x - self.robot_x, pose.pose.position.y - self.robot_y)
            if dist < min_dist: min_dist, closest_idx = dist, i
        target_idx = closest_idx
        while target_idx < len(self.trajectory) - 1:
            dist = math.hypot(self.trajectory[target_idx].pose.position.x - self.robot_x, self.trajectory[target_idx].pose.position.y - self.robot_y)
            if dist >= lookahead_dist: break
            target_idx += 1
        return target_idx

    def is_goal_reached(self):
        dist = math.hypot(self.trajectory[-1].pose.position.x - self.robot_x, self.trajectory[-1].pose.position.y - self.robot_y)
        return dist < self.dist_tol
    
    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def publish_cmd(self, v, omega):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(omega)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidingController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()