"""
Generate a time-parameterized trajectory (nav_msgs/Path) from smoothed points.
"""

import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from rclpy.duration import Duration


def yaw_to_quaternion(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def generate_time_parameterized_trajectory(node, path_points, velocity=0.2):
    """
    node      : rclpy node (for time)
    path_points : list of (x,y)
    velocity  : constant linear velocity (m/s)
    returns   : nav_msgs/Path with PoseStamped entries where each pose.header.stamp = now + t_i
    """
    path_msg = Path()
    now = node.get_clock().now()
    path_msg.header.stamp = now.to_msg()
    path_msg.header.frame_id = "odom"

    if len(path_points) == 0:
        return path_msg

    # compute segment distances
    seg_dists = []
    for i in range(1, len(path_points)):
        dx = path_points[i][0] - path_points[i - 1][0]
        dy = path_points[i][1] - path_points[i - 1][1]
        seg_dists.append(math.hypot(dx, dy))

    # times: cumulative distance / velocity
    cumulative = 0.0
    times = [0.0]  # t=0 for first point
    for d in seg_dists:
        cumulative += d
        times.append(cumulative / velocity if velocity > 0 else 0.0)

    # build PoseStamped list
    for i, (pt, tsec) in enumerate(zip(path_points, times)):
        ps = PoseStamped()
        ts = now + Duration(seconds=float(tsec))
        ps.header.stamp = ts.to_msg()
        ps.header.frame_id = "odom"
        ps.pose.position = Point(x=float(pt[0]), y=float(pt[1]), z=0.0)

        # orientation: tangent to next point if available
        if i < len(path_points) - 1:
            dx = path_points[i + 1][0] - pt[0]
            dy = path_points[i + 1][1] - pt[1]
            yaw = math.atan2(dy, dx)
        else:
            if i > 0:
                dx = pt[0] - path_points[i - 1][0]
                dy = pt[1] - path_points[i - 1][1]
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0
        ps.pose.orientation = yaw_to_quaternion(yaw)
        path_msg.poses.append(ps)

    return path_msg
