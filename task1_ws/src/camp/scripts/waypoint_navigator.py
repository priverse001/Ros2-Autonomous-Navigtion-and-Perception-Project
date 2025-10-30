#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import math
from rclpy.node import Node


class PoseTracker(Node):
    def __init__(self):
        super().__init__('pose_tracker')
        self.current_pose = None
        self.subscription = self.create_subscription(
            PoseStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        self.current_pose = msg.pose

def create_pose(x, y, yaw, frame_id='map', stamp=None):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    if stamp:
        pose.header.stamp = stamp
    pose.pose.position.x = x
    pose.pose.position.y = y

    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose

def distance(p1, p2):
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    return math.hypot(dx, dy)

def yaw_from_quaternion(qz, qw):
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)

def yaw_diff(yaw1, yaw2):
    return abs((yaw1 - yaw2 + math.pi) % (2 * math.pi) - math.pi)

def main():
    rclpy.init()
    navigator = BasicNavigator()
    pose_tracker = PoseTracker()

    rclpy.spin_once(pose_tracker, timeout_sec=1.0)

    initial_pose = create_pose(-7.0, 5.0, 3.14, stamp=navigator.get_clock().now().to_msg())
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    goal_poses = [
        create_pose(-13.0, -4.5, 3.14, stamp=navigator.get_clock().now().to_msg()),
        create_pose(10.0, 5.0, 0.0, stamp=navigator.get_clock().now().to_msg()),
        create_pose(1.5, -4.0, 1.57, stamp=navigator.get_clock().now().to_msg())
    ]

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    current_goal_idx = 0
    i = 0
    goal_reached_flags = [False] * len(goal_poses)
    pos_tol = 0.5      
    yaw_tol = 0.35     

    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            wp_idx = feedback.current_waypoint
            print(f"[Nav] Approaching waypoint {wp_idx + 1} / {len(goal_poses)}")

            rclpy.spin_once(pose_tracker, timeout_sec=0.1)
            if pose_tracker.current_pose is None:
                print("[Warning] Waiting for current pose...")
                continue

            robot_pose = pose_tracker.current_pose
            goal_pose = goal_poses[wp_idx].pose

            pos_error = distance(robot_pose.position, goal_pose.position)
            robot_yaw = yaw_from_quaternion(robot_pose.orientation.z, robot_pose.orientation.w)
            goal_yaw = yaw_from_quaternion(goal_pose.orientation.z, goal_pose.orientation.w)
            yaw_error = yaw_diff(robot_yaw, goal_yaw)

            if pos_error < pos_tol and yaw_error < yaw_tol:
                print(f"[Nav] Close enough to waypoint {wp_idx + 1}. Skipping to next.")
                goal_reached_flags[wp_idx] = True
                current_goal_idx = wp_idx + 1

                if current_goal_idx < len(goal_poses):
                    navigator.followWaypoints(goal_poses[current_goal_idx:])
                else:
                    break

        now = navigator.get_clock().now()
        if now - nav_start > Duration(seconds=600.0):
            navigator.cancelTask()
            print("[Nav] Timeout. Cancelling task.")
            break

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Navigation completed!")
    elif result == TaskResult.CANCELED:
        print("Navigation canceled.")
    elif result == TaskResult.FAILED:
        print("Navigation failed.")
    else:
        print("Unknown task result.")

    navigator.lifecycleShutdown()
    pose_tracker.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()
