#!/usr/bin/env python3

import rclpy
import cv2
import math
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class ArucoNav(Node):
    def __init__(self):
        super().__init__('aruco_navigator')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
    Image,
    '/camera/color/rgbd_camera/image_raw',  
    self.image_callback,
    10
)
        self.aruco_found = False
        self.navigator = BasicNavigator()

    def image_callback(self, msg):
        if self.aruco_found:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)


        if ids is not None:
            self.get_logger().info(f"ArUco marker detected! IDs: {ids.flatten()}")
            self.aruco_found = True

            marker_x = 3.0
            marker_y = 4.0
            marker_yaw = 0.0

            self.navigate_to_marker(marker_x, marker_y, marker_yaw)

    def navigate_to_marker(self, x, y, yaw=0.0):
        self.navigator.waitUntilNav2Active()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.get_logger().info("Sending goal to ArUco marker position...")
        self.navigator.goToPose(pose)

        while not self.navigator.isTaskComplete():
            self.get_logger().info("Navigating...")
            rclpy.spin_once(self)

        result = self.navigator.getResult()
        if result == 0:
            self.get_logger().info("Navigation successful!")
        else:
            self.get_logger().warn("Navigation failed or canceled.")

        self.navigator.lifecycleShutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
