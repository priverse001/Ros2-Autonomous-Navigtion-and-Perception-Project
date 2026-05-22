#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np

class CarrotBroadcaster(Node):
    def __init__(self):
        super().__init__('carrot_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.sent_carrot = False

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'husky_robot_model__base_link',
                'camera_link',
                rclpy.time.Time())

            t = trans.transform.translation
            q = trans.transform.rotation

            pos = np.array([t.x, t.y, t.z])
            quat = np.array([q.x, q.y, q.z, q.w])
            rot_matrix = R.from_quat(quat).as_matrix()

            T = np.eye(4)
            T[:3, :3] = rot_matrix
            T[:3, 3] = pos

            self.get_logger().info(f"\n--- camera_link w.r.t base_link ---\n{T}")

        except Exception as e:
            self.get_logger().warn(f"camera_link not available: {e}")
            return


        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "husky_robot_model__base_link"
        t.child_frame_id = "carrot"
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = CarrotBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
