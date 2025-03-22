#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster, TransformStamped

class TFPublisherNode(Node):
    def __init__(self):
        super().__init__('tf_publisher_node')
        self.subscription = self.create_subscription(
            Quaternion,
            '/orientation',
            self.orientation_callback,
            10
        )
        # Create a TF broadcaster to publish transforms
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("TF Publisher Node has been started.")
        
    def orientation_callback(self, msg: Quaternion):
        t = TransformStamped()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        # Use current time for the timestamp
        t.header.stamp = self.get_clock().now().to_msg()  
        # Set rotation from the incoming quaternion message
        t.transform.rotation.x = msg.x
        t.transform.rotation.y = msg.y
        t.transform.rotation.z = msg.z
        t.transform.rotation.w = msg.w
        # Set translation to zero (if desired)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug(f"Published TF with rotation: {t}")

def main(args=None):
    rclpy.init(args=args)
    node = TFPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
