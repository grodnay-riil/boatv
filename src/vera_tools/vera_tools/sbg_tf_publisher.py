import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sbg_driver.msg import SbgEkfQuat  # Ensure sbg_driver is installed and declared as a dependency

class TFPublisherNode(Node):
    def __init__(self):
        super().__init__('sbg_tf_broadcaster')
        # Subscribe to the /sbg/ekf_quat topic
        self.subscription = self.create_subscription(
            SbgEkfQuat,
            '/sbg/ekf_quat',
            self.ekf_quat_callback,
            10
        )
        # Create a TF broadcaster to publish transforms
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("TF Publisher Node has been started.")

    def ekf_quat_callback(self, msg: SbgEkfQuat):
        t = TransformStamped()
        # Use current time for the timestamp
        t.header.stamp = self.get_clock().now().to_msg()
        # Parent frame is "sbg_origin"
        t.header.frame_id = 'sbg_origin'
        # Child frame is "sbg_rotated"
        t.child_frame_id = 'sbg_rotated'
        
        # Set translation to the origin (0,0,0)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Set rotation from the incoming message quaternion
        t.transform.rotation.x = msg.quaternion.x
        t.transform.rotation.y = msg.quaternion.y
        t.transform.rotation.z = msg.quaternion.z
        t.transform.rotation.w = msg.quaternion.w

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug(
            f"Published TF with rotation: "
            f"({msg.quaternion.x}, {msg.quaternion.y}, {msg.quaternion.z}, {msg.quaternion.w})"
        )

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