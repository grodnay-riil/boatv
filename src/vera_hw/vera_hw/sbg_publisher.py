#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Twist
from geographic_msgs.msg import GeoPoint
from sbg_driver.msg import SbgEkfNav, SbgEkfQuat, SbgEkfVelBody, SbgImuData
from tf_transformations import quaternion_from_euler, quaternion_multiply


class SBGInsAggregator(Node):
    def __init__(self):
        super().__init__('sbg_publisher')
        
        # Orientation: Subscribe to /sbg/ekf_quat (assumed to be a Quaternion message)
        self.orientation_sub = self.create_subscription(
            SbgEkfQuat,
            '/sbg/ekf_quat',
            self.orientation_callback,
            10
        )
        self.orientation_pub = self.create_publisher(Quaternion, 'orientation', 10)
        
        # Position: Subscribe to /sbg/ekf_nav
        self.position_sub = self.create_subscription(
            SbgEkfNav,
            '/sbg/ekf_nav',
            self.position_callback,
            10
        )
        self.position_pub = self.create_publisher(GeoPoint, 'position', 10)
        
        # Velocity: Subscribe to /sbg/ekf_vel_body 
        self.velocity_sub = self.create_subscription(
            SbgEkfVelBody,
            '/sbg/ekf_vel_body',
            self.velocity_callback,
            10
        )
        self.velocity_pub = self.create_publisher(Twist, 'velocity', 10)
        
        self.get_logger().info("SBG INS Publisher Node started.")
    
    def orientation_callback(self, msg: SbgEkfQuat):   
        self.orientation_pub.publish(msg.quaternion)
        
    
    def position_callback(self, msg: SbgEkfNav):
        # Convert EkfNav to a GeoPoint.
        geo = GeoPoint()
        geo.latitude = msg.latitude
        geo.longitude = msg.longitude
        geo.altitude = msg.altitude
        self.position_pub.publish(geo)
        self.get_logger().debug(
            f"Position received and republished: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}"
        )
    
    def velocity_callback(self, msg: SbgEkfVelBody):
        # Convert EkfVelBody (assumed body-frame velocity) to a Twist message.
        twist = Twist()
        twist.linear.x = msg.velocity.x
        twist.linear.y = msg.velocity.y
        twist.linear.z = msg.velocity.z
        # Angular velocities are not provided; set to zero.
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        self.velocity_pub.publish(twist)
        self.get_logger().debug(
            f"Velocity received and republished (body frame): x={msg.velocity_x:.3f}, y={msg.velocity_y:.3f}, z={msg.velocity_z:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SBGInsAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
