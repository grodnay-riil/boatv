import math
import rclpy
import tf_transformations
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, Quaternion
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix, NavSatStatus

class SurfaceVehicleSimulator(Node):
    def __init__(self):
        super().__init__('surface_vehicle_simulator')
        
        # Declare parameters with default values
        self.declare_parameter('waves_amplitude', 0.1)  # radians
        self.declare_parameter('waves_frequency', 0.5)    # Hz
        self.declare_parameter('current_x', 0.0)          # m/s
        self.declare_parameter('current_y', 0.0)          # m/s
        self.declare_parameter('start_lat', 32.163721)      # Starting latitude
        self.declare_parameter('start_lon', 34.793327)    # Starting longitude
        self.declare_parameter('max_v', 1.0)              # Maximum velocity (m/s)
        
        # Retrieve parameters
        self.waves_amplitude = self.get_parameter('waves_amplitude').value
        self.waves_frequency = self.get_parameter('waves_frequency').value
        self.current_x = self.get_parameter('current_x').value
        self.current_y = self.get_parameter('current_y').value
        self.start_lat = self.get_parameter('start_lat').value
        self.start_lon = self.get_parameter('start_lon').value
        self.max_v = self.get_parameter('max_v').value

        # Publishers
        self.position_pub = self.create_publisher(GeoPoint, 'position', 10)
        self.orientation_pub = self.create_publisher(Quaternion, 'orientation', 10)
        self.velocity_pub = self.create_publisher(Twist, 'velocity', 10)
        self.navsatfix_pub = self.create_publisher(NavSatFix, 'navsatfix', 10)
        
        # Subscriber for cmd_vel
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Internal state: position in meters (relative to start) and heading (theta in rad)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Store the last received cmd_vel (default is zero velocities)
        self.last_cmd = Twist()
        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()  # Added to track last cmd_vel timestamp
        
        # Timer callback for simulation updates (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)
    
    def cmd_vel_callback(self, msg: Twist):
        # Update the last received command velocity and the timestamp
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()
    
    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Check if no cmd_vel has been received within the last 1 second.
        if (now - self.last_cmd_time) > Duration(seconds=1.0):
            #self.get_logger().warn("No /cmd_vel received in the last second. Resetting velocity to zero.")
            self.last_cmd = Twist()  # Reset velocity to zero
        
        # Get commanded velocities
        v = self.last_cmd.linear.x   # forward velocity in m/s
        w = self.last_cmd.angular.z  # angular velocity in rad/s

        # Update position: integrate forward motion plus constant current
        dx = (v * math.cos(self.theta) + self.current_x) * dt
        dy = (v * math.sin(self.theta) + self.current_y) * dt
        self.x += dx
        self.y += dy

        # Update heading (yaw)
        self.theta += w * dt
        yaw = self.theta
        
        # Apply wave effects that influence roll and pitch only
        t = now.nanoseconds / 1e9
        roll = self.waves_amplitude * math.sin(2 * math.pi * self.waves_frequency * t)
        pitch = (self.waves_amplitude * math.sin(2 * math.pi * self.waves_frequency * t * 2)
                 - max(0, min(v / self.max_v, 1)) * math.pi * 30/180)
        
        # Convert Euler angles (roll, pitch, yaw) to quaternion using tf_transformations
        q_tuple = tf_transformations.quaternion_from_euler(roll, pitch, yaw) 
        q_msg = Quaternion()
        q_msg.x, q_msg.y, q_msg.z, q_msg.w = q_tuple
        self.orientation_pub.publish(q_msg)

        # Convert local (x, y) displacements to geographic coordinates
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(self.start_lat))
        delta_lat = self.y / meters_per_deg_lat
        delta_lon = self.x / meters_per_deg_lon
        
        geo = GeoPoint()
        geo.latitude = self.start_lat + delta_lat
        geo.longitude = self.start_lon + delta_lon
        geo.altitude = 0.0  # Assuming surface level
        self.position_pub.publish(geo)
        
        navsat_msg = NavSatFix()
        navsat_msg.header.stamp = now.to_msg()
        navsat_msg.header.frame_id = "map"
        navsat_msg.latitude = self.start_lat + delta_lat
        navsat_msg.longitude = self.start_lon + delta_lon
        navsat_msg.altitude = 0.0
        navsat_msg.status.status = NavSatStatus.STATUS_FIX
        navsat_msg.status.service = NavSatStatus.SERVICE_GPS
        navsat_msg.position_covariance = [0.0] * 9
        navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.navsatfix_pub.publish(navsat_msg)

        self.velocity_pub.publish(self.last_cmd)
        # Uncomment the next line to log simulation state:
        # self.get_logger().info(f"Simulated state: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SurfaceVehicleSimulator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
