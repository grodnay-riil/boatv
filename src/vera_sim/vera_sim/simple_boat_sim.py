import math
import rclpy
import tf_transformations
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from geographic_msgs.msg import GeoPoint

class SurfaceVehicleSimulator(Node):
    def __init__(self):
        super().__init__('surface_vehicle_simulator')
        
        # Declare parameters with default values
        self.declare_parameter('waves_amplitude', 0.1)  # radians
        self.declare_parameter('waves_frequency', 0.5)    # Hz
        self.declare_parameter('current_x', 0.0)          # m/s
        self.declare_parameter('current_y', 0.0)          # m/s
        self.declare_parameter('start_lat', 37.7749)      # Example: San Francisco latitude
        self.declare_parameter('start_lon', -122.4194)    # Example: San Francisco longitude
        self.declare_parameter('max_v', 1.0)              # maximum velocity (m/s)
        
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
        
        # Subscriber for cmd_vel
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Internal state: position in meters (relative to start) and heading (theta in rad)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Store the last received cmd_vel (default is zero velocities)
        self.last_cmd = Twist()
        self.last_time = self.get_clock().now()
        
        # Timer callback for simulation updates (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)
    
    def cmd_vel_callback(self, msg: Twist):
        # Update the last received command velocity
        self.last_cmd = msg
    
    def timer_callback(self):
        # Calculate elapsed time (dt)
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

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
        # For pitch, apply a wave effect (at twice the frequency) and add a term proportional to x velocity ratio.
        pitch =  (self.waves_amplitude * math.sin(2 * math.pi * self.waves_frequency * t * 2) - max(0,min(v / self.max_v, 1)) * math.pi*30/180)
        
        # Convert Euler angles (roll, pitch, yaw) to quaternion using tf_transformations
        q_tuple = tf_transformations.quaternion_from_euler(roll, pitch, yaw) 
        q_msg = Quaternion()
        q_msg.x = q_tuple[0]
        q_msg.y = q_tuple[1]
        q_msg.z = q_tuple[2]
        q_msg.w = q_tuple[3]
        self.orientation_pub.publish(q_msg)

        # Convert local (x, y) displacements to geographic coordinates using pyproj
      # Convert local (x, y) displacements to geographic coordinates
        # Approximation: 1 deg latitude ~ 111320 m; longitude factor depends on latitude
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(self.start_lat))
        delta_lat = self.y / meters_per_deg_lat
        delta_lon = self.x / meters_per_deg_lon
        
        geo = GeoPoint()
        geo.latitude = self.start_lat + delta_lat
        geo.longitude = self.start_lon + delta_lon
        geo.altitude = 0.0  # Assuming surface level
        self.position_pub.publish(geo)

        # Immediately publish the received velocity command unchanged
        self.velocity_pub.publish(self.last_cmd)
        # self.get_logger().info(f"Simulated state: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SurfaceVehicleSimulator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
