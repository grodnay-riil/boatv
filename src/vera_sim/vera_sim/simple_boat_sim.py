import math
import rclpy
import tf_transformations
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, TwistStamped, Quaternion
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu

class SurfaceVehicleSimulator(Node):
    def __init__(self):
        super().__init__('surface_vehicle_simulator')
        
        # Declare simulation parameters
        self.declare_parameter('waves_amplitude', 0.1)  # radians
        self.declare_parameter('waves_frequency', 0.5)    # Hz
        self.declare_parameter('current_x', 0.0)          # m/s
        self.declare_parameter('current_y', 0.0)          # m/s
        self.declare_parameter('start_lat', 32.163721)    # Starting latitude
        self.declare_parameter('start_lon', 34.793327)    # Starting longitude
        self.declare_parameter('max_v', 10.0)             # Maximum velocity (m/s)
        self.declare_parameter('max_pitch_deg', 30.0)     # Maximum pitch in degrees
        
        # Declare topic and frame parameters
        self.declare_parameter('orientation_topic', 'imu/data')
        self.declare_parameter('velocity_topic', 'imu/velocity')
        self.declare_parameter('navsatfix_topic', 'imu/nav_sat_fix')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('frame_id', 'imu_link')
        
        # Declare parameter for cmd_vel timeout in seconds
        self.declare_parameter('cmd_vel_timeout_sec', 1.0)
        
        # Retrieve simulation parameters
        self.waves_amplitude = self.get_parameter('waves_amplitude').value
        self.waves_frequency = self.get_parameter('waves_frequency').value
        self.current_x = self.get_parameter('current_x').value
        self.current_y = self.get_parameter('current_y').value
        self.start_lat = self.get_parameter('start_lat').value
        self.start_lon = self.get_parameter('start_lon').value
        self.max_v = self.get_parameter('max_v').value
        self.max_pitch_deg = self.get_parameter('max_pitch_deg').value

        # Retrieve topic and frame parameters
        orientation_topic = self.get_parameter('orientation_topic').value
        velocity_topic = self.get_parameter('velocity_topic').value
        navsatfix_topic = self.get_parameter('navsatfix_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        # Retrieve timeout parameter
        cmd_vel_timeout_sec = self.get_parameter('cmd_vel_timeout_sec').value

        # Publishers using parameters
        self.orientation_pub = self.create_publisher(Imu, orientation_topic, 10)
        self.velocity_pub = self.create_publisher(TwistStamped, velocity_topic, 10)
        self.navsatfix_pub = self.create_publisher(NavSatFix, navsatfix_topic, 10)
        
        # Subscriber for cmd_vel using parameter topic name
        self.cmd_vel_sub = self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)

        # Internal state: position (in meters) and heading (theta in rad)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Last received command (initialized to zero) and time tracking
        self.last_cmd = Twist()
        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()
        
        # Timer callback for simulation updates (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # Set cmd_vel timeout based on parameter
        self.cmd_vel_timeout = Duration(seconds=cmd_vel_timeout_sec)
    
    def cmd_vel_callback(self, msg: Twist):
        # Update the last received command velocity and timestamp
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()
    
    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Check if no cmd_vel has been received within the timeout duration.
        if (now - self.last_cmd_time) > self.cmd_vel_timeout:
            # self.get_logger().warn("No cmd_vel received in the last {:.1f} seconds. Resetting velocity to zero.".format(self.cmd_vel_timeout.nanoseconds / 1e9))
            self.last_cmd = Twist()  # Reset velocity

        # Retrieve commanded velocities
        v = self.last_cmd.linear.x   # Forward velocity in m/s
        w = self.last_cmd.angular.z  # Angular velocity in rad/s

        # Update position: integrate forward motion plus constant current
        dx = (v * math.cos(self.theta) + self.current_x) * dt
        dy = (v * math.sin(self.theta) + self.current_y) * dt
        self.x += dx
        self.y += dy

        # Update heading (yaw)
        self.theta += w * dt
        yaw = self.theta
        
        # Compute wave effects: roll and pitch based on time
        t = now.nanoseconds / 1e9
        roll = self.waves_amplitude * math.sin(2 * math.pi * self.waves_frequency * t)
        max_pitch_rad = math.radians(self.max_pitch_deg)
        pitch = (self.waves_amplitude * math.sin(2 * math.pi * self.waves_frequency * t * 2)
                 - max(0, min(v / self.max_v, 1)) * max_pitch_rad)
        
        # Convert Euler angles (roll, pitch, yaw) to quaternion
        q_tuple = tf_transformations.quaternion_from_euler(roll, pitch, yaw) 
        temp_q = Quaternion()
        temp_q.x, temp_q.y, temp_q.z, temp_q.w = q_tuple
        
        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = self.frame_id
        imu_msg.orientation = temp_q
        self.orientation_pub.publish(imu_msg)

        # Convert local (x, y) displacements to geographic coordinates
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(self.start_lat))
        delta_lat = self.y / meters_per_deg_lat
        delta_lon = self.x / meters_per_deg_lon
        
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
        
        vel_msg = TwistStamped()
        vel_msg.header.stamp = now.to_msg()
        vel_msg.header.frame_id = self.frame_id
        vel_msg.twist = self.last_cmd
        self.velocity_pub.publish(vel_msg)
        # Optionally log simulation state:
        # self.get_logger().info(f"x: {self.x:.2f}, y: {self.y:.2f}, theta: {self.theta:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SurfaceVehicleSimulator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
