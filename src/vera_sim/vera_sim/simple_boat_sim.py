import rclpy
from rclpy.node import Node
from sbg_driver.msg import SbgEkfEuler, SbgEkfQuat, SbgEkfStatus
from geometry_msgs.msg import Vector3, Quaternion, Twist
from std_msgs.msg import Header
import math
import time

class SimpleBoatSim(Node):
    def __init__(self):
        super().__init__('SimpleBoatSim')

        # Publishers for SbgEkfEuler and SbgEkfQuat
        self.euler_publisher = self.create_publisher(SbgEkfEuler, 'sbg/ekf_euler', 10)
        self.quat_publisher = self.create_publisher(SbgEkfQuat, 'sbg/ekf_quat', 10)

        # Subscriber for /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Simulation parameters
        self.yaw = 0.0  # Start heading
        self.yaw_rate = 0.0  # Initial yaw rate
        self.wave_amplitude = math.radians(5)  # ±5° waves
        self.wave_frequency = 0.5  # 0.5 Hz waves (2 sec period)

        # Timer for publishing at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_ekf_messages)

        self.start_time = time.time()

    def cmd_vel_callback(self, msg: Twist):
        """ Updates yaw rate based on /cmd_vel input """
        self.yaw_rate = msg.angular.z  # Use angular.z as the new yaw rate
        self.get_logger().info(f'Received yaw rate: {math.degrees(self.yaw_rate):.2f}°/s')

    def publish_ekf_messages(self):
        elapsed_time = time.time() - self.start_time

        # Update yaw based on the current yaw rate
        self.yaw += self.yaw_rate * 0.1  # dt = 0.1s (10Hz)
        if self.yaw > math.pi:
            self.yaw -= 2 * math.pi  # Keep yaw in [-π, π]
        elif self.yaw < -math.pi:
            self.yaw += 2 * math.pi

        # Simulate wave motion
        pitch = self.wave_amplitude * math.sin(2 * math.pi * self.wave_frequency * elapsed_time)
        roll = self.wave_amplitude * math.cos(2 * math.pi * self.wave_frequency * elapsed_time)

        # Convert Euler angles to Quaternion
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, self.yaw)

        # Create SbgEkfEuler message
        euler_msg = SbgEkfEuler()
        euler_msg.header = self.create_header()
        euler_msg.time_stamp = int(elapsed_time * 1e6)
        euler_msg.angle = Vector3(x=roll, y=pitch, z=self.yaw)
        euler_msg.accuracy = Vector3(x=math.radians(0.1), y=math.radians(0.1), z=math.radians(0.5))
        euler_msg.status = self.create_status()

        # Create SbgEkfQuat message
        quat_msg = SbgEkfQuat()
        quat_msg.header = self.create_header()
        quat_msg.time_stamp = int(elapsed_time * 1e6)
        quat_msg.quaternion = Quaternion(x=qx, y=qy, z=qz, w=qw)
        quat_msg.accuracy = Vector3(x=math.radians(0.1), y=math.radians(0.1), z=math.radians(0.5))
        quat_msg.status = self.create_status()

        # Publish messages
        self.euler_publisher.publish(euler_msg)
        self.quat_publisher.publish(quat_msg)

        self.get_logger().info(
            f'Yaw: {math.degrees(self.yaw):.2f}°, Pitch: {math.degrees(pitch):.2f}°, Roll: {math.degrees(roll):.2f}°'
        )

    def euler_to_quaternion(self, roll, pitch, yaw):
        """ Converts Euler angles to a Quaternion """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return qx, qy, qz, qw

    def create_header(self):
        """ Creates a ROS 2 message header with timestamp """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "imu"
        return header

    def create_status(self):
        """ Creates a fake status message """
        status = SbgEkfStatus()
        status.solution_mode = 1  # Assume some valid mode
        status.attitude_valid = True
        status.heading_valid = True
        status.velocity_valid = True
        status.position_valid = True
        return status

def main(args=None):
    print("Simple Boat  Simulator Node Running")
    rclpy.init(args=args)
    node = SimpleBoatSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
