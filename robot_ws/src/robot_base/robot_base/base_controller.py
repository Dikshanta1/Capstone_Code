#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import math
import time

class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')

        # Declare parameters
        self.declare_parameter('wheel_diameter', 0.083) # 83mm wheels (URDF wheel_radius * 2)
        self.declare_parameter('wheel_base_width', 0.205) # From URDF (distance between wheels)
        self.declare_parameter('ticks_per_rev', 340.0)
        self.declare_parameter('serial_port', '/dev/ttyTHS0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('speed_to_pwm_ratio', 200.0) # PWM value per 1.0 m/s

        # Get parameters
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_base_width = self.get_parameter('wheel_base_width').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.speed_to_pwm_ratio = self.get_parameter('speed_to_pwm_ratio').value

        self.meters_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_lf = 0
        self.last_lr = 0
        self.last_rf = 0
        self.last_rr = 0
        self.last_time = self.get_clock().now()
        self.first_reading = True

        # Serial
        self.ser = None
        self.connect_serial()

        # Publishers / Subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Timer for reading serial
        self.timer = self.create_timer(0.02, self.serial_read_loop) # 50Hz

        self.get_logger().info("Base controller initialized.")

    def connect_serial(self):
        while rclpy.ok() and self.ser is None:
            try:
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
                self.get_logger().info(f"Connected to {self.serial_port}")
                self.first_reading = True # Reset on reconnect
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to connect to {self.serial_port}: {e}. Retrying in 2 seconds...")
                time.sleep(2.0)

    def cmd_vel_callback(self, msg):
        v_x = msg.linear.x
        w_z = msg.angular.z

        # Differential drive kinematics mapping to left and right velocities
        v_left = v_x - (w_z * self.wheel_base_width / 2.0)
        v_right = v_x + (w_z * self.wheel_base_width / 2.0)

        # Convert velocity to PWM
        pwm_left = int(v_left * self.speed_to_pwm_ratio)
        pwm_right = int(v_right * self.speed_to_pwm_ratio)

        # Normalize and constrain PWM
        pwm_left = max(min(pwm_left, self.max_pwm), -self.max_pwm)
        pwm_right = max(min(pwm_right, self.max_pwm), -self.max_pwm)

        if self.ser and self.ser.is_open:
            try:
                cmd = f"m,{pwm_left},{pwm_right}\n"
                self.ser.write(cmd.encode('ascii'))
            except serial.SerialException:
                self.get_logger().error("Serial write failed. Connection lost.")
                self.ser.close()
                self.ser = None

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

    def serial_read_loop(self):
        if self.ser is None or not self.ser.is_open:
            self.connect_serial()
            return

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('ascii').strip()
                if line.startswith('e,'):
                    parts = line.split(',')
                    if len(parts) == 5:
                        lf, lr, rf, rr = map(int, parts[1:5])
                        self.process_encoders(lf, lr, rf, rr)
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")
            self.ser.close()
            self.ser = None

    def process_encoders(self, lf, lr, rf, rr):
        current_time = self.get_clock().now()

        if self.first_reading:
            self.last_lf = lf
            self.last_lr = lr
            self.last_rf = rf
            self.last_rr = rr
            self.last_time = current_time
            self.first_reading = False
            return

        # Calculate deltas
        d_lf = lf - self.last_lf
        d_lr = lr - self.last_lr
        d_rf = rf - self.last_rf
        d_rr = rr - self.last_rr

        self.last_lf = lf
        self.last_lr = lr
        self.last_rf = rf
        self.last_rr = rr

        # Average left and right side ticks
        d_left_ticks = (d_lf + d_lr) / 2.0
        d_right_ticks = (d_rf + d_rr) / 2.0

        d_left_dist = d_left_ticks * self.meters_per_tick
        d_right_dist = d_right_ticks * self.meters_per_tick

        d_center_dist = (d_left_dist + d_right_dist) / 2.0
        d_theta = (d_right_dist - d_left_dist) / self.wheel_base_width

        dt_msg = current_time - self.last_time
        dt = dt_msg.nanoseconds / 1e9
        self.last_time = current_time

        if dt > 0.0:
            v_x = d_center_dist / dt
            w_z = d_theta / dt
        else:
            v_x = 0.0
            w_z = 0.0

        # Update pose
        self.x += d_center_dist * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center_dist * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        self.publish_odometry(current_time, v_x, w_z)

    def publish_odometry(self, current_time, v_x, w_z):
        qx, qy, qz, qw = self.euler_to_quaternion(0, 0, self.theta)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v_x
        odom.twist.twist.angular.z = w_z

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.write(b"m,0,0\n")
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
