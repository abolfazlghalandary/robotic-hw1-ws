#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
import math

class OdometryEstimator(Node):
    def __init__(self):
        super().__init__('odometry_estimator')
        
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_base', 0.3)
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.left_rpm = 0.0
        self.right_rpm = 0.0
        
        self.last_time = self.get_clock().now()
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        self.left_rpm_sub = self.create_subscription(
            Float64, 'left_wheel_rpm', self.left_rpm_callback, 10)
        self.right_rpm_sub = self.create_subscription(
            Float64, 'right_wheel_rpm', self.right_rpm_callback, 10)
        
        self.timer = self.create_timer(0.05, self.update_odometry)  
        
        self.get_logger().info('Odometry Estimator node started')

    def left_rpm_callback(self, msg):
        self.left_rpm = msg.data

    def right_rpm_callback(self, msg):
        self.right_rpm = msg.data

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt == 0:
            return
        
        omega_left = (self.left_rpm * 2.0 * math.pi) / 60.0
        omega_right = (self.right_rpm * 2.0 * math.pi) / 60.0
        
        v_left = omega_left * self.wheel_radius
        v_right = omega_right * self.wheel_radius
        
        v = (v_right + v_left) / 2.0  
        omega = (v_right - v_left) / self.wheel_base  
        
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = omega * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)
        
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        
        self.odom_pub.publish(odom)
        
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = OdometryEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
