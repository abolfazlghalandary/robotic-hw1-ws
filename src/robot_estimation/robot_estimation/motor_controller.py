#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        self.declare_parameter('wheel_radius', 0.1) 
        self.declare_parameter('wheel_base', 0.3)   
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        
        self.left_rpm_pub = self.create_publisher(Float64, 'left_wheel_rpm', 10)
        self.right_rpm_pub = self.create_publisher(Float64, 'right_wheel_rpm', 10)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        self.get_logger().info('Motor Controller node started')

    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x  
        angular_vel = msg.angular.z
        
        v_left = linear_vel - (angular_vel * self.wheel_base / 2.0)
        v_right = linear_vel + (angular_vel * self.wheel_base / 2.0)

        omega_left = v_left / self.wheel_radius 
        omega_right = v_right / self.wheel_radius
        
        rpm_left = (omega_left * 60.0) / (2.0 * 3.14159265359)
        rpm_right = (omega_right * 60.0) / (2.0 * 3.14159265359)
        
        left_msg = Float64()
        left_msg.data = rpm_left
        self.left_rpm_pub.publish(left_msg)
        
        right_msg = Float64()
        right_msg.data = rpm_right
        self.right_rpm_pub.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
