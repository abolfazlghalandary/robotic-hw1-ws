#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math
import numpy as np

class IMUFilter(Node):
    def __init__(self):
        super().__init__('imu_filter')
        
        self.declare_parameter('lowpass_alpha', 0.2)
        self.declare_parameter('complementary_alpha', 0.98)
        
        self.lowpass_alpha = self.get_parameter('lowpass_alpha').value
        self.complementary_alpha = self.get_parameter('complementary_alpha').value
        
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.bias_samples = []
        self.bias_calibrated = False
        self.calibration_samples = 100
        
        self.prev_accel = np.zeros(3)
        self.prev_gyro = np.zeros(3)
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = None
        
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        
        self.filtered_imu_pub = self.create_publisher(
            Imu, '/imu/filtered', 10)
        
        self.orientation_pub = self.create_publisher(
            Imu, 'estimation/orientation', 10)
        
        self.get_logger().info('IMU Filter node started')
        self.get_logger().info('Collecting bias calibration samples...')

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        
        accel = np.array([msg.linear_acceleration.x,
                         msg.linear_acceleration.y,
                         msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z])
        
        if not self.bias_calibrated:
            self.bias_samples.append((accel.copy(), gyro.copy()))
            if len(self.bias_samples) >= self.calibration_samples:
                accel_samples = np.array([s[0] for s in self.bias_samples])
                gyro_samples = np.array([s[1] for s in self.bias_samples])
                
                self.accel_bias = np.mean(accel_samples, axis=0)
                self.accel_bias[2] -= 9.81
                self.gyro_bias = np.mean(gyro_samples, axis=0)
                
                self.bias_calibrated = True
                self.get_logger().info(f'Bias calibration complete')
                self.get_logger().info(f'Accel bias: {self.accel_bias}')
                self.get_logger().info(f'Gyro bias: {self.gyro_bias}')
            return
        
        accel = accel - self.accel_bias
        gyro = gyro - self.gyro_bias
        
        accel_filtered = self.lowpass_alpha * accel + (1 - self.lowpass_alpha) * self.prev_accel
        gyro_filtered = self.lowpass_alpha * gyro + (1 - self.lowpass_alpha) * self.prev_gyro
        
        self.prev_accel = accel_filtered
        self.prev_gyro = gyro_filtered
        
        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            
            self.roll += gyro_filtered[0] * dt
            self.pitch += gyro_filtered[1] * dt
            self.yaw += gyro_filtered[2] * dt
            
            accel_roll = math.atan2(accel_filtered[1], accel_filtered[2])
            accel_pitch = math.atan2(-accel_filtered[0], 
                                    math.sqrt(accel_filtered[1]**2 + accel_filtered[2]**2))
            
            self.roll = self.complementary_alpha * self.roll + \
                       (1 - self.complementary_alpha) * accel_roll
            self.pitch = self.complementary_alpha * self.pitch + \
                        (1 - self.complementary_alpha) * accel_pitch
        
        self.last_time = current_time
        
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        filtered_msg.header.stamp = current_time.to_msg()
        filtered_msg.linear_acceleration.x = accel_filtered[0]
        filtered_msg.linear_acceleration.y = accel_filtered[1]
        filtered_msg.linear_acceleration.z = accel_filtered[2]
        filtered_msg.angular_velocity.x = gyro_filtered[0]
        filtered_msg.angular_velocity.y = gyro_filtered[1]
        filtered_msg.angular_velocity.z = gyro_filtered[2]
        
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        cp = math.cos(self.pitch * 0.5)
        sp = math.sin(self.pitch * 0.5)
        cr = math.cos(self.roll * 0.5)
        sr = math.sin(self.roll * 0.5)
        
        filtered_msg.orientation.w = cr * cp * cy + sr * sp * sy
        filtered_msg.orientation.x = sr * cp * cy - cr * sp * sy
        filtered_msg.orientation.y = cr * sp * cy + sr * cp * sy
        filtered_msg.orientation.z = cr * cp * sy - sr * sp * cy
        
        self.filtered_imu_pub.publish(filtered_msg)
        self.orientation_pub.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
