#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import asyncio
import websockets
import json
import threading
from scipy.spatial.transform import Rotation
from collections import deque

class MobileIMUReader(Node):
    def __init__(self):
        super().__init__('mobile_imu_reader')
        
        
        self.declare_parameter('sensor_server_ip', '172.27.160.215')
        self.declare_parameter('sensor_server_port', 8080)
        self.declare_parameter('publish_rate', 50.0)
        
        self.server_ip = self.get_parameter('sensor_server_ip').value
        self.server_port = self.get_parameter('sensor_server_port').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        
        self.imu_raw_pub = self.create_publisher(Imu, '/mobile/imu/raw', 10)
        self.imu_filtered_pub = self.create_publisher(Imu, '/mobile/imu/filtered', 10)
        self.path_pub = self.create_publisher(Path, '/mobile/trajectory', 10)
        
        
        self.accel_data = np.array([0.0, 0.0, 0.0])
        self.gyro_data = np.array([0.0, 0.0, 0.0])
        self.mag_data = np.array([0.0, 0.0, 0.0])
        self.data_lock = threading.Lock()
        
        
        self.accel_buffer = deque(maxlen=100)
        self.gyro_buffer = deque(maxlen=100)
        
        
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        
        
        self.alpha_accel = 0.1
        self.alpha_gyro = 0.1
        self.filtered_accel = np.array([0.0, 0.0, 0.0])
        self.filtered_gyro = np.array([0.0, 0.0, 0.0])
        
        
        self.alpha_comp = 0.98
        
        
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])  
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, 0.0])
        
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'
        
        
        self.last_time = None
        self.calibration_samples = 0
        self.is_calibrated = False
        
        
        self.accel_url = f'ws://{self.server_ip}:{self.server_port}/sensor/connect?type=android.sensor.accelerometer'
        self.gyro_url = f'ws://{self.server_ip}:{self.server_port}/sensor/connect?type=android.sensor.gyroscope'
        self.mag_url = f'ws://{self.server_ip}:{self.server_port}/sensor/connect?type=android.sensor.magnetic_field'
        
        
        self.ws_thread = threading.Thread(target=self.run_websocket_loop, daemon=True)
        self.ws_thread.start()
        
        
        self.timer = self.create_timer(1.0/self.publish_rate, self.timer_callback)
        
        self.get_logger().info(f'Mobile IMU Reader initialized')
        self.get_logger().info(f'Connecting to: ws://{self.server_ip}:{self.server_port}')
        self.get_logger().info('Calibrating... Please keep the device still for 2 seconds')

    def run_websocket_loop(self):
        """Run asyncio event loop in separate thread"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.connect_to_sensors())

    async def connect_to_sensors(self):
        """Connect to all sensors via WebSocket"""
        tasks = [
            self.read_sensor(self.accel_url, 'accelerometer'),
            self.read_sensor(self.gyro_url, 'gyroscope'),
            self.read_sensor(self.mag_url, 'magnetometer')
        ]
        await asyncio.gather(*tasks)

    async def read_sensor(self, url, sensor_type):
        """Read data from a specific sensor"""
        while True:
            try:
                async with websockets.connect(url) as websocket:
                    self.get_logger().info(f'Connected to {sensor_type}')
                    
                    async for message in websocket:
                        try:
                            data = json.loads(message)
                            
                            
                            if 'values' in data:
                                values = data['values']
                                
                                with self.data_lock:
                                    if sensor_type == 'accelerometer':
                                        self.accel_data = np.array([
                                            values[0], values[1], values[2]
                                        ])
                                    elif sensor_type == 'gyroscope':
                                        self.gyro_data = np.array([
                                            values[0], values[1], values[2]
                                        ])
                                    elif sensor_type == 'magnetometer':
                                        self.mag_data = np.array([
                                            values[0], values[1], values[2]
                                        ])
                        
                        except json.JSONDecodeError:
                            self.get_logger().warn(f'Failed to parse {sensor_type} data')
                        except Exception as e:
                            self.get_logger().error(f'Error processing {sensor_type}: {str(e)}')
                            
            except Exception as e:
                self.get_logger().error(f'WebSocket error for {sensor_type}: {str(e)}')
                await asyncio.sleep(1)  

    def calibrate_sensors(self, accel, gyro):
        """Calibrate sensors by calculating bias"""
        if self.calibration_samples < 100:
            self.accel_buffer.append(accel)
            self.gyro_buffer.append(gyro)
            self.calibration_samples += 1
            
            if self.calibration_samples == 100:
                self.accel_bias = np.mean(self.accel_buffer, axis=0)
                self.gyro_bias = np.mean(self.gyro_buffer, axis=0)
                
                
                self.accel_bias[2] -= 9.81
                
                self.is_calibrated = True
                self.get_logger().info('Calibration complete!')
                self.get_logger().info(f'Accel bias: {self.accel_bias}')
                self.get_logger().info(f'Gyro bias: {self.gyro_bias}')

    def apply_bias_correction(self, accel, gyro):
        """Apply bias correction"""
        accel_corrected = accel - self.accel_bias
        gyro_corrected = gyro - self.gyro_bias
        return accel_corrected, gyro_corrected

    def low_pass_filter(self, new_value, filtered_value, alpha):
        """Apply low-pass filter"""
        return alpha * new_value + (1 - alpha) * filtered_value

    def complementary_filter(self, accel, gyro, dt):
        """Complementary filter for orientation estimation"""
        
        gyro_quat = self.integrate_gyro(gyro, dt)
        
        
        accel_quat = self.accel_to_quaternion(accel)
        
        
        self.orientation = self.slerp(gyro_quat, accel_quat, 1 - self.alpha_comp)
        
        
        self.orientation = self.orientation / np.linalg.norm(self.orientation)

    def integrate_gyro(self, gyro, dt):
        """Integrate gyroscope to update orientation"""
        wx, wy, wz = gyro
        w, x, y, z = self.orientation
        
        
        dq = 0.5 * np.array([
            -x*wx - y*wy - z*wz,
            w*wx + y*wz - z*wy,
            w*wy - x*wz + z*wx,
            w*wz + x*wy - y*wx
        ])
        
        
        new_quat = self.orientation + dq * dt
        return new_quat / np.linalg.norm(new_quat)

    def accel_to_quaternion(self, accel):
        """Convert accelerometer reading to tilt quaternion"""
        
        accel_norm = accel / np.linalg.norm(accel)
        
        
        roll = np.arctan2(accel_norm[1], accel_norm[2])
        pitch = np.arctan2(-accel_norm[0], 
                          np.sqrt(accel_norm[1]**2 + accel_norm[2]**2))
        
        
        r = Rotation.from_euler('xyz', [roll, pitch, 0])
        quat = r.as_quat()  
        
        return np.array([quat[3], quat[0], quat[1], quat[2]])  

    def slerp(self, q1, q2, t):
        """Spherical linear interpolation between quaternions"""
        dot = np.dot(q1, q2)
        
        if dot < 0:
            q2 = -q2
            dot = -dot
        
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        theta = np.arccos(np.clip(dot, -1, 1))
        sin_theta = np.sin(theta)
        
        w1 = np.sin((1 - t) * theta) / sin_theta
        w2 = np.sin(t * theta) / sin_theta
        
        return w1 * q1 + w2 * q2

    def integrate_acceleration(self, accel, dt):
        """Integrate acceleration to update velocity and position"""
        
        R = Rotation.from_quat([
            self.orientation[1], self.orientation[2], 
            self.orientation[3], self.orientation[0]
        ])
        accel_world = R.apply(accel)
        
        
        accel_world[2] -= 9.81
        
        
        self.velocity += accel_world * dt
        self.position += self.velocity * dt

    def timer_callback(self):
        """Main callback for processing sensor data"""
        import time
        current_time = time.time()
        
        
        with self.data_lock:
            accel = self.accel_data.copy()
            gyro = self.gyro_data.copy()
            mag = self.mag_data.copy()
        
        
        if self.last_time is None:
            self.last_time = current_time
            return
        
        dt = current_time - self.last_time
        self.last_time = current_time
        
        
        if not self.is_calibrated:
            self.calibrate_sensors(accel, gyro)
            return
        
        
        self.publish_raw_imu(accel, gyro, mag)
        
        
        accel_corrected, gyro_corrected = self.apply_bias_correction(accel, gyro)
        
        
        self.filtered_accel = self.low_pass_filter(
            accel_corrected, self.filtered_accel, self.alpha_accel
        )
        self.filtered_gyro = self.low_pass_filter(
            gyro_corrected, self.filtered_gyro, self.alpha_gyro
        )
        
        
        self.complementary_filter(self.filtered_accel, self.filtered_gyro, dt)
        
        
        self.integrate_acceleration(self.filtered_accel, dt)
        
        
        self.publish_filtered_imu(self.filtered_accel, self.filtered_gyro)
        
        
        self.publish_path()

    def publish_raw_imu(self, accel, gyro, mag):
        """Publish raw IMU data"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mobile_imu'
        
        msg.linear_acceleration.x = float(accel[0])
        msg.linear_acceleration.y = float(accel[1])
        msg.linear_acceleration.z = float(accel[2])
        
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])
        
        self.imu_raw_pub.publish(msg)

    def publish_filtered_imu(self, accel, gyro):
        """Publish filtered IMU data with orientation"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mobile_imu'
        
        msg.orientation.w = float(self.orientation[0])
        msg.orientation.x = float(self.orientation[1])
        msg.orientation.y = float(self.orientation[2])
        msg.orientation.z = float(self.orientation[3])
        
        msg.linear_acceleration.x = float(accel[0])
        msg.linear_acceleration.y = float(accel[1])
        msg.linear_acceleration.z = float(accel[2])
        
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])
        
        self.imu_filtered_pub.publish(msg)

    def publish_path(self):
        """Publish trajectory path"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        
        pose.pose.position.x = float(self.position[0])
        pose.pose.position.y = float(self.position[1])
        pose.pose.position.z = float(self.position[2])
        
        pose.pose.orientation.w = float(self.orientation[0])
        pose.pose.orientation.x = float(self.orientation[1])
        pose.pose.orientation.y = float(self.orientation[2])
        pose.pose.orientation.z = float(self.orientation[3])
        
        self.path_msg.poses.append(pose)
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        
        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MobileIMUReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
