#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarFrameTransformer(Node):
    def __init__(self):
        super().__init__('lidar_frame_transformer')
        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan_raw',
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        
        self.get_logger().info('LiDAR Frame Transformer Node Started')

    def listener_callback(self, msg):
        transformed_msg = LaserScan()
        transformed_msg.header = msg.header
        transformed_msg.header.frame_id = 'lidar_link'
        transformed_msg.header.stamp = self.get_clock().now().to_msg()
        
        transformed_msg.angle_min = msg.angle_min
        transformed_msg.angle_max = msg.angle_max
        transformed_msg.angle_increment = msg.angle_increment
        transformed_msg.time_increment = msg.time_increment
        transformed_msg.scan_time = msg.scan_time
        transformed_msg.range_min = msg.range_min
        transformed_msg.range_max = msg.range_max
        transformed_msg.ranges = msg.ranges
        transformed_msg.intensities = msg.intensities
        
        self.publisher.publish(transformed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarFrameTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
