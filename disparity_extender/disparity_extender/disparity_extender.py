#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np


class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap')        
        self.disparity_thresh = 2.0
        self.bubble_size = 0.3
        self.max_speed = 1.0
        self.min_speed = 0.0
        self.max_steering = 1.0
        
        self.scan_sub = self.create_subscription(LaserScan, '/autodrive/f1tenth_1/lidar', self.scan_callback, 10)
        self.steering_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.throttle_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)
        
    def preprocess(self, ranges):
        """Disparity extender + safety bubble"""
        for i in range(len(ranges)-1):
            if ranges[i+1] - ranges[i] > self.disparity_thresh:
                bubble_idx = int(self.bubble_size * len(ranges) / (ranges[i] * np.pi))
                for j in range(i+1, i+2+bubble_idx):
                    ranges[j] = ranges[i]
            elif ranges[i] - ranges[i+1] > self.disparity_thresh:
                bubble_idx = int(self.bubble_size * len(ranges) / (ranges[i+1] * np.pi))
                for j in range(i-bubble_idx, i+1):
                    ranges[j] = ranges[i+1]
    
    def find_furthest(self, ranges):
        """Find furthest point"""
        return np.argmax(ranges)
    
    def scan_callback(self, msg):
        self.preprocess(msg.ranges)
        
        best_idx = np.argmax(msg.ranges)
        
        steering = np.clip(
            msg.angle_min + best_idx * msg.angle_increment,
            -self.max_steering,
            self.max_steering
        )
        
        speed = self.min_speed + (self.max_speed - self.min_speed) * (
            1.0 - abs(steering) / self.max_steering
        )
        
        steering_msg = Float32()
        steering_msg.data = float(steering)
        self.steering_pub.publish(steering_msg)
        
        throttle_msg = Float32()
        throttle_msg.data = float(speed)
        self.throttle_pub.publish(throttle_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()