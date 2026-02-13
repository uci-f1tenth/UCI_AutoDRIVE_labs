#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np


class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap')
        
        # Parameters
        self.declare_parameters('', [
            ('disparity_threshold', 0.5),
            ('bubble_size', 0.3),
            ('car_width', 0.2),
            ('max_speed', 5.0),
            ('min_speed', 1.5),
            ('max_steering', 0.35),
        ])
        
        self.disparity_thresh = self.get_parameter('disparity_threshold').value
        self.bubble_size = self.get_parameter('bubble_size').value
        self.car_width = self.get_parameter('car_width').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_steering = self.get_parameter('max_steering').value
        
        self.scan_sub = self.create_subscription(LaserScan, '/autodrive/f1tenth_1/lidar', self.scan_callback, 10)
        self.steering_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.throttle_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)
        
        self.angle_min = None
        self.angle_inc = None
    
    def preprocess(self, ranges):
        """Disparity extender + safety bubble"""
        proc = np.array(ranges)
        proc[np.isinf(proc)] = 100.0
        
        # Disparity extender
        disparities = np.where(np.abs(np.diff(proc)) > self.disparity_thresh)[0]
        for d in disparities:
            if np.isinf(proc[d]) or np.isinf(proc[d + 1]):
                continue
            
            closer = min(proc[d], proc[d + 1])
            bubble_idx = int(self.bubble_size * len(proc) / (closer * np.pi))
            
            if proc[d] < proc[d + 1]:
                end = min(d + bubble_idx + 2, len(proc))
                proc[d + 1:end] = proc[d]
            else:
                start = max(d - bubble_idx, 0)
                proc[start:d + 1] = proc[d + 1]
        
        # Safety bubble
        proc[proc < self.car_width] = 0.0
        closest = np.argmin(proc)
        if proc[closest] > 0:
            bubble_idx = int(self.car_width * len(proc) / (proc[closest] * 2 * np.pi))
            start = max(0, closest - bubble_idx)
            end = min(len(proc), closest + bubble_idx + 1)
            proc[start:end] = 0.0
        
        return proc
    
    def find_gap(self, ranges):
        """Find largest gap"""
        non_zero = ranges > 0
        gaps = []
        start = None
        
        for i, valid in enumerate(non_zero):
            if valid and start is None:
                start = i
            elif not valid and start is not None:
                gaps.append((start, i - 1))
                start = None
        
        if start is not None:
            gaps.append((start, len(ranges) - 1))
        
        if not gaps:
            return len(ranges) // 2, len(ranges) // 2
        
        return max(gaps, key=lambda g: g[1] - g[0])
    
    def scan_callback(self, msg):
        if self.angle_min is None:
            self.angle_min = msg.angle_min
            self.angle_inc = msg.angle_increment
        
        # Process scan
        proc_ranges = self.preprocess(msg.ranges)
        
        # Find gap and best point
        gap_start, gap_end = self.find_gap(proc_ranges)
        best_idx = gap_start + np.argmax(proc_ranges[gap_start:gap_end + 1])
        
        # Calculate steering
        steering = np.clip(
            self.angle_min + best_idx * self.angle_inc,
            -self.max_steering,
            self.max_steering
        )
        
        # Calculate speed (slower for sharp turns)
        speed = self.min_speed + (self.max_speed - self.min_speed) * (
            1.0 - abs(steering) / self.max_steering
        )
        
        # Publish
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