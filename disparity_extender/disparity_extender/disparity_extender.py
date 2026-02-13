
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap')        
        self.disparity_thresh = 2.0
        self.bubble_size = 1.0
        self.max_speed = 0.5
        self.min_speed = 0.3
        self.max_steering = 1.0
        
        self.scan_sub = self.create_subscription(LaserScan, '/autodrive/f1tenth_1/lidar', self.scan_callback, 10)
        self.steering_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.throttle_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)
    
    def preprocess(self, ranges):
        """Disparity extender + safety bubble"""
        proc = np.array(ranges, dtype=float)  
        proc[np.isinf(proc)] = 100.0  
        
        
        for i in range(len(proc)-1):
            if proc[i+1] - proc[i] > self.disparity_thresh:
                bubble_idx = int(self.bubble_size * len(proc) / (proc[i] * np.pi))
                end = min(i + 2 + bubble_idx, len(proc))  
                proc[i+1:end] = proc[i]
            elif proc[i] - proc[i+1] > self.disparity_thresh:
                bubble_idx = int(self.bubble_size * len(proc) / (proc[i+1] * np.pi))
                start = max(i - bubble_idx, 0)  
                proc[start:i+1] = proc[i+1]
        
        
        return proc
    
    def scan_callback(self, msg):
        proc_ranges = self.preprocess(msg.ranges)
        
        # Only look in front 120 degrees
        mid = len(proc_ranges) // 2
        window = len(proc_ranges) // 3
        front_ranges = proc_ranges[mid-window:mid+window]
        
        best_idx = mid - window + np.argmax(front_ranges)
        
        steering = np.clip(
            msg.angle_min + best_idx * msg.angle_increment,
            -self.max_steering,
            self.max_steering
        )
        
        speed = self.max_speed * (1.0 - 0.7 * abs(steering) / self.max_steering)
        
        self.steering_pub.publish(Float32(data=float(steering)))
        self.throttle_pub.publish(Float32(data=float(speed)))

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