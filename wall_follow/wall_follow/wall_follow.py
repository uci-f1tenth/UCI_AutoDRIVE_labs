import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
from dataclasses import dataclass
from typing import Optional, Any


@dataclass
class PID:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    setpoint: float = 0.0
    out_limits: tuple[Optional[float], Optional[float]] = (-1.0, 1.0)
    integral: float = 0.0
    previous_error: Optional[float] = None

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = None

    def update(self, measurement: float, dt: Optional[float]) -> float:
        error: float = self.setpoint - measurement
        if dt and dt > 0.0:
            self.integral += error * dt
        derivative: float = (
            (error - self.previous_error) / dt
            if dt and dt > 0.0 and self.previous_error is not None
            else 0.0
        )
        u: float = self.kp * error + self.ki * self.integral + self.kd * derivative
        low, high = self.out_limits
        if low is not None and u < low:
            u = low
        if high is not None and u > high:
            u = high
        self.previous_error = error
        return u


def get_range(
    lidar_range_array: np.ndarray,
    angle: float,
    min_angle: float = -3.0 * np.pi / 4.0,
    max_angle: float = 3.0 * np.pi / 4.0,
):
    angle_increment = (max_angle - min_angle) / (lidar_range_array.size - 1)
    index = round((angle - min_angle) / angle_increment)
    if lidar_range_array is not None and index < lidar_range_array.size:
        return lidar_range_array[index]
    else:
        return 0.0


def get_distance_from_wall(lidar_range_array: np.ndarray):
    theta = np.radians(45.0)
    b = get_range(lidar_range_array, np.radians(90))
    a = get_range(lidar_range_array, np.radians(45))
    alpha = np.arctan2(a * np.cos(theta) - b, a * np.sin(theta))
    D = b * np.cos(alpha)
    error = D + 1.5 * np.sin(alpha)

    return error


class WallFollow(Node):
    def __init__(self):
        super().__init__("wall_follow")
        self.steering_pid = PID(kp=1.0, kd=0.0, ki=0, setpoint=-0.6)
        self.scan_sub = self.create_subscription(
            LaserScan, "/autodrive/f1tenth_1/lidar", self.scan_callback, 10
        )
        self.steering_pub = self.create_publisher(
            Float32, "/autodrive/f1tenth_1/steering_command", 10
        )
        self.throttle_pub = self.create_publisher(
            Float32, "/autodrive/f1tenth_1/throttle_command", 10
        )

    def scan_callback(self, msg):
        print("test")
        speed = 0.2
        steering = self.steering_pid.update(
            -get_distance_from_wall(np.array(msg.ranges)),
            msg.time_increment,
        )
        self.steering_pub.publish(Float32(data=float(steering)))
        self.throttle_pub.publish(Float32(data=float(speed)))


# # Initialize vehicle(s)
# f1tenth_1 = autodrive.F1TENTH()
# f1tenth_1.id = "V1"
# steering_pid = PID(kp=1.0, kd=0.0, ki=0, setpoint=-0.6)

# # Initialize the server
# sio = socketio.Server()

# # Flask (web) app
# app = Flask(__name__)  # '__main__'


# # Registering "connect" event handler for the server
# @sio.on("connect")
# def connect(sid, environ):
#     print("Connected!")


# # Registering "Bridge" event handler for the server
# @sio.on("Bridge")
# def bridge(sid, data):
#     if data:
#         f1tenth_1.parse_data(data)

#         f1tenth_1.throttle_command = 0.2  # [-1, 1]

#         f1tenth_1.steering_command = steering_pid.update(
#             -get_distance_from_wall(f1tenth_1.lidar_range_array),
#             1 / f1tenth_1.lidar_scan_rate,
#         )

#         ########################################################################

#         json_msg = f1tenth_1.generate_commands()

#         try:
#             sio.emit("Bridge", data=json_msg)
#         except Exception as exception_instance:
#             print(exception_instance)


# ################################################################################

# if __name__ == "__main__":
#     app = socketio.Middleware(
#         sio, app
#     )  # Wrap flask application with socketio's middleware
#     eventlet.wsgi.server(
#         eventlet.listen(("", 4567)), app
#     )  # Deploy as an eventlet WSGI server


def main(args=None):
    rclpy.init(args=args)

    wall_follow = WallFollow()

    try:
        rclpy.spin(wall_follow)
    except KeyboardInterrupt:
        pass
    finally:
        wall_follow.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
