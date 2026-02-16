#!/usr/bin/env python

# Import libraries
from typing import Any
import numpy as np
import socketio
import eventlet
from flask import Flask
import autodrive

################################################################################

# Initialize vehicle(s)
f1tenth_1 = autodrive.F1TENTH()
f1tenth_1.id = "V1"
min_angle: float = -np.pi / 2.0  # radians
max_angle: float = np.pi / 2.0  # radians
bubble_size: float = 120  # lidar points

# Initialize the server
sio = socketio.Server()

# Flask (web) app
app = Flask(__name__)  # '__main__'


# Registering "connect" event handler for the server
@sio.on("connect")
def connect(sid, environ):
    print("Connected!")


def index_to_angle(index: int, num_points: int) -> float:
    angle_increment = (max_angle - min_angle) / (num_points - 1)
    angle = min_angle + index * angle_increment
    return angle


def find_best_point(lidar_range_array: np.ndarray[Any]) -> int:
    best_index = 0
    best_min_distance = 0.0

    for i in range(len(lidar_range_array) - bubble_size + 1):
        window = lidar_range_array[i : i + bubble_size]
        min_distance = np.min(window)

        if min_distance > best_min_distance:
            best_min_distance = min_distance
            best_index = i

    return best_index + bubble_size // 2


def compute_speed(target_distance: float) -> float:
    if target_distance < 6.0:
        return target_distance / 6.0
    return 1.0


# Registering "Bridge" event handler for the server
@sio.on("Bridge")
def bridge(sid, data):
    if data:
        f1tenth_1.parse_data(data)

        lidar_range_array = f1tenth_1.lidar_range_array[
            f1tenth_1.lidar_range_array.size // 6 : -f1tenth_1.lidar_range_array.size
            // 6
        ]

        best_point_index = find_best_point(lidar_range_array)

        best_point_angle = index_to_angle(best_point_index, lidar_range_array.size)
        f1tenth_1.steering_command = best_point_angle / (np.pi / 2.0)

        target_distance = lidar_range_array[best_point_index]
        f1tenth_1.throttle_command = compute_speed(target_distance)

        ########################################################################

        json_msg = f1tenth_1.generate_commands()  # Generate vehicle 1 message

        try:
            sio.emit("Bridge", data=json_msg)
        except Exception as exception_instance:
            print(exception_instance)


################################################################################

if __name__ == "__main__":
    app = socketio.Middleware(
        sio, app
    )  # Wrap flask application with socketio's middleware
    eventlet.wsgi.server(
        eventlet.listen(("", 4567)), app
    )  # Deploy as an eventlet WSGI server
