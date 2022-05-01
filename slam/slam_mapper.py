"""
Created on 05/01/2022

@author: SherwynBraganza31
"""

import zmq
import math
import json
import numpy as np
import matplotlib.animation as anim
import matplotlib.pyplot as plt
import differential_drive as dd

"""
ZMQ Initialization
"""
context = zmq.Context()
pub_socket = context.socket(zmq.PUB)
# pub_socket.connect("ipc:///tmp/robo_sim/pub.ipc")
pub_socket.connect("tcp://localhost:5557")

sub_socket = context.socket(zmq.SUB)
# sub_socket.connect("ipc:///tmp/robo_sim/sub.ipc")
sub_socket.connect("tcp://localhost:5555")

"""
State Variables Initialization
"""
s = 10
k = 0.0
r = 0.5
w = 1
k_dir = 1
count = 0
start = (0,0,0)
current_orientation = start

"""
Lidar variables
"""
max_dist = 0.5
# minimum distances b/w consecutive lidar sweeps to be considered as an anomaly
min_anomaly_distance = 0.3 * max_dist
angle_interval = np.pi / 19

"""
Robot Dimension Variables
"""
robot_width = 0.1
robot_length = 0.2
robot_radius = 0.05
robot1 = dd.DifferentialDrive(robot_width, robot_length, robot_radius)

"""
Finds the point "distance" units away from the current position(state) 
"""
def parametric_point_locator(state: list, angle:float, distance:float) -> list:
    return [ [state[0]+ distance*math.cos(state[3]+angle)], [state[1]+ distance*math.sin(state[3]+angle)] ]

"""
Detects for anomalies.

An anomaly is a change in distance between 2 consecutive lidar readings 
that is less than <min_anomaly_distance>
"""
def identify_landmark(lidar_distances: np.array, state: list) -> list:
    anomaly_indices = []
    landmark_coods = []

    for index in range(1, len(lidar_distances)):
        if abs(lidar_distances[index-1] - lidar_distances[index]) < min_anomaly_distance:
            anomaly_indices.append(index)

    for anomaly in anomaly_indices:
        landmark_coods.append(parametric_point_locator(state,
                                                       anomaly*angle_interval,
                                                       lidar_distances[anomaly]))
    return landmark_coods


while True:
    # if k > 1:
    #     k_dir = -1
    # elif k < -1:
    #     k_dir = 1
    # k += k_dir*0.01
    topic, message = sub_socket.recv_multipart()
    # print(topic, ":", json.loads(message.decode()))

    if topic == b"lidar":
        message_dict = json.loads(message.decode())
        lidar_dists = message_dict["distances"]

        min_index = np.argmin(lidar_dists)
        min_angle = -np.pi / 2 + min_index * angle_interval
        min_dist = lidar_dists[min_index]

        if min_dist < 0.8 * max_dist:
            k = -10 * np.copysign(np.exp(-np.abs(min_angle)), min_angle)
            print(k)
            if not np.isfinite(k):
                k = 0
        else:
            k = 0

        # print("k: {}, s: {}".format(k, s))

        omega1 = (s + w * k) / (2 * r)
        omega2 = (s - w * k) / (2 * r)

        wheel_speeds = {"omega1": omega1, "omega2": omega2}
        pub_socket.send_multipart(
            [b"wheel_speeds", json.dumps(wheel_speeds).encode()])
        # print(count)
        count += 1





