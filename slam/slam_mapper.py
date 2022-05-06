#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
from scipy.integrate import solve_ivp
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

sub_socket.setsockopt(zmq.SUBSCRIBE, b"state")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"collision")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"lidar")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"landmarks")

"""
State Variables Initialization
"""
s = 10
k = 0.0
r = 0.5
w = 1
k_dir = 1
count = 0
start = [0,0,0]
predicted_state = start
sensed_state = start
actual_state = start

"""
Lidar variables
"""
max_dist = 0.5
# minimum distances b/w consecutive lidar sweeps to be considered as an anomaly
min_anomaly_distance = 0.5 * max_dist
angle_interval = np.pi / 19

"""
Robot Dimension Variables
"""
robot_width = 0.1
robot_length = 0.2
robot_radius = 0.05
robot1 = dd.DifferentialDrive(robot_width, robot_length, robot_radius)
time_step = 0.02

"""
Mapping Variables
"""
landmarks = []
predicted_path_points = []
actual_path_points = []

"""
Finds the point "distance" units away from the current position(state) 
"""
def parametric_point_locator(state: list, angle:float, distance:float) -> list:
    return [state[0]+ distance*math.cos(state[2]+angle), state[1]+ distance*math.sin(state[2]+angle)]

"""
Detects for anomalies.

An anomaly is a change in distance between 2 consecutive lidar readings 
that is less than <min_anomaly_distance>
"""
def identify_landmark(lidar_distances: np.array, state: list) -> list:
    anomaly_indices = []
    landmark_coods = []

    for index in range(1, len(lidar_distances)):
        if abs(lidar_distances[index-1] - lidar_distances[index]) > min_anomaly_distance:
            anomaly_indices.append(index)

    for anomaly in anomaly_indices:
        landmark_coods.append(parametric_point_locator(state,
                                                       anomaly*angle_interval,
                                                       lidar_distances[anomaly]))
    return landmark_coods

"""
Detects drift and locates the robots from the the sensors

If a newly detected landmark is within <distance_step> of a previously detected
landmark, its probably the same landmark but the robot has drifted. Using this 
information of drift, the "sensed_state" is calculated as an alternative to 
the predicted_state obtained from just the physics
"""
def grab_sensed_state(new_landmarks: list, predicted_state, sensed_state, distance_step = 0.1 * min_anomaly_distance):
    new_sensed_state = sensed_state
    for x in new_landmarks:
        for old in landmarks[-4:-1]:
            if math.sqrt((x[0] - old[0])**2 + (x[1] - old[1])**2) > distance_step:
                drift = [ x[0] - old[0],
                          x[1] - old[1] ]
                new_sensed_state = [ new_sensed_state[0]/2 + (predicted_state[0] + drift[0])/2,
                                 new_sensed_state[1]/2 + (predicted_state[1] + drift[1])/2 ]

    return new_sensed_state


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

        newfound_landmarks = identify_landmark(lidar_dists, predicted_state)
        sensed_state = grab_sensed_state(newfound_landmarks, sensed_state=sensed_state,
                                         predicted_state=predicted_state)

        # TODO Insert EKF prediction for actual state here using the above calculated states

        # general robot navigational metrics
        if min_dist < 0.8 * max_dist: # get too close to an object, rotate a little CC.
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

        predicted_state = list((solve_ivp(robot1.deriv, [0, time_step],
                    predicted_state, args=[[omega1, omega2], 0])).y[:,-1])

        landmarks += newfound_landmarks

        actual_path_points.append(sensed_state)
        # print(count)
        count += 1

        if count % 100 == 0:
            plt.plot(list(x[0] for x in predicted_path_points), list(y[1] for y in predicted_path_points))
            plt.plot(list(x[0] for x in actual_path_points), list(y[1] for y in actual_path_points))
            plt.scatter(list(x[0] for x in actual_path_points), list(y[1] for y in actual_path_points))
            plt.show()




