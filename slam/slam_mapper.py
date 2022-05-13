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
import shapely.geometry as geom
import shapely.affinity as aff
import descartes as dc
from differential_drive import DifferentialDrive

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
delta_d = 0

"""
Lidar variables
"""
max_dist = 0.5
# minimum distances b/w consecutive lidar sweeps to be considered as an anomaly
min_anomaly_distance = 0.5 * max_dist
angle_interval = np.pi / 19
lidar_uncertainty = 0.01

"""
Robot Dimension Variables
"""
robot_width = 0.1
robot_length = 0.2
robot_radius = 0.05
time_step = 0.02
robot1 = dd.DifferentialDrive(robot_width, robot_length, robot_radius)


"""
Mapping Variables
"""
landmarks_repo = []
predicted_path_points = []
actual_path_points = []

def create_ellipse(center: tuple, axes_lengths:tuple, angle:float):
    """
    Creates a Shapely Ellipse with the specified params
    """
    # create a cricle first
    circ = geom.Point(center).buffer(1)

    # create the ellipse along x and y:
    ell = aff.scale(circ, int(axes_lengths[0]), int(axes_lengths[1]))

    # rotate the ellipse (clockwise, x axis pointing right):
    ellr = aff.rotate(ell, angle)

    return ellr

def parametric_point_locator(state: list, angle:float, distance:float) -> list:
    """
    Finds the point "distance" units away from the current position(state)
    """
    return [state[0]+ distance*math.cos(state[2]+angle), state[1]+ distance*math.sin(state[2]+angle)]

def identify_landmark(predicted_state, lidar_distances: np.array) -> list:
    """
    Detects for anomalies.
    An anomaly is a change in distance between 2 consecutive lidar readings
    that is less than <min_anomaly_distance>
    """
    anomaly_indices = []
    landmark_shapes = []

    # scan the distances received for an anomaly between two consecutive sweeps
    # and keep track of the indices
    for index in range(1, len(lidar_distances)):
        dist = abs(lidar_distances[index - 1] - lidar_distances[index])
        if dist > min_anomaly_distance:
            anomaly_indices.append(index-1)

    for anomaly in anomaly_indices:
        center = parametric_point_locator(predicted_state, anomaly*angle_interval,lidar_distances[anomaly])
        # Error Ellipse Representation
        landmark_shapes.append([
            create_ellipse(
                center=tuple(center),
                axes_lengths=(lidar_uncertainty, max_dist * math.sin(math.pi/20)),
                angle=anomaly*angle_interval),
            lidar_distances[anomaly],
            anomaly*angle_interval
        ])

    return landmark_shapes

def refactor_landmarks(new_landmarks:list, landmarks:list, delta_d:float):
    """
    Compares landmarks between consecutive sweeps. If the new sweep
    encounters landmarks that are relatively close to the old ones,
    then they must be the same landmark and hence it averages out the
    position from the new and old and refactors it back into the landmark list
    """
    size = len(new_landmarks)

    if len(landmarks) != 0:
        for new_land in new_landmarks:
            for index, old_land in enumerate(landmarks[-size:]): #iterate over the last 'size' # of landamrks
                if (new_land[0].centroid).distance(old_land[0].centroid) < delta_d:
                    # new_center = (new_land[0].centroid[0].x + old_land[0].centroid[0].x)/2, \
                    #              (new_land[0].centroid[1].y + old_land[0].centroid[1].y)/2
                    i = new_land[0].intersection(old_land[0])
                    maj_axis = max(i.distance(i))
                    min_axis = min(i.distance(i))
                    new_center = i.centroid.xy
                    angle = float(np.mean([old_land[2], new_land[2]])[0])
                    landmarks[index] = [
                        create_ellipse(
                            center=tuple(new_center),
                            axes_lengths=(maj_axis, min_axis),
                            angle=angle),
                        (new_land[1]+old_land[1])/2,
                        angle
                    ]
                else:
                    landmarks.append(new_land)
        return landmarks
    else:
        return [x for x in new_landmarks]

"""
Detects drift and locates the robots from the the sensors
If a newly detected landmark is within <distance_step> of a previously detected
landmark, its probably the same landmark but the robot has drifted. Using this 
information of drift, the "sensed_state" is calculated as an alternative to 
the predicted_state obtained from just the physics
"""
def grab_sensed_state(landmarks:list, size_of_latest_found_landmarks):

    new_sensed_state = [0,0,0]
    size_of_latest_found_landmarks = 3 if size_of_latest_found_landmarks > 3 else size_of_latest_found_landmarks

    # we dont know the actual angle, so append a 0.
    for i in range(0,size_of_latest_found_landmarks):
        state = [landmarks[-1+i][0].centroid.x, landmarks[-1+i][0].centroid.y, 0]
        state = parametric_point_locator(state, math.pi+landmarks[-1+i][2], landmarks[-1+i][1])
        state.append(landmarks[-1+i][2])
        new_sensed_state.append(state)

    # average the sensed state value over the last found landmarks
    new_sensed_state = [np.mean(new_sensed_state[:][0]),
                        np.mean(new_sensed_state[:][1]),
                        np.mean(new_sensed_state[:][2])]

    return list(new_sensed_state)

def filter_kalman():
    delta_t = 0.1
    N = int(10 / delta_t)
    t = range(0, 10)  # create iterations with delta_t = 0.1
    x = np.zeros(2, N)
    z = np.zeros(2, N)
    var1 = 1
    var2 = 1

    # Filter Variables
    H = np.matrix(([1,0,0],
                  [0,1,0],
                  [0,0,0]))
    HT = np.transposee(H)

    sigma = 1 # standard deviation of noise assuming its normally distributed
    V = np.matrix(([sigma**2, 0],
                   [0, sigma**2]))

    W = np.matrix([[var2, 0], [0, 0]])
    P = np.zeros(2, 2)  # initial covariance with 0 values
    x_est = np.zeros(2, N)  # initially start at 0

while True:
    topic, message = sub_socket.recv_multipart()
    omega1,omega2 = 0, 0

    if topic == b"lidar":
        message_dict = json.loads(message.decode())
        lidar_dists = message_dict["distances"]

        min_index = np.argmin(lidar_dists)
        min_angle = -np.pi / 2 + min_index * angle_interval
        min_dist = lidar_dists[min_index]

        if count == 1:
            print("here")

        newfound_landmarks = identify_landmark(predicted_state, lidar_dists)
        if len(newfound_landmarks) != 0:
            landmarks_repo = refactor_landmarks(newfound_landmarks, landmarks_repo, delta_d)
        if len(newfound_landmarks) != 0:
            sensed_state = grab_sensed_state(landmarks_repo, len(newfound_landmarks))
        else:
            sensed_state = predicted_state

        if sensed_state is []:
            sensed_state = predicted_state

        # TODO Insert EKF prediction for actual state here using the above calculated states


        # general robot navigational metrics
        if min_dist < 0.8 * max_dist: # get too close to an object, rotate a little CC.
            k = -10 * np.copysign(np.exp(-np.abs(min_angle)), min_angle)
            print(k)
            if not np.isfinite(k):
                k = 0
        else:
            k = 0

        omega1 = (s + w * k) / (2 * r)
        omega2 = (s - w * k) / (2 * r)

        wheel_speeds = {"omega1": omega1, "omega2": omega2}
        pub_socket.send_multipart([b"wheel_speeds", json.dumps(wheel_speeds).encode()])

        delta_d = predicted_state

        predicted_state = list((solve_ivp(robot1.deriv, [0, time_step],
                    predicted_state, args=[[omega1, omega2], 0])).y[:,-1])

        delta_d = math.sqrt((delta_d[0] - predicted_state[0])**2 + (delta_d[1] - predicted_state[1])**2)

        predicted_path_points.append(predicted_state)
        actual_path_points.append(sensed_state)

        # print(count)
        count += 1

        if count % 100 == 0:
            plt.plot(list(x[0] for x in predicted_path_points), list(y[1] for y in predicted_path_points), color='red')
            plt.plot(list(x[0] for x in actual_path_points), list(y[1] for y in actual_path_points), color= 'green')
            plt.scatter(list(x[0].centroid.x for x in landmarks_repo), list(y[0].centroid.y for y in landmarks_repo))
            plt.show()




