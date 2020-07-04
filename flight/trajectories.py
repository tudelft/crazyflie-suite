"""
Provides functions that return trajectories in form of a list of setpoints.
Author: S. Pfeiffer, MAVLab
"""

import numpy as np


def takeoff(x, y, altitude, yaw):
    setpoints = [(x, y, altitude, yaw)]

    return setpoints


def landing(x, y, altitude, yaw):
    setpoints = [(x, y, altitude, yaw), (x, y, 0, yaw)]

    return setpoints


def scan_area(x_bound, y_bound, spacing, altitude, yaw):
    x_min, x_max = x_bound
    y_min, y_max = y_bound

    setpoints = []
    x_range = x_max - x_min
    n_lines = int(np.floor(x_range / spacing) + 1)
    y_is_min = True

    for i in range(n_lines):
        x_line = x_min + i * spacing
        y = y_min if y_is_min else y_max
        setpoints.append((x_line, y, altitude, yaw))
        y_is_min = not (y_is_min)
        y = y_min if y_is_min else y_max
        setpoints.append((x_line, y, altitude, yaw))

    return setpoints


def xy_square(x0, y0, side_length, altitude, yaw):
    x_min = x0 - side_length / 2
    x_max = x0 + side_length / 2
    y_min = y0 - side_length / 2
    y_max = y0 + side_length / 2

    setpoints = [
        (x_min, y_min, altitude, yaw),
        (x_min, y_max, altitude, yaw),
        (x_max, y_max, altitude, yaw),
        (x_max, y_min, altitude, yaw),
        (x_min, y_min, altitude, yaw),
    ]

    return setpoints


def xy_hourglass(x0, y0, side_length, altitude, yaw):
    x_min = x0 - side_length / 2
    x_max = x0 + side_length / 2
    y_min = y0 - side_length / 2
    y_max = y0 + side_length / 2

    setpoints = [
        (x_max, y_max, altitude, yaw),
        (x_max, y_min, altitude, yaw),
        (x_min, y_max, altitude, yaw),
        (x_min, y_min, altitude, yaw),
    ]

    return setpoints


def xy_polygon(x0, y0, N, radius, altitude, yaw):
    setpoints = []
    for i in range(N):
        angle = 2 * np.pi * i / N
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        setpoints.append((x0 + x, y0 + y, altitude, yaw))
    setpoints.append(setpoints[0])

    return setpoints
