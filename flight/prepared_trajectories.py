"""
Contiains off-the-shelf trajectories accounting for scale.
Author: S. Pfeiffer, MAVLab
"""

import random

from flight.trajectories import *


def hover(x0, y0, altitude):
    setpoints = []
    for _ in range(9):
        setpoints += takeoff(x0, y0, altitude, 0.0)
    return setpoints


def square(x0, y0, side_length, altitude):
    setpoints = xy_square(x0, y0, side_length, altitude, 0.0)
    return setpoints


def octagon(x0, y0, radius, altitude):
    setpoints = xy_polygon(x0, y0, 8, radius, altitude, 0.0)
    return setpoints


def triangle(x0, y0, radius, altitude):
    setpoints = xy_polygon(x0, y0, 3, radius, altitude, 0.0)
    return setpoints


def hourglass(x0, y0, side_length, altitude):
    setpoints = xy_hourglass(x0, y0, side_length, altitude, 0.0)
    return setpoints


def star(x0, y0, radius, altitude):
    polypoints = xy_polygon(x0, y0, 5, radius, altitude, 0.0)
    setpoints = []
    setpoints.append(polypoints[0])
    setpoints.append(polypoints[2])
    setpoints.append(polypoints[4])
    setpoints.append(polypoints[1])
    setpoints.append(polypoints[3])
    setpoints.append(polypoints[0])
    return setpoints

def randoms(x0, y0, x_bound, y_bound, altitude):
    setpoints = []
    points = 10
    for i in range(points):
        x = random.uniform(*x_bound)
        y = random.uniform(*y_bound)
        setpoints.append((x, y, altitude, 0.0))
    return setpoints


def scan(x0, y0, x_bound, y_bound, altitude):
    setpoints = scan_area(x_bound, y_bound, 0.5, altitude, 0.0)
    return setpoints
