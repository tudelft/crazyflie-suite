"""
Contains custom utility functions used in the Python scripts to control the Crazyflie.
Author: S. Pfeiffer, MAVLab
"""

import math
import numpy as np
import numpy.linalg as npl

RAD2DEG = 180 / math.pi


def quat2euler(q):
    # Convert OptiTrack quaternions into Crazyflie Euler angles (degrees)
    q = q / npl.norm(q)
    pitch = RAD2DEG * math.atan2(
        -2 * (q[1] * q[2] - q[0] * q[3]), q[0] ** 2 - q[1] ** 2 + q[2] ** 2 - q[3] ** 2
    )
    roll = RAD2DEG * math.asin(2 * (q[2] * q[3] + q[0] * q[1]))
    yaw = RAD2DEG * (
        -math.atan2(
            -2 * (q[1] * q[3] - q[0] * q[2]),
            q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2,
        )
    )

    if pitch > 0:
        pitch = pitch - 180
    else:
        pitch = pitch + 180

    eulerAngles = np.array([roll, pitch, yaw])

    return eulerAngles


def ot2ned(vector_3d_ot):
    # Convert vector from OptiTrack coordinates to NED
    vector_3d_ned = np.zeros(3)
    vector_3d_ned[0] = vector_3d_ot[0]  # NED.x = OT.x
    vector_3d_ned[1] = vector_3d_ot[2]  # NED.y = OT.z
    vector_3d_ned[2] = -vector_3d_ot[1]  # NED.z = -OT.y

    return vector_3d_ned


def ot2control(vector_3d_ot):
    # Convert vector from OptiTrack coordinates to Crazyflie control coordinates
    vector_3d_ctrl = np.zeros(3)
    vector_3d_ctrl[0] = vector_3d_ot[2]  # CONTROL.x = OT.z
    vector_3d_ctrl[1] = vector_3d_ot[0]  # CONTROL.y = OT.x
    vector_3d_ctrl[2] = vector_3d_ot[1]  # CONTROL.z = OT.y

    return vector_3d_ctrl

def ot2control_quat(quaternion_4d_ot):
    quaternion_4d_ctrl = np.zeros(4)
    quaternion_4d_ctrl[0] = quaternion_4d_ot[2] # CONTROL.x = OT.z
    quaternion_4d_ctrl[1] = quaternion_4d_ot[0] # CONTROL.y = OT.x
    quaternion_4d_ctrl[2] = quaternion_4d_ot[1] # CONTROL.z = OT.y
    quaternion_4d_ctrl[3] = quaternion_4d_ot[3]
    
    return quaternion_4d_ctrl