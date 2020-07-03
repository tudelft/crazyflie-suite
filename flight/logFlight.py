"""
Execute a Crazyflie flight and log it.
"""

import argparse
import time
from datetime import datetime

import yaml
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie

import flight.customUtils as util
from flight.FileLogger import FileLogger
from flight.NatNetClient import NatNetClient
from flight.preparedTrajectories import *


def create_filename(fileroot, estimator, uwb, optitrack, trajectory):
    # Date
    date = datetime.today().strftime(r"%Y-%m-%d+%H:%M:%S")
    # Options
    if optitrack:
        options = "+".join([estimator, uwb, trajectory, "optitrack"])
    else:
        options = "+".join([estimator, uwb, trajectory])

    # Join
    if fileroot[-1] == "/":
        return f"{fileroot}{date}+{options}.csv"
    else:
        return f"{fileroot}/{date}+{options}.csv"


def setup_logger(cf, uri, fileroot, logconfig, estimator, uwb, optitrack, trajectory):
    # Create filename from options and date
    file = create_filename(fileroot, estimator, uwb, optitrack, trajectory)
    print(f"Log location: {file}")

    # Logger setup
    flogger = FileLogger(cf, uri, logconfig, file)

    # Enable log configurations based on system setup:
    # Defaults
    flogger.enableConfig("attitude")
    flogger.enableConfig("gyros")
    flogger.enableConfig("acc")
    flogger.enableConfig("state")
    # UWB
    if uwb == "twr":
        flogger.enableConfig("twr")
    elif uwb == "tdoa":
        pass
    # OptiTrack
    if optitrack:
        flogger.enableConfig("otpos")
        flogger.enableConfig("otatt")
    # Estimator
    if estimator == "kalman":
        flogger.enableConfig("kalman")
        flogger.enableConfig("laser")
    elif estimator == "mhe":
        flogger.enableConfig("uwb2posIn")
        flogger.enableConfig("uwb2posEx")
        flogger.enableConfig("uwb2posStats")
        flogger.enableConfig("uwb2posCore")
        flogger.enableConfig("MHEpred")
        flogger.enableConfig("MHEpredV")
        flogger.enableConfig("MHEcorr")
        flogger.enableConfig("MHEcorrV")
        flogger.enableConfig("MHEfinal")
        flogger.enableConfig("MHEfinalV")
        flogger.enableConfig("MHEex")
        flogger.enableConfig("MHEuwb")
        flogger.enableConfig("MHEstats")

    # Start
    flogger.start()
    print("Logging started")

    return flogger


def setup_optitrack(optitrack):
    # If we don't use OptiTrack
    if not optitrack:
        ot_position = None
        ot_attitude = None
    # If we do use OptiTrack
    else:
        # Global placeholders
        ot_position = np.zeros(3)
        ot_attitude = np.zeros(3)

        # Streaming client in separate thread
        streaming_client = NatNetClient()
        streaming_client.newFrameListener = receive_new_frame
        streaming_client.rigidBodyListener = receive_rigidbody_frame
        streaming_client.run()
        print("OptiTrack streaming client started")

    # TODO: do we need to return StreamingClient?
    return ot_position, ot_attitude


def reset_estimator(cf, estimator):
    # Kalman
    if estimator == "kalman":
        cf.param.set_value("kalman.resetEstimation", "1")
        time.sleep(1)
        cf.param.set_value("kalman.resetEstimation", "0")
    # MHE
    elif estimator == "mhe":
        cf.param.set_value("MHE.resetEstimation", "1")
        time.sleep(1)
        cf.param.set_value("MHE.resetEstimation", "0")


def receive_new_frame(*args, **kwargs):
    pass


def receive_rigidbody_frame(id, position, rotation):
    # Modify globals
    # TODO: is flogger needed?
    global flogger, ot_position, ot_attitude

    # Check ID
    if id == ot_id:
        # Register position
        ot_position = util.ot2control(position)
        ot_pos_dict = {
            "otX": ot_position[0],
            "otY": ot_position[1],
            "otZ": ot_position[2],
        }
        flogger.registerData("otpos", ot_pos_dict)

        # Register attitude
        rotation_euler = util.quat2euler(rotation)
        ot_attitude = util.ot2control(rotation_euler)
        ot_att_dict = {
            "otRoll": ot_attitude[0],
            "otPitch": ot_attitude[1],
            "otYaw": ot_attitude[2],
        }
        flogger.registerData("otatt", ot_att_dict)


def select_trajectory(trajectory, space):
    # Load yaml file with space specification
    with open(space, "r") as f:
        space = yaml.full_load(f)
        home = space["home"]
        ranges = space["range"]

    # Account for height offset
    altitude = home["z"] + ranges["z"]
    side_length = min([ranges["x"], ranges["y"]]) * 2
    radius = min([ranges["x"], ranges["y"]])
    x_bound = [home["x"] - ranges["x"], home["x"] + ranges["x"]]
    y_bound = [home["y"] - ranges["y"], home["y"] + ranges["y"]]

    # Select correct trajectory and scale
    if trajectory == "nothing":
        setpoints = None
    if trajectory == "hover":
        setpoints = hover(home["x"], home["y"], altitude)
    elif trajectory == "square":
        setpoints = square(home["x"], home["y"], side_length, altitude)
    elif trajectory == "octagon":
        setpoints = octagon(home["x"], home["y"], radius, altitude)
    elif trajectory == "triangle":
        setpoints = triangle(home["x"], home["y"], radius, altitude)
    elif trajectory == "hourglass":
        setpoints = hourglass(home["x"], home["y"], side_length, altitude)
    elif trajectory == "random":
        setpoints = randoms(home["x"], home["y"], x_bound, y_bound, altitude)
    elif trajectory == "scan":
        setpoints = scan(home["x"], home["y"], x_bound, y_bound, altitude)

    return setpoints


def follow_setpoints(cf, setpoints):
    # Start
    try:
        print("Flight started")
        # Do nothing, just sit on the ground
        if setpoints is None:
            while True:
                pass

        # Do actual flight
        else:
            for i, point in enumerate(setpoints):
                print(f"Next setpoint: {point}")

                # Compute time based on distance
                # Take-off
                if i == 0:
                    distance = point[2]
                # No take-off
                else:
                    distance = np.sqrt(
                        ((np.array(point) - np.array(setpoints[i - 1])) ** 2).sum()
                    )

                # If zero distance, at least some wait time
                if distance == 0.0:
                    wait = 1
                else:
                    wait = distance * 2

                # Send position and wait
                time_passed = 0.0
                while time_passed < wait:
                    cf.commander.send_position_setpoint(*point)
                    time.sleep(0.05)
                    time_passed += 0.05

            # Finished
            cf.commander.send_stop_setpoint()

    # Prematurely break off flight / quit doing nothing
    except KeyboardInterrupt:
        if setpoints is None:
            print("Quit doing nothing!")
        else:
            print("Emergency landing!")
            wait = setpoints[i][2] * 2
            cf.commander.send_position_setpoint(*setpoints[i][:2], 0.0, 0.0)
            time.sleep(wait)
            cf.commander.send_stop_setpoint()


if __name__ == "__main__":

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--fileroot", type=str, required=True)
    parser.add_argument("--logconfig", type=str, required=True)
    parser.add_argument("--space", type=str, required=True)
    parser.add_argument(
        "--estimator", choices=["kalman", "mhe"], type=str.lower, required=True
    )
    parser.add_argument("--uwb", choices=["twr", "tdoa"], type=str.lower, required=True)
    parser.add_argument(
        "--trajectory",
        choices=[
            "nothing",
            "hover",
            "square",
            "octagon",
            "triangle",
            "hourglass",
            "random",
            "scan",
        ],
        type=str.lower,
        default="hover",
    )
    parser.add_argument("--optitrack_id", type=int, default=None)
    parser.add_argument("--optitrack", action="store_true")
    args = vars(parser.parse_args())

    # Set up Crazyflie
    uri = "radio://0/100/2M/E7E7E7E7E7"
    cflib.crtp.init_drivers(enable_debug_driver=False)
    cf = Crazyflie(rw_cache="./cache")

    # Set up logging
    flogger = setup_logger(
        cf,
        uri,
        args["fileroot"],
        args["logconfig"],
        args["estimator"],
        args["uwb"],
        args["optitrack"],
        args["trajectory"],
    )

    # Check OptiTrack if it's there
    ot_id = args["optitrack_id"]
    ot_position, ot_attitude = setup_optitrack(args["optitrack"])

    # Wait for fix
    if ot_position is not None:
        while (ot_position == 0).any():
            print("Waiting for OptiTrack fix...")
            time.sleep(1)
        print("OptiTrack fix acquired")

    # Reset estimator
    reset_estimator(cf, args["estimator"])
    time.sleep(2)

    # Select trajectory
    setpoints = select_trajectory(args["trajectory"], args["space"])

    # Do flight
    follow_setpoints(cf, setpoints)

    # End flight
    print("Done")
    time.sleep(2)
    cf.close_link()
