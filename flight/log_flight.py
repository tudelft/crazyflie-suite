"""
Execute a Crazyflie flight and log it.
"""

import argparse
import time
from datetime import datetime
from pathlib import Path

import yaml
import numpy as np
import pandas as pd

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie import Console

import flight.utils as util
from flight.FileLogger import FileLogger
from flight.NatNetClient import NatNetClient

# TODO: merge these? (prepared trajectories and trajectories)
from flight.trajectories import takeoff, landing
from flight.prepared_trajectories import *


def create_filename(fileroot, keywords, estimator, uwb, optitrack, trajectory):
    # Date
    date = datetime.today().strftime(r"%Y-%m-%d+%H:%M:%S")

    # Additional keywords
    if keywords is not None:
        keywords = "+" + "+".join(keywords)
    else:
        keywords = ""

    # Options
    if optitrack == "logging":
        options = f"{estimator}+{uwb}{keywords}+optitracklog+{'_'.join(trajectory)}"
    elif optitrack == "state":
        options = f"{estimator}+{uwb}{keywords}+optitrackstate+{'_'.join(trajectory)}"
    else:
        options = f"{estimator}+{uwb}{keywords}+{'_'.join(trajectory)}"

    # Join
    if fileroot[-1] == "/":
        return f"{fileroot}{date}+{options}.csv"
    else:
        return f"{fileroot}/{date}+{options}.csv"


def setup_logger(
    cf, uri, fileroot, keywords, logconfig, estimator, uwb, flow, optitrack, trajectory
):
    # Create filename from options and date
    file = create_filename(fileroot, keywords, estimator, uwb, optitrack, trajectory)
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
        print("Needs custom TDoA logging in firmware!")
        # For instance, see here: https://github.com/Huizerd/crazyflie-firmware/blob/master/src/utils/src/tdoa/tdoaEngine.c
        # flogger.enableConfig("tdoa")
    # Flow
    if flow:
        flogger.enableConfig("laser")
        flogger.enableConfig("flow")
    # OptiTrack
    if optitrack != "none":
        flogger.enableConfig("otpos")
        flogger.enableConfig("otatt")
    # Estimator
    if estimator == "kalman":
        flogger.enableConfig("kalman")

    # Start
    flogger.start()
    print("Logging started")

    return flogger, file


def setup_optitrack(optitrack):
    # If we don't use OptiTrack
    if optitrack == "none":
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


def console_cb(text):
    global console_log
    console_log.append(text)


def do_taskdump(cf):
    cf.param.set_value("system.taskDump", "1")


def process_taskdump(file, console_log):
    # Dataframe placeholders
    label_data, load_data, stack_data = [], [], []

    # Get headers
    headers = []
    for i, line in enumerate(console_log):
        if "Task dump" in line:
            headers.append(i)
    # None indicates the end of the list
    headers.append(None)

    # Get one task dump
    for i in range(len(headers) - 1):
        dump = console_log[headers[i] + 2 : headers[i + 1]]

        # Process strings: strip \n, \t, spaces, SYSLOAD:
        loads, stacks, labels = [], [], []
        for line in dump:
            entries = line.strip("SYSLOAD: ").split("\t")
            loads.append(entries[0].strip())  # no sep means strip all space, \n, \t
            stacks.append(entries[1].strip())
            labels.append(entries[2].strip())

        # Store labels
        if not label_data:
            label_data = labels

        # Append to placeholders
        load_data.append(loads)
        stack_data.append(stacks)

    # Check if we have data at all
    if headers[0] is not None and label_data:
        # Put in dataframe
        load_data = pd.DataFrame(load_data, columns=label_data)
        stack_data = pd.DataFrame(stack_data, columns=label_data)

        # Save dataframes
        load_data.to_csv(file.strip(".csv") + "+load.csv", sep=",", index=False)
        stack_data.to_csv(file.strip(".csv") + "+stackleft.csv", sep=",", index=False)
    else:
        print("No task dump data found")


def build_trajectory(trajectories, space):
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

    # Build trajectory
    # Takeoff
    setpoints = takeoff(home["x"], home["y"], altitude, 0.0)
    for trajectory in trajectories:
        # If nothing, only nothing
        if trajectory == "nothing":
            setpoints = None
            return setpoints
        elif trajectory == "hover":
            setpoints += hover(home["x"], home["y"], altitude)
        elif trajectory == "square":
            setpoints += square(home["x"], home["y"], side_length, altitude)
        elif trajectory == "octagon":
            setpoints += octagon(home["x"], home["y"], radius, altitude)
        elif trajectory == "triangle":
            setpoints += triangle(home["x"], home["y"], radius, altitude)
        elif trajectory == "hourglass":
            setpoints += hourglass(home["x"], home["y"], side_length, altitude)
        elif trajectory == "random":
            setpoints += randoms(home["x"], home["y"], x_bound, y_bound, altitude)
        elif trajectory == "scan":
            setpoints += scan(home["x"], home["y"], x_bound, y_bound, altitude)
        else:
            raise ValueError(f"{trajectory} is an unknown trajectory")

    # Add landing
    setpoints += landing(home["x"], home["y"], altitude, 0.0)

    return setpoints


def follow_setpoints(cf, setpoints, optitrack):
    # Counter for task dump logging
    time_since_dump = 0.0

    # Start
    try:
        print("Flight started")
        # Do nothing, just sit on the ground
        if setpoints is None:
            while True:
                time.sleep(0.05)
                time_since_dump += 0.05

                # Task dump
                if time_since_dump > 2:
                    print("Do task dump")
                    do_taskdump(cf)
                    time_since_dump = 0.0

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
                    # If we use OptiTrack for control, send position to Crazyflie
                    if optitrack == "state":
                        cf.extpos.send_extpos(
                            ot_position[0], ot_position[1], ot_position[2]
                        )
                    cf.commander.send_position_setpoint(*point)
                    time.sleep(0.05)
                    time_passed += 0.05
                    time_since_dump += 0.05

                # Task dump
                if time_since_dump > 2:
                    print("Do task dump")
                    do_taskdump(cf)
                    time_since_dump = 0.0

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
    parser.add_argument("--keywords", nargs="+", type=str.lower, default=None)
    parser.add_argument("--logconfig", type=str, required=True)
    parser.add_argument("--space", type=str, required=True)
    parser.add_argument(
        "--estimator",
        choices=["complementary", "kalman"],
        type=str.lower,
        required=True,
    )
    parser.add_argument(
        "--uwb", choices=["none", "twr", "tdoa"], type=str.lower, required=True
    )
    parser.add_argument("--flow", action="store_true")
    parser.add_argument("--trajectory", nargs="+", type=str.lower, required=True)
    parser.add_argument(
        "--optitrack",
        choices=["none", "logging", "state"],
        type=str.lower,
        default="none",
    )
    parser.add_argument("--optitrack_id", type=int, default=None)
    args = vars(parser.parse_args())

    # If no UWB, then OptiTrack
    # If no UWB and Flowdeck, then complementary
    if args["uwb"] == "none":
        assert args["optitrack"] == "state", "OptiTrack state needed in absence of UWB"
        if not args["flow"]:
            assert args["estimator"] == "complementary", "Absence of UWB and Flowdeck will lead Crazyflie to set estimator to 'complementary'"

    # Set up Crazyflie
    uri = "radio://0/80/2M/E7E7E7E7E7"
    cflib.crtp.init_drivers(enable_debug_driver=False)
    cf = Crazyflie(rw_cache="./cache")

    # Set up print connection to console
    # TODO: synchronize this with FileLogger: is this possible?
    console_log = []
    console = Console(cf)
    console.receivedChar.add_callback(console_cb)

    # Create directory if not there
    Path(args["fileroot"]).mkdir(exist_ok=True)

    # Set up logging
    flogger, file = setup_logger(
        cf,
        uri,
        args["fileroot"],
        args["keywords"],
        args["logconfig"],
        args["estimator"],
        args["uwb"],
        args["flow"],
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

    # Build trajectory
    setpoints = build_trajectory(args["trajectory"], args["space"])

    # Do flight
    follow_setpoints(cf, setpoints, args["optitrack"])

    # End flight
    print("Done")
    time.sleep(2)
    cf.close_link()

    # Process task dumps
    # TODO: add timestamps / ticks (like logging) to this
    process_taskdump(file, console_log)
