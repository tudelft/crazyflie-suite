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
import os
import sys

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie import Console
from cfclient.utils.input import JoystickReader

import flight.utils as util
from flight.FileLogger import FileLogger
from flight.NatNetClient import NatNetClient

# TODO: merge these? (prepared trajectories and trajectories)
from flight.trajectories import takeoff, landing
from flight.prepared_trajectories import *

class LogFlight():
    def __init__(self, args):
        self.args = args
        self.optitrack_enabled = False
        self.console_dump_enabled = False

        cflib.crtp.init_drivers(enable_debug_driver=False)
        self._cf = Crazyflie(rw_cache="./cache")
        self._jr = JoystickReader(do_device_discovery=False)

        # Set flight mode (manual/autonomous)
        if self.args["trajectory"][0] == "manual": 
            # Check if controller is connected
            assert self.controller_connected(), "No controller detected for manual flight."
            self.setup_controller()
            self.manual_flight = True
        else:
            # Make sure drone is setup to perform autonomous flight
            if args["uwb"] == "none":
                assert args["optitrack"] == "state", "OptiTrack state needed in absence of UWB"
                assert args["estimator"] == "kalman", "OptiTrack state needs Kalman estimator"
            self.manual_flight = False

        # set estimator to kalman in absence of UWB and Flowdeck
        if args["uwb"]=="none" and not args["flow"] and args["estimator"]=="kalman":
            print(
                "Absence of UWB and Flowdeck will lead Crazyflie to set estimator to 'complementary', so we set it manually to 'kalman'"
            )
            self._cf.param.set_value("stabilizer.estimator", "2")

        # Setup the logging framework
        self.setup_logger()

        # Setup optitrack if required
        if not args["optitrack"] == "none":
            self.setup_optitrack()

    def get_filename(self):
        # get relevant arguments
        fileroot = self.args["fileroot"] 
        keywords = self.args["keywords"]
        estimator = self.args["estimator"]
        uwb = self.args["uwb"]
        optitrack = self.args["optitrack"]
        trajectory = self.args["trajectory"]

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
        name = "{}+{}.csv".format(date, options)
        return os.path.normpath(os.path.join(os.getcwd(), fileroot, name))


    def setup_logger(self):
        # Create directory if not there
        Path(self.args["fileroot"]).mkdir(exist_ok=True)
        
        # Create filename from options and date
        self.log_file = self.get_filename()
        print(f"Log location: {self.log_file}")

        # Logger setup
        logconfig = self.args["logconfig"]
        self.flogger = FileLogger(self._cf, logconfig, self.log_file)

        # Enable log configurations based on system setup:
        # Defaults
        self.flogger.enableConfig("attitude")
        self.flogger.enableConfig("gyros")
        self.flogger.enableConfig("acc")
        self.flogger.enableConfig("state")
        # UWB
        if self.args["uwb"] == "twr":
            self.flogger.enableConfig("twr")
        elif self.args["uwb"] == "tdoa":
            print("Needs custom TDoA logging in firmware!")
            # For instance, see here: https://github.com/Huizerd/crazyflie-firmware/blob/master/src/utils/src/tdoa/tdoaEngine.c
            # flogger.enableConfig("tdoa")
        # Flow
        if self.args["flow"]:
            self.flogger.enableConfig("laser")
            self.flogger.enableConfig("flow")
        # OptiTrack
        if self.args["optitrack"] != "none":
            self.flogger.enableConfig("otpos")
            self.flogger.enableConfig("otatt")
        # Estimator
        if self.args["estimator"] == "kalman":
            self.flogger.enableConfig("kalman")


    def setup_optitrack(self):
        self.ot_id = self.args["optitrack_id"]
        self.ot_position = np.zeros(3)
        self.ot_attitude = np.zeros(3)
        self.ot_quaternion = np.zeros(4)

        # Streaming client in separate thread
        streaming_client = NatNetClient()
        streaming_client.newFrameListener = self.ot_receive_new_frame
        streaming_client.rigidBodyListener = self.ot_receive_rigidbody_frame
        streaming_client.run()
        self.optitrack_enabled = True
        print("OptiTrack streaming client started")

        # TODO: do we need to return StreamingClient?


    def reset_estimator(self):
        # Kalman
        if self.args["estimator"] == "kalman":
            self._cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(1)
            self._cf.param.set_value("kalman.resetEstimation", "0")

    def ot_receive_new_frame(self, *args, **kwargs):
        pass

    def ot_receive_rigidbody_frame(self, id, position, rotation):
        # Check ID
        if id == self.ot_id:
            # Register position
            self.ot_position = util.ot2control(position)
            ot_pos_dict = {
                "otX": self.ot_position[0],
                "otY": self.ot_position[1],
                "otZ": self.ot_position[2],
            }
            self.flogger.registerData("otpos", ot_pos_dict)

            # Register attitude
            self.ot_attitude = util.quat2euler(rotation)
            self.ot_quaternion = util.ot2control_quat(rotation)
            ot_att_dict = {
                "otRoll": self.ot_attitude[0],
                "otPitch": self.ot_attitude[1],
                "otYaw": self.ot_attitude[2],
                "otq0": self.ot_quaternion[0],
                "otq1": self.ot_quaternion[1],
                "otq2": self.ot_quaternion[2],
                "otq3": self.ot_quaternion[3]
            }
            self.flogger.registerData("otatt", ot_att_dict)

    def do_taskdump(self):
        self._cf.param.set_value("system.taskDump", "1")

    def process_taskdump(self, console_log):
        file = self.get_filename()
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


    def build_trajectory(self, trajectories, space):
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
            elif trajectory == "hover_fw":
                setpoints += hover_fw(home["x"], home["y"], altitude)
            elif trajectory == "square":
                setpoints += square(home["x"], home["y"], side_length, altitude)
            elif trajectory == "square_fw":
                setpoints += square_fw(home["x"], home["y"], side_length, altitude)
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


    def follow_setpoints(self, cf, setpoints, optitrack):
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
                        self.do_taskdump()
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
                            ((np.array(point[:3]) - np.array(setpoints[i - 1][:3])) ** 2).sum()
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
                                self.ot_position[0], self.ot_position[1], self.ot_position[2]
                            )
                        cf.commander.send_position_setpoint(*point)
                        time.sleep(0.05)
                        time_passed += 0.05
                        time_since_dump += 0.05

                    # Task dump
                    if time_since_dump > 2:
                        print("Do task dump")
                        self.do_taskdump()
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

    def controller_connected(self):
        """ Return True if a controller is connected """
        return len(self._jr.available_devices()) > 0

    def setup_controller(self, map="PS3_Mode_1"):
        devs = []
        for d in self._jr.available_devices():
            devs.append(d.name)
        
        if len(devs)==1:
            input_device = 0
        else:
            print("Multiple controllers detected:")
            for i, dev in enumerate(devs):
                print(" - Controller #{}: {}".format(i, dev))
            
            input_device = int(input("Select controller: "))

        if not input_device in range(len(devs)):
            raise ValueError
        
        self._jr.start_input(devs[input_device])
        self._jr.set_input_map(devs[input_device], map)

    def connect_crazyflie(self, uri):   
        """Connect to a Crazyflie on the given link uri"""
        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        if self.manual_flight:
            # Add callbacks for manual control
            self._cf.param.add_update_callback(
                group="imu_sensors", name="AK8963", cb=(
                    lambda name, found: self._jr.set_alt_hold_available(
                        eval(found))))
            self._jr.assisted_control_updated.add_callback(
                lambda enabled: self._cf.param.set_value("flightmode.althold",
                                                     enabled))
            self._cf.open_link(uri)
            self._jr.input_updated.add_callback(self._cf.commander.send_setpoint)

        else:
            self._cf.open_link(uri)

    def _connected(self, link):
        """This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print("Connected to %s" % link)
        self.flogger.start()
        print("logging started")

    def _connection_failed(self, link_uri, msg):
        print("Connection to %s failed: %s" % (link_uri, msg))
        self.flogger.is_connected = False

    def _connection_lost(self, link_uri, msg):
        print("Connection to %s lost: %s" % (link_uri, msg))
        self.flogger.is_connected = False

    def _disconnected(self, link_uri):
        print("Disconnected from %s" % link_uri)
        self.flogger.is_connected = False

    def ready_to_fly(self):
        # Wait for connection
        timeout = 10
        while not self._cf.is_connected():
            print("Waiting for Crazyflie connection...")
            time.sleep(2)
            timeout -= 1
            if timeout<=0:
                return False
        
        # Wait for optitrack
        if self.optitrack_enabled:
            while (self.ot_position == 0).any():
                print("Waiting for OptiTrack fix...")
                time.sleep(2)
                timeout -= 1
                if timeout <= 0:
                    return False

            print("OptiTrack fix acquired")

        print("Reset Estimator...")
        self.reset_estimator()

        return True
        
    def start_flight(self):
        if self.ready_to_fly():
            if self.manual_flight:
                print("Manual Flight - Ready to fly")
                while(True):
                    # self._cf.extpos.send_extpose(
                    #     self.ot_position[0], self.ot_position[1], self.ot_position[2],
                    #     self.ot_quaternion[0], self.ot_quaternion[1], self.ot_quaternion[2], self.ot_quaternion[3]
                    #     )
                    time.sleep(0.01)
            else:
                setpoints = self.build_trajectory(args["trajectory"], args["space"])
                print("Autonomous Flight - Starting flight")
                # Build trajectory
                # Do flight
                print("flying...")
                self.follow_setpoints(self._cf, setpoints, args["optitrack"])
                print("Flight complete.")
        
        else:
            print("Timeout while waiting for flight ready.")

    def setup_console_dump(self):
        # Console dump file
        self.console_log = []
        console = Console(self._cf)
        console.receivedChar.add_callback(self._console_cb)
        self.console_dump_enabled = True

    def _console_cb(self, text):
        self.console_log.append(text)

    def end(self):
        self._cf.close_link()
        # Process task dumps
        # TODO: add timestamps / ticks (like logging) to this
        if self.console_dump_enabled:
            self.process_taskdump(self.console_log)

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

    # Set up log flight
    lf = LogFlight(args)
    lf.connect_crazyflie("radio://0/80/2M/E8E7E7E7E8")
    # Set up print connection to console
    # TODO: synchronize this with FileLogger: is this possible?
    # lf.setup_console_dump()

    try:
        lf.start_flight()
    except KeyboardInterrupt:
        print("Keyboard Interrupt")

    # End flight
    print("Done - Please Wait for CF to disconnect")
    time.sleep(1)
    lf.end()
