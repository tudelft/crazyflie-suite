# Crazyflie suite
Flight and data analysis framework for Crazyflies.

# Installation

Ideally, make use of a Python virtual environment:
```bash
$ sudo apt install python3-venv
$ git clone https://github.com/Huizerd/crazyflie-suite.git
$ cd crazyflie-suite
$ python3 -m venv venv
$ source venv/bin/activate
```

You can exit the virtual environment by typing `deactivate` at any time. Next, update `pip` and `setuptools` and install the package and its dependencies:
```bash
$ pip install --upgrade pip && pip install --upgrade setuptools
$ pip install -e .
```

We're installing the package in editable (`-e`) mode, such that we can change it without having to install again.

Code is formatted using [Black](https://github.com/psf/black). To automatically format when committing, run this once:
```bash
$ pre-commit install
```

# Logging a flight

Files needed:
- A logging configuration, like [here](flight/logcfg.json)
- A file specifying the flight space, like [here](flight/space_cyberzoo.yaml)

A flight can be started by running `python flight/logFlight.py`. Several arguments can be supplied:
- `--fileroot`: folder to store the log
- `--keywords`: keywords to identify your run (optional)
- `--logconfig`: location of the logging configuration
- `--space`: location of the flight space specification
- `--estimator`: which estimator to use (`complementary` or `kalman`, must be compatible with flashed firmware)
- `--uwb`: which UWB mode to use (`none`, `twr` or `tdoa`, must be compatible with anchor settings)
- `--flow`: whether or not a Flowdeck is used (optional)
- `--trajectory`: trajectory (or trajectories) to fly (see [here](flight/prepared_trajectories.py) for all options)
- `--optitrack`: how to use OptiTrack (`none`, `logging` or `state`, optional)
- `--optitrack_id`: if using OptiTrack, provide the rigid body ID here (optional)

A simple example can be found [here](configs/example_cyberzoo.sh).

## Autonomous flight
Note that some options are incompatible. For instance, without UWB (`none`), you need OptiTrack for providing state (`state`). If either UWB or a Flowdeck (`--flow`) is used, the Kalman filter has to be selected.

## Manual flight
To log a manual flight, choose "manual" as trajectory (i.e. `--trajectory manual`). Per default, the "PS3_Mode_1" controller mapping is used. It is however recommended to perform the following steps to make sure your controller works as intended:
- Connect your controller.
- Open the crazyflie client. This can be done from the terminal in your virtual environment with the command `cfclient`. Your controller should show up under Input device > Device
- Select a device mapping in Input device > Device > Input map. You can check the behaviour of your controller by moving the sticks and observing the numbers in "Gamepad input" in the "Flight Control" tab.
- If you can't find a mapping that works with your controller, you can create your own map in Input device > Configure device mapping. Select your device, click configure and detect all inputs. Finally save the profile using a memorable name.
- In the flight/log_flight.py file, change line 43 to `self.setup_controller(map="your_profile_name")`