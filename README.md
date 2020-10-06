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

Note that some options are incompatible. For instance, without UWB (`none`), you need OptiTrack for providing state (`state`). If either UWB or a Flowdeck (`--flow`) is used, the Kalman filter has to be selected.

A simple example can be found [here](configs/example_cyberzoo.sh).
