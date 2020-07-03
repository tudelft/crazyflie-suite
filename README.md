# crazyflie-suite
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

A flight can be started by running `python flight/logFlight.py`. Several arguments need to be supplied:
- `--fileroot`: folder to store the log
- `--logconfig`: location of the logging configuration
- `--space`: location of the flight space specification
- `--estimator`: which estimator to use (`kalman` or `mhe`, must be compatible with flashed firmware)
- `--uwb`: which UWB mode to use (`twr` or `tdoa`, must be compatible with anchor settings)
- `--trajectory`: trajectory to fly (see [here](flight/preparedTrajectories.py) for all options)
- `--optitrack`: flag for using OptiTrack
- `--optitrack_id`: if using OptiTrack, provide the rigid body ID here

An example of all this can be found [here](flight/example_cyberzoo.sh).
