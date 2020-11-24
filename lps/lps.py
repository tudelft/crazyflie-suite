import os
import serial
from datetime import datetime
from dataclasses import dataclass

@dataclass
class test_parameters_t:
    location: str
    test_mode: str
    nominal_range: str
    tx_mode: str
    anchor_orientation: str
    tag_orientation: str
    options: str = None

""" Create a compatible logname (full path) from the test parameters """
def logname_from_params(params: test_parameters_t, data_dir=None):
    if data_dir is None:
        cwd = os.path.dirname(__file__)
        data_dir = os.path.join(cwd, '../data')

    time = datetime.today().strftime(r"%H%M%S")
    date = datetime.today().strftime(r"%Y%m%d")

    dirname = '{}-{}'.format(params.location, date)
    filename = '{}_{}_{}_{}_{}_{}.csv'.format(
        time, params.test_mode, params.nominal_range, params.tx_mode,
        params.anchor_orientation, params.tag_orientation)
    
    if params.options is not None:
        filename = filename.replace('.csv', '_{}.csv'.format(params.options))
    
    full_path = os.path.normpath(os.path.join(data_dir, dirname, filename))
    
    return full_path


""" Extract test parameters from a compatible log name (full path)"""
def params_from_logname(logpath: str):
    (directory, filename) = os.path.split(logpath)
    (_, dirname) = os.path.split(directory)
    loc = dirname.split('-')[0]
    parts = filename.split('.')[0].split('_')

    if len(parts)<5:
        raise ValueError("Incompatible Logname")

    params = test_parameters_t(
        location=loc,
        test_mode=parts[1],
        nominal_range=parts[2],
        tx_mode=parts[3],
        anchor_orientation=parts[4],
        tag_orientation=parts[5]
    )

    if len(parts)>6:
        params.options = parts[6]
    
    return params


""" Opens log at with standardized name & path and returns file handle. 
    Creates new directory if required. """
def log_open(params: test_parameters_t):
    log_path = logname_from_params(params)
    
    dir_path = os.path.dirname(log_path)
    if not os.path.exists(dir_path):
        os.mkdir(dir_path)
        print("Created new data directory: '{}'".format(dir_path))
    else:
        print("Logging to existing data directory: '{}'".format(dir_path))

    try:
        file_handle = open(log_path, 'w')
        print("Opened new log file '{}'".format(os.path.split(log_path)[1]))
        return file_handle
    except KeyboardInterrupt:
        print("Could not open log file")
        return None


""" Open serial connection to a tty device and return handle"""
def serial_connect(port, baudrate, timeout):
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print("Successfully connected to '{}'".format(port))
        return ser
    except serial.SerialException:
        print("Could not connect to '{}'".format(port))
        return None
    except ValueError:
        print('Could not open serial connection: Invalid Parameters')
        return None


""" Read a full line from a serial connection"""
def serial_read_line(handle):
    data_line = ""
    try:
        while True:
            data_raw = handle.read()
            data = data_raw.decode('utf-8')
            data_line = data_line + data
            if data == "\n":
                break
        return data_line

    except serial.SerialException:
        return "[Error]: SerialException in serial_read_line()"


# debugging
if __name__ == "__main__":
    params = test_parameters_t(
        location='LAB',
        test_mode='code-test',
        nominal_range='0m',
        tx_mode='none',
        anchor_orientation='noA',
        tag_orientation='noT',
        options='TEST'
    )

    log_open(params)