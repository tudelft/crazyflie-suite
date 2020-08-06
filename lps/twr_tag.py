import serial
import time
import sys
import os
import numpy as np
from datetime import datetime
from dataclasses import dataclass, field

# default serial port configuration
DEVICE_DEFAULT = 'ttyACM3'
BAUDRATE_DEFAULT = 115200
FRAMESIZE_DEFAULT = 9
N_ANCHOR = 7
QUEUE_LENGTH = 20
TEST_OPTIONS = 'Aruba'

# Anchor data
@dataclass
class AnchorData_s:
    cnt_total: int = 0
    cnt_success: int = 0
    rate_success: float = 0
    dist_queue: list = field(default_factory=lambda: [])
    dist_mean: float = 0
    dist_stdev: float = 0
    dist_last: float = 0

anchorData = [AnchorData_s() for _ in range(N_ANCHOR)]

""" Open Log File """
def log_open():
    date = datetime.today().strftime(r"%Y%m%d_%H%M%S")
    file_name = date + "_twr_" + TEST_OPTIONS + ".csv"
    cwd = os.path.dirname(__file__)
    file_path = os.path.join(cwd, '../data/', file_name)
    try:
        file_handle = open(file_path, 'w')
        print("Opened log file at '{}'".format(file_path))
        header = "Time, ID, D, DMean{}, DStd{}, \n".format(QUEUE_LENGTH, QUEUE_LENGTH)
        file_handle.write(header)
        return file_handle
    except KeyboardInterrupt:
        print("Could not open log file")
        return None

""" Open connection to a tty device """
def serial_connect(port, baudrate, timeout):
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print('Successfully connected to {}'.format(port))
        return ser
    except serial.SerialException:
        print('Could not connect to {}'.format(port))
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

""" Update anchor data with latest serial data """
def anchor_data_new_distance(dataline):
    global anchorData
    data = dataline.split(" ")
    id = int(data[1][0])
    distance = float(data[-1][:-4])/1000

    anchorData[id].cnt_success += 1
    anchorData[id].rate_success = anchorData[id].cnt_success/anchorData[id].cnt_total
    anchorData[id].dist_last = distance

    if len(anchorData[id].dist_queue)>=30:
        anchorData[id].dist_queue.pop(0)
        anchorData[id].dist_queue.append(distance)
    else:
        anchorData[id].dist_queue.append(distance)
    
    anchorData[id].dist_mean = np.mean(anchorData[id].dist_queue)
    anchorData[id].dist_stdev = np.std(anchorData[id].dist_queue)
    return id

def getArguments():
    arguments = {'Device': None, 'Port': None, 'Baudrate': None}
    arguments['Device'] = DEVICE_DEFAULT
    arguments['Port'] = '/dev/' + arguments['Device']
    arguments['Baudrate'] = BAUDRATE_DEFAULT
    return arguments

def main():
    global anchorData
    t0 = time.perf_counter()
    args = getArguments()

    serial_connection = serial_connect(args['Port'], args['Baudrate'], None)
    log_handle = log_open()

    if (serial_connection is None or log_handle is None):
        return 1

    try:
        while True:
            line = serial_read_line(serial_connection)
            
            if "Interrogating anchor" in line:
                anchor_id = int(line[-3])
                anchorData[anchor_id].cnt_total += 1
            elif "distance" in line:
                anchor_id = anchor_data_new_distance(line)
                t = time.perf_counter() - t0
                log_handle.write("{:.4f}, {}, {:.3f}, {:.3f}, {:.3f}\n".format(
                    t, anchor_id, anchorData[anchor_id].dist_last,
                    anchorData[anchor_id].dist_mean, anchorData[anchor_id].dist_stdev
                ))
            else:
                continue
            
            console_out = "\rID {} ({:.2f}): Current={:5.2f}, Mean={:5.2f}m, Stdev={:3.2f}m".format(
                0, anchorData[0].rate_success, anchorData[0].dist_last, anchorData[0].dist_mean, anchorData[0].dist_stdev
            )
            sys.stdout.write(console_out)

    except KeyboardInterrupt:
        serial_connection.close()
        print('\nConnection Terminated')
        log_handle.close()
        return 0
        


if __name__ == '__main__':
    exit_code = main()