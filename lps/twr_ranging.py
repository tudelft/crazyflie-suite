import time
import sys
import numpy as np
from dataclasses import dataclass, field

import lps

# default serial port configuration
DEVICE_DEFAULT = 'ttyACM1'
BAUDRATE_DEFAULT = 115200
FRAMESIZE_DEFAULT = 9
N_ANCHOR = 7
QUEUE_LENGTH = 50

# Test parameters (for Filename)
test_params = lps.test_parameters_t(
    location='Aruba',
    test_mode='twr-ranging',
    tx_mode='max',
    nominal_range='7m',
    anchor_orientation='A-V0',
    tag_orientation='T-V0',
    options='24.4-70',
)

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

    # open serial connection to tag
    serial_connection = lps.serial_connect(args['Port'], args['Baudrate'], None)
    if (serial_connection is None):
        return 1

    # open log file
    log_handle = lps.log_open(test_params)
    if (log_handle is None):
        serial_connection.close()
        return 1
    else:
        header = "Time, ID, D, DMean{}, DStd{}, \n".format(QUEUE_LENGTH, QUEUE_LENGTH)
        log_handle.write(header)

    # read serial data and write to log
    try:
        while True:
            line = lps.serial_read_line(serial_connection)
            
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
            
            console_out = "\rID {} ({:.2f}): Current={:5.2f}, Mean={:5.2f}m, Stdev={:3.2f}m -- ID {} ({:.2f}): Current={:5.2f}, Mean={:5.2f}m, Stdev={:3.2f}m".format(
                0, anchorData[0].rate_success, anchorData[0].dist_last, anchorData[0].dist_mean, anchorData[0].dist_stdev,
                1, anchorData[1].rate_success, anchorData[1].dist_last, anchorData[1].dist_mean, anchorData[1].dist_stdev
            )
            sys.stdout.write(console_out)

    except KeyboardInterrupt:
        serial_connection.close()
        print('\nConnection Terminated')
        log_handle.close()
        return 0
        


if __name__ == '__main__':
    exit_code = main()