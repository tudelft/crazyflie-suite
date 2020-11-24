import serial
import time
import sys
import os
import numpy as np
from datetime import datetime
from dataclasses import dataclass, field

import lps

# default serial port configuration
DEVICE_DEFAULT = 'ttyACM1'
BAUDRATE_DEFAULT = 115200
FRAMESIZE_DEFAULT = 9
N_ANCHOR = 7

# Test parameters (for Filename)
test_params = lps.test_parameters_t(
    location='Lab',
    test_mode='twr-ranging',
    tx_mode='max',
    nominal_range='7m',
    anchor_orientation='A-V0',
    tag_orientation='T-V0',
    options='forceZ',
)

# Constants
TIMEOUT_THRESHOLD = 0.2 #[s]
FORCE_Z = 1 #[m], set to -1 for no force

# Anchor data
@dataclass
class AnchorData_s:
    anchor_id: int
    cnt_total: int = 0
    cnt_success: int = 0
    rate_success: float = 0
    distance: float = 0
    timestamp: float = -1
    position: list = field(default_factory=lambda:[0,0,0])
    timeout: bool = True

anchor = [AnchorData_s(anchor_id=i) for i in range(N_ANCHOR)]
t0 = time.perf_counter()

def twr_anchor_update_position(dataline):
    global anchor

    idx = int(dataline[7])
    pos_data = dataline.split(':')[1][1:-2]
    pos = pos_data.split(',')

    anchor[idx].position = [float(pos[0]), float(pos[1]), float(pos[2])]

def twr_tag_getpos(position_prior):
    global anchor, t0
    good_anchors = []
    t = time.perf_counter()-t0
    for an in anchor:
        if t-an.timestamp < TIMEOUT_THRESHOLD:
            good_anchors.append(an)
        else:
            an.timeout = True
            continue
    
    multilat_min_anchors = 4 if FORCE_Z<0 else 3

    if len(good_anchors)>multilat_min_anchors:
        return twr_multilat(good_anchors)
    else:
        return twr_projection(good_anchors, position_prior)


def twr_multilat(anchors):
    N = len(anchors)
    if FORCE_Z<0:
        A = np.empty((N,4))
        b = np.empty((N,1))

        for i, an in enumerate(anchors):
            A[i] = [1, -2*an.position[0], -2*an.position[1], -2*an.position[2]]
            b[i] = an.distance**2 - an.position[0]**2 - an.position[1]**2 - an.position[2]**2
        result = np.dot(np.linalg.pinv(A), b)
        pos = result[1:]

    else:
        A = np.empty((N,3))
        b = np.empty((N))

        for i, an in enumerate(anchors):
            A[i] = [1, -2*an.position[0], -2*an.position[1]]
            b[i] = an.distance**2 - an.position[0]**2 - an.position[1]**2 - (an.position[2]-FORCE_Z)**2
        result = np.dot(np.linalg.pinv(A), b)
    result = np.dot(np.linalg.pinv(A), b)
    pos = [result[1], result[2], FORCE_Z]

    return pos


def twr_projection(anchors, position_prior):
    N = len(anchors)
    pos = np.empty((N,3))

    if FORCE_Z<0:
        for i, an in enumerate(anchors):
            r = np.subtract(position_prior-an.position)
            r_norm = r/np.linalg.norm(r)
            pos[i] = np.add(an.position, np.multiply(an.distance, r_norm))
    else:
        for i, an in enumerate(anchors):
            r = np.subtract(position_prior[:2], an.position[:2])
            r_norm = r/np.linalg.norm(r)
            pos[i][:2] = np.add(an.position[:2], np.multiply(an.distance, r_norm))
            pos[i][2] = FORCE_Z

    return np.mean(pos, axis=0)


""" Update anchor data with latest serial data """
def anchor_add_distance(dataline):
    global anchor

    data = dataline.split(" ")
    idx = int(data[1][0])
    dist = float(data[-1][:-4])/1000

    anchor[idx].cnt_success += 1
    # to avoid crashes at start when the interrogation line gets lost
    if anchor[idx].cnt_total == 0:
        anchor[idx].cnt_total = 1
    anchor[idx].rate_success = anchor[idx].cnt_success/anchor[idx].cnt_total
    anchor[idx].distance = dist
    anchor[idx].timeout = False

    return idx

def getArguments():
    arguments = {'Device': None, 'Port': None, 'Baudrate': None}
    arguments['Device'] = DEVICE_DEFAULT
    arguments['Port'] = '/dev/' + arguments['Device']
    arguments['Baudrate'] = BAUDRATE_DEFAULT
    return arguments

def main():
    global anchor
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
        header = 'Time, ID, dist, X, Y, Z, dc \n'
        log_handle.write(header)

    # read serial data and write to log
    pos = [0,0,0] # initialize position prior
    try:
        sys.stdout.write('  t ||    A0,    A1,    A2,    A3,    A4,    A5,    A6 || Position\n')
        t=0
        while t<60:
            dataline = lps.serial_read_line(serial_connection)
            t = time.perf_counter()-t0

            if "Interrogating anchor" in dataline:
                idx = int(dataline[-3])
                anchor[idx].cnt_total += 1

            elif "Position" in dataline:
                twr_anchor_update_position(dataline)

            elif "distance" in dataline:
                idx = anchor_add_distance(dataline)
                anchor[idx].timestamp = t
                pos = twr_tag_getpos(pos)

                log_entry = "{:.4f}, {}, {:.3f}, {:.3f}, {:.3f}, {:.3f}\n".format(
                            t, idx, anchor[idx].distance, pos[0], pos[1], pos[2])
                log_handle.write(log_entry)
                
            else:
                continue
            
            anchor_state = ['' for _ in range(N_ANCHOR)]
            for i, an in enumerate(anchor):
                anchor_state[i] = '{:5.2f}'.format(an.distance) if not an.timeout else ' D/C '

            
            console_out = "\r{:3.0f} || {}, {}, {}, {}, {}, {}, {} || ({:5.2f},{:5.2f},{:5.2f})".format(
                t, *anchor_state, *pos)
            sys.stdout.write(console_out)

    except KeyboardInterrupt:
        pass
    finally:
        serial_connection.close()
        print('\nConnection Terminated')
        log_handle.close()
        return 0
        


if __name__ == '__main__':
    exit_code = main()