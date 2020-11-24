import time
import sys
import os
import numpy as np
from dataclasses import dataclass, field

# import from different directory
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)

import lps

# Serial port configuration
DEVICE = 'ttyACM1'
BAUDRATE = 115200
FRAMESIZE = 9
ttyargs = {'Device': DEVICE, 'Port': '/dev/' + DEVICE, 'Baudrate': BAUDRATE}

# Parameters
N_ANCHOR = 8
file_name = 'auto_measure.csv'

dir_path = os.path.dirname(os.path.realpath(__file__))
csv_file = os.path.join(dir_path, file_name)

# Anchor data
@dataclass
class AnchorData_s:
    dist_queue: list = field(default_factory=lambda: [])
    dist_mean: float = 0
    dist_stdev: float = 0
    dist_isGood: bool = False

anchorData = [AnchorData_s() for _ in range(N_ANCHOR)]

""" Update anchor data with latest serial data """
def anchor_data_new_distance(dataline):
    global anchorData
    data = dataline.split(" ")
    id = int(data[1][0])
    distance = float(data[-1][:-4])/1000

    anchorData[id].dist_queue.append(distance)
    anchorData[id].dist_mean = np.mean(anchorData[id].dist_queue)
    anchorData[id].dist_stdev = np.std(anchorData[id].dist_queue)

    if len(anchorData[id].dist_queue) >= 100:
        anchorData[id].dist_isGood = True


def main():
    global anchorData
    # open serial connection to tag
    serial_connection = lps.serial_connect(ttyargs['Port'], ttyargs['Baudrate'], None)
    if (serial_connection is None):
        print('Could not open serial connection to tag: Aborting')
        return 1

    print('---Anchor position data collector---')
    print('Please enter tag coordinates:')
    x = input('x: ')
    y = input('y: ')
    z = input('z: ')

    print('Collecting distance data: |')

    # read serial data and write to log
    counter = 0
    try:
        while True:
            counter += 1
            if counter%50 == 0:
                console_out = "\rCollecting distance data: |" + (counter/20)*'#'
                sys.stdout.write(console_out)

            line = lps.serial_read_line(serial_connection)
            
            if "distance" in line:
                anchor_data_new_distance(line)
            else:
                continue
            
            isGood = True
            for i in range(N_ANCHOR):
                isGood = isGood and anchorData[i].dist_isGood

            if isGood or counter>10000:
                break
                
    except:
        print('An error occured while collecting distance data!')
        return 1

    
    # open log file
    try:
        if os.path.isfile(csv_file):
            f = open(csv_file, 'a')
        else:
            f = open(csv_file, 'w')
            f.write('x,y,z,d0,d1,d2,d3,d4,d5,d6,d7\n')

        data = [x, y, z]
        for i in range(N_ANCHOR):
            data.append(anchorData[i].dist_mean)

        f.write('{},{},{},{},{},{},{},{},{},{},{}\n'.format(*data))
    except:
        print('Error writing to file!')
    finally:
        f.close

        


if __name__ == '__main__':
    exit_code = main()