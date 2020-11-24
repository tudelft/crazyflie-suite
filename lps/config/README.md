# Anchor Configuration
Before using an installed UWB system for localization, the anchors must be configured properly with their respective positions. These scripts are intended to help in this process if the anchors are to far apart to measure their positions accurately with measurement tapes or optitrack.

This calibration process relies on 4 seperate distance measurements from known locations (ideally as spaced out as possible) at different heights. The location of the anchor can then be found by triangulation. The ```anchor_positions.py``` script will calculate the optimal location in the least squares sense.

## Measuring the distances automatically
The ```get_distance_data.py``` script can be used to collect data automatically. Connect a UWB node to your computer (via USB) and configure it in TWR tag mode. Use ```dmesg | grep tty``` the serial port to which the tag is connected. Change the 'DEVICE' variable in the script if it is different from the identified device. Position the tag at a know location and launch the script. Enter the known position of the tag and wait for distance data to be collected (Make sure that the tag does not move during data collection). After data has been collected it will be written to the file specified at the beginning of the script.

## Manual distance measurements
If distances are collected manually, they should be written to a .csv, following the ```measurements.csv``` template. Here, (x,y,z) are the position to which the measurement was taken, and d0-d7 are the distances to anchors 0-7 respectively. If less than 8 anchors are used, leave the corresponding distances empty or set to 0.

## Anchor positions
Once the measurements have been recorded in a .csv file (automatically or manually), the ```anchor_positions.py``` script can be used to calculate the least squares solution to the anchor positions. (NOTE: Make sure the .csv file used is the same as the one you collected the data in)