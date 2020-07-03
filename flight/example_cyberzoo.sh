#!/usr/bin/env bash

# Check whether pwd is project root
if [[ "${PWD##*/}" != crazyflie-suite ]]; then
    echo "Should be run from project root, exiting"
    exit 1
fi

# Run from project root
python flight/logFlight.py \
    --fileroot data \
    --logconfig flight/logcfg.json \
    --space flight/space_cyberzoo.yaml \
    --estimator kalman \
    --uwb twr \
    --trajectory square \
    --optitrack \
    --optitrack_id 1
