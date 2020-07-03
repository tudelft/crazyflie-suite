#!/usr/bin/env bash

# Check whether pwd is project root
if [[ "${PWD##*/}" != crazyflie-suite ]]; then
    echo "Should be run from project root, exiting"
    exit 1
fi

# Run
python flight/logFlight.py \
    --fileroot data \
    --logconfig flight/logcfg.json \
    --space flight/space_living.yaml \
    --estimator kalman \
    --uwb twr \
    --trajectory scan \
