#!/usr/bin/env bash

python flight/logFlight.py \
    --fileroot . \
    --logconfig flight/logcfg.json \
    --space flight/space_cyberzoo.yaml \
    --estimator kalman \
    --uwb twr \
    --trajectory square \
    --optitrack \
    --optitrack_id 1
