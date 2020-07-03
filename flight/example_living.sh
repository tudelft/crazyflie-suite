#!/usr/bin/env bash

python flight/logFlight.py \
    --fileroot . \
    --logconfig flight/logcfg.json \
    --space flight/space_living.yaml \
    --estimator kalman \
    --uwb twr \
    --trajectory scan \
