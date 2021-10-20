#!/usr/bin/env bash

# Check whether pwd is project root
if [[ "${PWD##*/}" != crazyflie-suite ]]; then
    echo "Should be run from project root, exiting"
    exit 1
fi

# Run
python flight/log_flight.py \
    --fileroot data/20211019 \
    --filename model\
    --logconfig flight/logcfg.json \
    --space flight/space_cyberzoo.yaml \
    --estimator complementary \
    --uwb none \
    --trajectory manual\
    --safetypilot \
    --optitrack logging \
    --optitrack_id 1
