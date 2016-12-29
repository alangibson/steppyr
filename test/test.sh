#!/bin/sh -e

export PYTHONPATH=$PWD/src:$PWD/test/mock

python3 test/test_a4988.py
python3 test/test_accelstepper.py
python3 test/test_drv8825.py
python3 test/test_stepdir.py
python3 test/test_tmc26x.py
