#!/bin/sh -e

export PYTHONPATH=$PWD/src:$PWD/test/mock

gpio mode 7 clock
gpio clock 7 1600000

python3 test/test_accel_profile.py
python3 test/test_max_profile.py
python3 test/test_rampprofile.py
python3 test/test_stepper.py
# TODO remove or update: python3 test/test_a4988.py
python3 test/test_accelstepper.py
# TODO remove or update: python3 test/test_drv8825.py
# TODO remove or update: python3 test/test_stepdir.py
python3 test/test_tmc26x.py
