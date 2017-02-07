from steppyr.profiles.accel import *

def test_set_target_speed():
  # Given
  speed_steps_per_sec = 100
  profile = AccelProfile()
  # When
  profile.set_target_speed(100)
  profile._ramp_step_number = 10
  # Then
  profile._target_speed = 100
  profile._ramp_delay_min_us = 1000000.0 / speed_steps_per_sec
  profile._ramp_step_number != 0

def test_calc_ramp_step_number_16():
  # Given
  current_speed = 10
  acceleration = 5
  # When
  steps_to_go = calc_ramp_step_number_16(current_speed, acceleration)
  # Then
  print('test_calc_ramp_step_number_16 steps_to_go=', steps_to_go)
  assert(steps_to_go == 10)

def test_calc_ramp_step_number_17():
  pass

def test_calc_ramp_delay_0():
  pass

if __name__ == '__main__':
  test_set_target_speed()
  test_calc_ramp_step_number_16()
