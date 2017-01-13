from RaspberryPiStepperDriver.profiles import *

def test_calc_speed_from_rpm():
  # Given
  rpm = 100
  steps_per_rev = 200
  microsteps = 1
  # When
  speed_steps_per_sec = calc_speed_from_rpm(rpm, steps_per_rev, microsteps)
  # Then
  assert(speed_steps_per_sec == (rpm / 60) * steps_per_rev * microsteps)

def test_calc_direction():
  # Given
  forward_steps = 100
  back_steps = -100
  no_steps = 0
  # When
  forward_direction = calc_direction(forward_steps)
  back_direction = calc_direction(back_steps)
  no_direction = calc_direction(no_steps)
  # Then
  assert(DIRECTION_CW == forward_direction)
  assert(DIRECTION_CCW == back_direction)
  assert(DIRECTION_NONE == no_direction)

def test_calc_step_interval_us():
  # Given
  speed_steps_per_sec = 999
  rev_speed_steps_per_sec = -999
  # When
  step_interval_us = calc_step_interval_us(speed_steps_per_sec)
  rev_step_interval_us = calc_step_interval_us(rev_speed_steps_per_sec)
  # Then
  assert(step_interval_us == (1000000.0 / speed_steps_per_sec))
  assert(step_interval_us == (1000000.0 / abs(rev_speed_steps_per_sec)))

if __name__ == '__main__':
  test_calc_speed_from_rpm()
  test_calc_direction()
  test_calc_step_interval_us()
