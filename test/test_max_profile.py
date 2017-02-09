import unittest
from steppyr.profiles.max import *

class TestSuite(unittest.TestCase):
  
  def test_constrain(self):
    assert(constrain(999, 10, 100) == 100)
    assert(constrain(1, 10, 100) == 10)

  def test_calc_steps_being_moved_1(self):
    # Given
    target_steps = 100
    previous_target_steps = -100
    # When
    steps_being_moved = calc_steps_being_moved(target_steps, previous_target_steps)
    # Then
    assert(steps_being_moved == 200)

  def test_calc_steps_being_moved_3(self):
    # Given
    target_steps = -100
    previous_target_steps = -100
    # When
    steps_being_moved = calc_steps_being_moved(target_steps, previous_target_steps)
    # Then
    assert(steps_being_moved == 0)

  def test_calc_steps_being_moved_4(self):
    # Given
    target_steps = 100
    previous_target_steps = 50
    # When
    steps_being_moved = calc_steps_being_moved(target_steps, previous_target_steps)
    # Then
    assert(steps_being_moved == 50)

  def test_calc_steps_being_moved_2(self):
    # Given
    target_steps = 100
    previous_target_steps = 50
    # When
    steps_being_moved = calc_steps_being_moved(target_steps, previous_target_steps)
    # Then
    assert(steps_being_moved == 50)

  def test_adjust_deceleration_steps_overlap_0(self):
    """ Overlapping accel and decel ramps """
    # Given
    acceleration_steps = 10
    deceleration_steps = 10
    steps_being_moved = 10
    # When
    deceleration_steps = adjust_deceleration_steps(acceleration_steps, deceleration_steps, steps_being_moved)
    # Then
    assert(deceleration_steps == 0)

  def test_adjust_deceleration_steps_overlap_1(self):
    """ Overlapping accel and decel ramps """
    # Given
    acceleration_steps = 6
    deceleration_steps = 6
    steps_being_moved = 10
    # When
    deceleration_steps = adjust_deceleration_steps(acceleration_steps, deceleration_steps, steps_being_moved)
    # Then
    assert(deceleration_steps == 4)

  def test_is_accelerating_true(self):
    # Given
    steps_being_moved = 20
    distance_to_go = 16
    acceleration_steps = 5
    # When
    is_accel = is_accelerating(steps_being_moved, distance_to_go, acceleration_steps)
    # Then
    assert(is_accel == True)

  def test_is_accelerating_false(self):
    # Given
    steps_being_moved = 20
    distance_to_go = 14
    acceleration_steps = 5
    # When
    is_accel = is_accelerating(steps_being_moved, distance_to_go, acceleration_steps)
    # Then
    assert(is_accel == False)

  def test_is_accelerating_negative(self):
    # Given
    steps_being_moved = 20
    distance_to_go = -16
    acceleration_steps = 5
    # When
    is_accel = is_accelerating(steps_being_moved, distance_to_go, acceleration_steps)
    # Then
    assert(is_accel == True)

  def test_is_decelerating_1(self):
    # Given
    distance_to_go = 3
    deceleration_steps = 5
    # When
    is_decel = is_decelerating(distance_to_go, deceleration_steps)
    # Then
    assert(is_decel == True)

  def test_is_decelerating_2(self):
    # Given
    distance_to_go = 6
    deceleration_steps = 5
    # When
    is_decel = is_decelerating(distance_to_go, deceleration_steps)
    # Then
    assert(is_decel == False)

  def test_is_decelerating_3(self):
    # Given
    distance_to_go = -3
    deceleration_steps = 5
    # When
    is_decel = is_decelerating(distance_to_go, deceleration_steps)
    # Then
    assert(is_decel == True)

  def test_is_decelerating_4(self):
    # Given
    distance_to_go = 10
    deceleration_steps = 0
    # When
    is_decel = is_decelerating(distance_to_go, deceleration_steps)
    # Then
    assert(is_decel == False)

  def test_compute_new_speed_0(self):
    """ Speed is always 0 when there are no steps to take """
    # Given
    profile = MaxProfile(acceleration_steps=5, max_start_speed=100.0)
    profile._current_speed = 999
    profile._step_interval_us = 999
    profile._target_steps = 0
    profile._current_steps = 0
    # When
    profile.compute_new_speed()
    # Then
    assert(profile._current_speed == 0.0)
    assert(profile._step_interval_us == 0)
    assert(profile._direction == DIRECTION_CCW)

  def test_compute_new_speed_1(self):
    """ Test hexagonal ramp """
    # Given
    # ( target_speed - max_start_speed) / acceleration_steps == 100
    target_speed = 1000
    acceleration_steps = 10
    deceleration_steps = 5
    max_start_speed = 100.0
    profile = MaxProfile(acceleration_steps=acceleration_steps, max_start_speed=max_start_speed, deceleration_steps=deceleration_steps)
    profile._current_speed = 0
    # profile._step_interval_us = 0
    profile._target_steps = 20
    profile._current_steps = 0
    # When
    profile.set_target_speed(target_speed)
    # called by set_target_speed(self): profile.compute_new_speed()
    # Then
    print(profile._current_speed, profile._step_interval_us)
    # In this profile, first step is taken at max_start_speed
    assert(profile._current_speed == max_start_speed)
    # Computing this value is tested elsewhere
    assert(profile._step_interval_us != 0)
    assert(profile._direction == DIRECTION_CW)

    # Last acceleration step
    profile._current_steps = 5
    profile._current_speed = 400
    profile.compute_new_speed()
    print(profile._current_speed, profile._step_interval_us)
    assert(profile._current_speed == 490)

    # In middle of ramp running at full speed
    profile._current_steps = 10
    profile._current_speed = 1000
    profile.compute_new_speed()
    print('middle', profile._current_speed, profile._step_interval_us)
    assert(profile._current_speed == 1000)

    # First step of deceleration phase
    profile._current_steps = 16
    profile._current_speed = 1000
    profile.compute_new_speed()
    print('First step of deceleration phase', profile._current_speed, profile._step_interval_us)
    assert(profile._current_speed == 800)
    assert(profile._step_interval_us != 0)

    # Stopped
    profile._current_steps = 20
    profile._current_speed = 100 # should never happen
    profile.compute_new_speed()
    print(profile._current_speed, profile._step_interval_us)
    assert(profile._current_speed == 0)
    assert(profile._step_interval_us == 0)

  def test_acceleration_increment_steps_0(self):
    # Given
    acceleration_steps = 0
    target_speed_steps_per_sec = 1000
    # When
    acceleration_increment_steps = calc_acceleration_increment_steps(
      acceleration_steps, target_speed_steps_per_sec, None)
    # Then
    assert(acceleration_increment_steps == target_speed_steps_per_sec)

  def test_acceleration_increment_steps(self):
    # Given
    acceleration_steps = 10
    target_speed_steps_per_sec = 1000
    max_start_speed = 100
    # When
    acceleration_increment_steps = calc_acceleration_increment_steps(
      acceleration_steps, target_speed_steps_per_sec, max_start_speed)
    # Then
    assert(acceleration_increment_steps == ( target_speed_steps_per_sec - max_start_speed) / acceleration_steps)

if __name__ == '__main__':
  unittest.main()
