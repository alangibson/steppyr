import logging
from steppyr import DIRECTION_CW, DIRECTION_CCW, DIRECTION_NONE
from steppyr.lib.functions import micros
from . import RampProfile, calc_direction, calc_step_interval_us

log = logging.getLogger(__name__)

def constrain(value, minn, maxn):
    return max(min(maxn, value), minn)

class MaxProfile(RampProfile):
  def __init__(self, acceleration_steps=0, max_start_speed=0.0, deceleration_steps=0):
    super().__init__()
    self._last_direction = DIRECTION_NONE
    # Number of steps to take to go from min start speed to target speed
    self._acceleration_steps = acceleration_steps
    self._deceleration_steps = deceleration_steps
    # Max speed that the motor can start at in steps per second
    self._max_start_speed = max_start_speed

  def compute_new_speed(self):
    # Determine direction
    # HACK don't change anything if distance = 0 because we get a default value from calc_direction()
    if self.steps_to_go == 0:
      # We are at our destination. Nothing more to do
      self._current_speed = 0.0
      self._step_interval_us = 0
      return
    else:
      self._last_direction = self._direction
      self._direction = calc_direction(self.steps_to_go)

    # Make sure accelerate and decelerate ramps don't overlap
    steps_being_moved = calc_steps_being_moved(self._target_steps, self._previous_target_steps)
    adjusted_deceleration_steps = min(self._deceleration_steps, steps_being_moved - self._acceleration_steps)
    # HACK fix the math so this conditional is not needed
    #if adjusted_deceleration_steps <= 0:
    #  adjusted_deceleration_steps = 1

    # Calculate the rate in steps at which we should accelerate
    acceleration_increment_steps = calc_acceleration_increment_steps(
      self._acceleration_steps, self._target_speed, self._max_start_speed)

    # Calculate the rate in steps at which we should decelerate
    deceleration_increment_steps = calc_deceleration_increment_steps(adjusted_deceleration_steps, self._target_speed)

    # Calculations are simpler if we use the absolute value of the current speed
    abs_current_speed = abs(self._current_speed)

    # Determine our speed
    # If we changed direction, start ramp over
    if self._last_direction != self._direction:
      # This is our first step from stop, which is a special case due to _max_start_speed.
      abs_current_speed = self._max_start_speed
    # elif (steps_being_moved - abs(self.distance_to_go)) <= self._acceleration_steps:
    elif is_accelerating(steps_being_moved, self.steps_to_go, self._acceleration_steps):
        # We are accelerating
        print(abs_current_speed, acceleration_increment_steps, self._max_start_speed, self._target_speed)
        abs_current_speed = constrain(abs_current_speed + acceleration_increment_steps,
          self._max_start_speed, self._target_speed)
    # elif abs(self.distance_to_go) < adjusted_deceleration_steps:
    elif is_decelerating(self.steps_to_go, adjusted_deceleration_steps):
      # We are within _acceleration_steps of the end point. Start decelerating.
      abs_current_speed = abs_current_speed - deceleration_increment_steps
    # else: We are cruising

    # Adjust for signed-ness direction
    if self._direction == DIRECTION_CCW:
      abs_current_speed = -abs_current_speed
    self._current_speed = abs_current_speed

    # Calculate the next step interval
    self._step_interval_us = calc_step_interval_us(self._current_speed)
    self._next_step_time_us = micros() + self._step_interval_us

    log.debug('Computed new speed. _direction=%s, _current_steps=%s, _target_steps=%s, distance_to_go=%s, _current_speed=%s, _step_interval_us=%s acceleration_increment_steps=%s deceleration_increment_steps=%s _target_speed=%s',
              self._direction, self._current_steps,
              self._target_steps, self.steps_to_go,
              self._current_speed, self._step_interval_us,
              acceleration_increment_steps, deceleration_increment_steps, self._target_speed)

def calc_acceleration_increment_steps(acceleration_steps, target_speed_steps_per_sec, max_start_speed):
  """ Calculate the rate in steps at which we should accelerate """
  if acceleration_steps <= 0:
    # No acceleration steps to take, so just to go max target speed
    acceleration_increment_steps = target_speed_steps_per_sec
  else:
    acceleration_increment_steps = ( target_speed_steps_per_sec - max_start_speed) / acceleration_steps
  return acceleration_increment_steps

def calc_deceleration_increment_steps(deceleration_steps, target_speed):
  """ Calculate the rate in steps at which we should decelerate """
  if deceleration_steps <= 0:
    # No decceleration steps to take, so just assume max target speed
    deceleration_increment_steps = target_speed
  else:
    deceleration_increment_steps = target_speed / deceleration_steps
  return deceleration_increment_steps

def calc_steps_being_moved(target_steps, previous_target_steps):
  """
  Absolute number of steps being moved. Never negative.
  """
  return abs(target_steps- previous_target_steps)

def adjust_deceleration_steps(acceleration_steps, deceleration_steps, steps_being_moved):
  return min(deceleration_steps, steps_being_moved - acceleration_steps)

def is_accelerating(steps_being_moved, distance_to_go, acceleration_steps):
  return (steps_being_moved - abs(distance_to_go)) <= acceleration_steps

def is_decelerating(distance_to_go, deceleration_steps):
  return abs(distance_to_go) < deceleration_steps
