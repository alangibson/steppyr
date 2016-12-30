import logging
from . import RampProfile, DIRECTION_CW, DIRECTION_CCW

log = logging.getLogger(__name__)

def constrain(value, minn, maxn):
    return max(min(maxn, value), minn)

class MaxProfile(RampProfile):
  def __init__(self, acceleration_steps=0, max_start_speed=0.0):
    super().__init__()

    self._last_direction = DIRECTION_CCW
    # Number of steps to take to go from min start speed to target speed
    self._acceleration_steps = acceleration_steps
    # Max speed that the motor can start at in steps per second
    self._max_start_speed = max_start_speed
    self._deceleration_steps = 5

  def set_target_speed(self, speed):
    """
    Set our requested ultimate cruising speed.

    Arguments:
      speed (float): Steps per second
    """
    if speed == self._target_speed:
      return
    self._target_speed = speed
    self.compute_new_speed()
    log.debug('Set speed %s _step_interval_us %s _direction %s',
      speed, self._step_interval_us, self._direction )
    return speed

  def set_acceleration(self, acceleration):
    pass

  def compute_new_speed(self):

    # Determine direction
    # HACK don't change anything if distance = 0 because we get a default value from calc_direction()
    if self.distance_to_go == 0:
      # We are at our destination. Nothing more to do
      self._current_speed = 0.0
      self._step_interval_us = 0
      return
    else:
      self._last_direction = self._direction
      self._direction = self.calc_direction(self.distance_to_go)

    # Make sure accelerate and decelerate ramps don't overlap
    _steps_being_moved = abs(abs(self._target_steps) - abs(self.parent._previous_target_steps))
    _adjusted_deceleration_steps = min(self._deceleration_steps, _steps_being_moved - self._acceleration_steps)
    # HACK fix the math so this conditional is not needed
    if _adjusted_deceleration_steps <= 0:
      _adjusted_deceleration_steps = 1

    # Calculate the rate in steps at which we should accelerate
    if self._acceleration_steps <= 0:
      _acceleration_increment = self._target_speed
    else:
      _acceleration_increment = (self._target_speed - self._max_start_speed) / self._acceleration_steps
    # Calculate the rate in steps at which we should decelerate
    if self._deceleration_steps <= 0:
      _deceleration_increment = self._target_speed
    else:
      _deceleration_increment = self._target_speed / _adjusted_deceleration_steps

    # Calculations are simpler if we use the absolute value of the current speed
    _current_speed = abs(self._current_speed)

    # Determine our speed
    # If we changed direction, start ramp over
    if self._last_direction != self._direction:
      # This is our first step from stop, which is a special case due to _max_start_speed.
      _current_speed = self._max_start_speed
    elif (_steps_being_moved - abs(self.distance_to_go)) <= self._acceleration_steps:
        # We are accelerating
        _current_speed = constrain(_current_speed + _acceleration_increment,
          self._max_start_speed, self._target_speed)
      # else: we are cruising
    elif abs(self.distance_to_go) < _adjusted_deceleration_steps:
      # We are within _acceleration_steps of the end point. Start decelerating.
      _current_speed = _current_speed - _deceleration_increment

    # Adjust for signed-ness direction
    if self._direction == DIRECTION_CCW:
      _current_speed = -_current_speed
    self._current_speed = _current_speed

    # Calculate the next step interval
    self._step_interval_us = self.calc_step_interval_us(self._current_speed)

    log.debug('Computed new speed. _direction=%s, _current_steps=%s, _target_steps=%s, distance_to_go=%s, _current_speed=%s, _step_interval_us=%s _acceleration_increment=%s _target_speed=%s',
      self._direction, self._current_steps,
      self._target_steps, self.distance_to_go,
      self._current_speed, self._step_interval_us,
      _acceleration_increment, self._target_speed)
    #log.debug('_acceleration_increment %s _deceleration_increment %s _adjusted_deceleration_steps %s',
    #  _acceleration_increment,_deceleration_increment, _adjusted_deceleration_steps)
    # log.debug('_steps_being_moved %s _adjusted_deceleration_steps %s', _steps_being_moved, _adjusted_deceleration_steps)

  def set_current_position(self, position):
    self._target_steps = self._current_steps = position
