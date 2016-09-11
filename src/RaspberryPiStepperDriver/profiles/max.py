import logging

log = logging.getLogger(__name__)

# HACK put this somewhere common to accelstepper too
DIRECTION_CCW = 0   # Clockwise
DIRECTION_CW  = 1   # Counter-Clockwise

def constrain(value, minn, maxn):
    return max(min(maxn, value), minn)

class MaxProfile:
  def __init__(self, acceleration_steps=0, max_start_speed=0.0):
    self._last_direction = DIRECTION_CCW
    # Number of steps to take to go from min start speed to target speed
    self._acceleration_steps = acceleration_steps
    # Max speed that the motor can start at in steps per second
    self._max_start_speed = max_start_speed

  def init(self, parent):
    self.parent = parent

  def set_target_speed(self, speed):
    """
    Set our requested ultimate cruising speed.

    Arguments:
      speed (float): Steps per second
    """
    if speed == self.parent._target_speed:
      return
    self.parent._target_speed = speed
    self.compute_new_speed()
    log.debug('Set speed %s _step_interval_us %s _direction %s',
      speed, self.parent._step_interval_us, self.parent._direction )
    return speed

  def set_acceleration(self, acceleration):
    pass

  def compute_new_speed(self):

    # Determine direction
    # HACK don't change anything if distance = 0 because we get a default value from calc_direction()
    if self.parent.distance_to_go == 0:
      # We are at our destination. Nothing more to do
      self.parent._current_speed = 0.0      
      self.parent._step_interval_us = 0
      return
    else:
      self._last_direction = self.parent._direction
      self.parent._direction = self.parent.calc_direction(self.parent.distance_to_go)

    # Calculate the rate in steps at which we should accelerate
    if self._acceleration_steps <= 0:
      _acceleration_increment = self.parent._target_speed
    else:
      _acceleration_increment = (self.parent._target_speed - self._max_start_speed) / self._acceleration_steps

    # Calculations are simpler if we use the absolute value of the current speed
    _current_speed = abs(self.parent._current_speed)

    # Determine our speed
    # If we changed direction, start ramp over
    if self._last_direction != self.parent._direction:
      # This is our first step from stop
      # log.debug('Resetting _current_speed to %s', self._max_start_speed)
      _current_speed = self._max_start_speed
    else:
      if _current_speed < self.parent._target_speed:
        # We are accelerating
        _current_speed = constrain(_current_speed + _acceleration_increment,
          self._max_start_speed, self.parent._target_speed)

    # Adjust for signed-ness direction
    if self.parent._direction == DIRECTION_CCW:
      _current_speed = -_current_speed
    self.parent._current_speed = _current_speed

    # Calculate the next step interval
    self.parent._step_interval_us = self.parent.calc_step_interval_us(self.parent._current_speed)

    log.debug('Computed new speed. _direction=%s, _current_steps=%s, _target_steps=%s, distance_to_go=%s, _current_speed=%s, _step_interval_us=%s _acceleration_increment=%s _target_speed=%s',
      self.parent._direction, self.parent._current_steps,
      self.parent._target_steps, self.parent.distance_to_go,
      self.parent._current_speed, self.parent._step_interval_us,
      _acceleration_increment, self.parent._target_speed)

  def set_current_position(self, position):
    self.parent._target_steps = self.parent._current_steps = position
