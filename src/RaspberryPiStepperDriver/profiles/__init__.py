
# HACK put this somewhere common to accelstepper too
DIRECTION_CCW = 0   # Clockwise
DIRECTION_CW  = 1   # Counter-Clockwise

class RampProfile:

  def __init__(self):
    # Current velocity/speed in steps per second
    # Generally: 1000000.0 / _step_interval_us
    self._current_speed = 0.0
    # Max velocity/speed in steps per second
    # Defaults to 1 to avoid ZeroDivisionError
    self._target_speed = 1.0
    # Number of microseconds between steps
    self._step_interval_us = 0
    # Direction we are currently moving in.
    self._direction = DIRECTION_CCW
    # Current stepper position in steps.
    self._current_steps = 0
    # Last requested absolute target position in steps
    self._target_steps = 0
    # Previously set _target_steps. Used in calculating ramps
    self._previous_target_steps = 0
    # Acceleration in steps per second per second
    self._acceleration = 0.0

    # TODO self.parent.calc_direction

  def set_target_speed(self, speed):
    """
    Set our requested ultimate cruising speed.

    Arguments:
      speed (float): Steps per second
    """
    pass

  def set_acceleration(self, acceleration):
    """
    Sets acceleration value in steps per second per second and computes new speed.

    Arguments:
      acceleration (float). Acceleration in steps per second per second.
    """
    pass

  def compute_new_speed(self):
    """
    Responsible for calculating the following values.
      _step_interval_us
      _current_speed
    """
    pass

  def set_current_position(self, position):
    """
    Useful during initialisation or after initial positioning.
    Sets speed to 0.
    """
    pass

  @property
  def distance_to_go(self):
    """
    # +ve is clockwise from curent location
    """
    return self._target_steps - self._current_steps

  def calc_step_interval_us(self, speed):
    if speed == 0.0:
      return 0
    return abs(1000000.0 / speed)

  def calc_direction(self, value):
    """
    Value can be speed or steps to go
    """
    return DIRECTION_CW if (value > 0.0) else DIRECTION_CCW

  def _current_direction(self):
    return DIRECTION_CW if self.distance_to_go > 0 else DIRECTION_CCW
