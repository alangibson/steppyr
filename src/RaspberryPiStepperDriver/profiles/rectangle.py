from . import RampProfile

"""
Extracted from tmc26x driver
"""

class RectangleProfile(RampProfile):

  def __init__(self):
    super().__init__()

  def set_target_speed(self, speed):
    """
    Set our requested ultimate cruising speed.

    Arguments:
      speed (float): Steps per second
    """
    if self._target_speed == speed:
      return
    self._target_speed = speed
    self.compute_new_speed()

  def set_acceleration(self, acceleration):
    # NOOP for rectangle profile. Just recompute new speed.
    self.compute_new_speed()

  def compute_new_speed(self):
    # This is the original rpm based way
    # us_per_min = 60 * 1000000
    # steps_per_min = self._steps_per_rev * self.speed_rpm * self.microsteps
    # self._step_interval_us = us_per_min / steps_per_min

    self._step_interval_us = 1000000 / self._target_speed
    # Derive current speed from _step_interval_us
    self._current_speed = 1000000.0 / self._step_interval_us
    self._direction = self._current_direction()

  def set_current_position(self, position):
    self._target_steps = self._current_steps = position
    self._step_interval_us = 0
    self._current_speed = 0.0
