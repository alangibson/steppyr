from steppyr.lib.functions import micros
from . import RampProfile, calc_step_interval_us, calc_speed_from_step_interval, calc_direction

"""
Extracted from tmc26x driver
"""

class RectangleProfile(RampProfile):

  def __init__(self):
    super().__init__()

  def compute_new_speed(self):
    # This is the original rpm based way
    # us_per_min = 60 * 1000000
    # steps_per_min = self._steps_per_rev * self.speed_rpm * self.microsteps
    # self._step_interval_us = us_per_min / steps_per_min

    self._step_interval_us = calc_step_interval_us(self._target_speed)
    # Derive current speed from _step_interval_us
    self._current_speed = calc_speed_from_step_interval(self._step_interval_us)
    self._direction = calc_direction(self.steps_to_go)
    self._next_step_time_us = micros() + self._step_interval_us

  """
  TODO implment this rpm based way also
  def _calc_step_pulse_us(self):
    "
    Calculate the step pulse in microseconds for a given rpm value.
    60[s/min] * 1000000[us/s] / microsteps / steps / rpm
    "
    if self.motor_steps and self.microsteps and self.rpm:
      self.step_pulse_us = 60 * 1000000 / self.motor_steps / self.microsteps / self.rpm
      log.debug('base.StepperController calculated step pulse %s us, motor_steps %s, microsteps %s, rpm %s',
        self.step_pulse_us, self.motor_steps, self.microsteps, self.rpm)
      # We currently try to do a 50% duty cycle so it's easy to see.
      # Other option is step_high_min, pulse_duration-step_high_min.
      self.pulse_duration_us = self.step_pulse_us / 2
  """
