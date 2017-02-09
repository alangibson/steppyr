from steppyr import DIRECTION_CW, DIRECTION_CCW
from steppyr.lib.functions import micros
from . import RampProfile, calc_speed_from_step_interval
import logging, math

log = logging.getLogger(__name__)

class AccelProfile(RampProfile):
  """
  Calculates AccelStepper profile.
  Based on AccelStepper (http://www.airspayce.com/mikem/arduino/AccelStepper).
  """

  def __init__(self):
    super().__init__()
    # Precomputed sqrt(2*_acceleration)
    self._sqrt_twoa = 1.0
    # Logical step number of current ramp. Not the same as physical step count.
    # 'n' in the Austin paper
    # Used for calculating acceleration and deceleration.
    # When +ve, we are accelerating; when -ve, we are decelerating.
    self._ramp_step_number = 0
    # _timer_count is 'c' in Austin paper
    # Defaults to 1.0 to avoid divide by zero errors
    self._ramp_delay_0_us = 1.0
    # All timer counts > _ramp_delay_0_us. _timer_count is 'c' in Austin paper
    # Defaults to 1.0 to avoid divide by zero errors
    self._ramp_delay_n_us = 1.0
    # Minimum microseconds for ramp delay
    self._ramp_delay_min_us = 1.0

  def set_target_speed(self, speed):
    """
    Set our requested ultimate cruising speed.

    Arguments:
      speed (float): Steps per second
    """
    if self._target_speed == speed:
      return
    self._target_speed = speed
    # TODO can we move the following block into compute_new_speed() ?
    self._ramp_delay_min_us = 1000000.0 / speed
    if self._ramp_step_number > 0:
      self._ramp_step_number = calc_ramp_step_number_16(self._current_speed, self._target_acceleration)
      self.compute_new_speed()

  def set_target_acceleration(self, acceleration):
    """
    Sets acceleration value in steps per second per second and computes new speed.
    Arguments:
      acceleration (float). Acceleration in steps per second per second.
    """
    if acceleration == 0.0 or self._target_acceleration == acceleration:
      return
    # Recompute _ramp_step_number per Equation 17
    self._ramp_step_number = calc_ramp_step_number_17(self._ramp_step_number, self._target_acceleration, acceleration)
    # New c0 per Equation 7, with correction per Equation 15
    self._ramp_delay_0_us = calc_ramp_delay_0(acceleration)
    self._target_acceleration = acceleration
    self._current_acceleration = acceleration
    self.compute_new_speed()

  def compute_new_speed(self):
    # Save distance to go
    distanceTo = self.steps_to_go
    # stepsToStop = int(((self._current_speed * self._current_speed) / (2.0 * self._target_acceleration))) # Equation 16
    # Get number of steps until stop per Equation 16
    stepsToStop = int(calc_ramp_step_number_16(self._current_speed , self._target_acceleration))

    if distanceTo == 0 and stepsToStop <= 1:
      # We are at the target and its time to stop
      # self._step_interval_us = 0
      # self._current_speed = 0.0
      # self._ramp_step_number = 0
      self.stop()
      self.set_current_steps(0)
      return

    if distanceTo > 0:
      # We are anticlockwise from the target
      # Need to go clockwise from here, maybe decelerate now
      if self._ramp_step_number > 0:
        # Currently accelerating, need to decel now? Or maybe going the wrong way?
        if (stepsToStop >= distanceTo) or self._direction == DIRECTION_CCW:
          # Start deceleration
          self._ramp_step_number = -stepsToStop
      elif self._ramp_step_number < 0:
        # Currently decelerating, need to accel again?
        if (stepsToStop < distanceTo) and self._direction == DIRECTION_CW:
          # Start accceleration
          self._ramp_step_number = -self._ramp_step_number
    elif distanceTo < 0:
      # We are clockwise from the target
      # Need to go anticlockwise from here, maybe decelerate
      if self._ramp_step_number > 0:
        # Currently accelerating, need to decel now? Or maybe going the wrong way?
        if (stepsToStop >= -distanceTo) or self._direction == DIRECTION_CW:
          # Start deceleration
          self._ramp_step_number = -stepsToStop
      elif self._ramp_step_number < 0:
        # Currently decelerating, need to accel again?
        if stepsToStop < -distanceTo and self._direction == DIRECTION_CCW:
          # Start accceleration
          self._ramp_step_number = -self._ramp_step_number

    # Need to accelerate or decelerate
    if self._ramp_step_number == 0:
      # First step from stopped
      self._ramp_delay_n_us = self._ramp_delay_0_us
      self._direction = self.direction
    else:
      # Subsequent step. Works for accel (_ramp_step_number is +_ve) and decel (_ramp_step_number is -ve).
      ramp_delay_n_us = calc_ramp_delay_n(self._ramp_delay_n_us, self._ramp_step_number)
      # Make sure we don't to below the min ramp delay
      self._ramp_delay_n_us = max(ramp_delay_n_us, self._ramp_delay_min_us)

    self._ramp_step_number += 1
    self._step_interval_us = self._ramp_delay_n_us
    self._next_step_time_us = micros() + self._step_interval_us
    self._current_speed = calc_speed_from_step_interval(self._ramp_delay_n_us)

    log.debug('Computed new speed. _direction=%s, _current_steps=%s, _target_steps=%s, distance_to_go=%s, _ramp_step_number=%s, _current_speed=%s, _step_interval_us=%s',
              self._direction, self._current_steps,
              self._target_steps, self.steps_to_go,
              self._ramp_step_number, self._current_speed, self._step_interval_us)

  def set_current_steps(self, position):
    super().set_current_steps(position)
    self._ramp_step_number = 0

def calc_ramp_step_number_16(current_speed, acceleration):
  """
  Recompute ramp step number from current speed and adjust speed if accelerating or cruising

  Equation 16
  """
  return (current_speed * current_speed) / (2.0 * acceleration)

def calc_ramp_step_number_17(ramp_step_number, old_acceleration, new_acceleration):
  """ Recompute _ramp_step_number per Equation 17 """
  return ramp_step_number * (old_acceleration / new_acceleration)

def calc_ramp_delay_0(acceleration):
  """ New c0 per Equation 7, with correction per Equation 15 """
  return 0.676 * math.sqrt(2.0 / acceleration) * 1000000.0

def calc_ramp_delay_n(ramp_delay_n_us, ramp_step_number):
  """ Equation 13 """
  return ramp_delay_n_us - ((2.0 * ramp_delay_n_us) / ((4.0 * ramp_step_number) + 1))
