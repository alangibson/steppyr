import logging
from .. import DIRECTION_CW, DIRECTION_CCW, DIRECTION_NONE
from steppyr.lib.functions import micros

log = logging.getLogger(__name__)

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
    # Defaults to 1.0 to avoid divide by zero errors
    self._target_acceleration = 1.0
    # Our current acceleration in steps / second / second
    self._current_acceleration = 0.0
    # Microstep denominator ( 1/_microsteps )
    self._microsteps = 1
    # Number of full steps for motor to turn one revolution
    # This default is for a 1.8 degree stepper
    self._full_steps_per_rev = 200
    # Time in microseconds that last step occured
    self._last_step_time_us = 0
    self._next_step_time_us = None

  def stop(self):
    """
    Reset speed to 0.
    """
    self._step_interval_us = 0
    self._current_speed = 0.0

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

  @property
  def target_speed(self):
    return self._target_speed

  @property
  def current_speed(self):
    return self._current_speed

  def set_target_rpm(self, rpm):
    """
    Set our requested ultimate cruising speed in rpm.

    Arguments:
      rpm (float): Desired revolutions per minute for motor.
      steps_per_rev: number of full steps per revolution of motor.
    """
    self.set_target_speed(calc_speed_from_rpm(rpm, self._full_steps_per_rev, self._microsteps))

  def set_target_steps(self, absolute_steps):
    if self._target_steps != absolute_steps:
      self._previous_target_steps = self._target_steps
      self._target_steps = absolute_steps
      self.compute_new_speed()

  def set_target_acceleration(self, acceleration):
    """
    Sets acceleration value in steps per second per second and computes new speed.

    Arguments:
      acceleration (float). Acceleration in steps per second per second.
    """
    self._target_acceleration = acceleration
    self.compute_new_speed()

  @property
  def target_acceleration(self):
    return self._target_acceleration

  @property
  def current_acceleration(self):
    return self._current_acceleration

  def set_full_steps_per_rev(self, steps_per_rev):
    self._full_steps_per_rev = steps_per_rev

  @property
  def full_steps_per_rev(self):
    return self._full_steps_per_rev

  # FIXME compute _next_step_time_us
  def compute_new_speed(self):
    """
    Responsible for calculating the following values.
      _step_interval_us
      _current_speed
      _next_step_time_us
    """
    pass

  def set_current_steps(self, position):
    """
    Useful during initialisation or after initial positioning.
    """
    self._target_steps = self._current_steps = position

  def set_microsteps(self, microsteps):
    """
    Set the number of microsteps and compute new speed.

    TODO recalculate our position in steps based on mew microsteps.
    """
    self._microsteps = microsteps
    self.compute_new_speed()

  @property
  def microsteps(self):
    return self._microsteps

  def step(self):
    """
    Register a step in the current direction.
    This method is for drivers to notify the profile that they are taking a step.
    """
    if self.direction == DIRECTION_CW:
      self._current_steps += 1
    elif self.direction == DIRECTION_CCW:
      self._current_steps -= 1
    # else: do nothing
    self.compute_new_speed()
    self._last_step_time_us = micros()

  def should_step(self):
    """
    Returns true if we should take a step.
    """
    if not self.step_interval_us or not self.steps_to_go:
      return False
    return (self._next_step_time_us and micros() >= self._next_step_time_us)

  @property
  def steps_to_go(self):
    """
    # +ve is clockwise from curent location
    """
    return self._target_steps - self._current_steps

  @property
  def current_steps(self):
    return self._current_steps

  @property
  def is_moving(self):
    return self.steps_to_go != 0

  @property
  def direction(self):
    """
    Calculates direction based on distance to go
    """
    # return DIRECTION_CW if self.distance_to_go > 0 else DIRECTION_CCW
    return calc_direction(self.steps_to_go)

  @property
  def step_interval_us(self):
    return self._step_interval_us

def calc_step_interval_us(speed):
  """
  Calculate step interval in microseconds based on speed in steps per second.
  """
  if speed == 0.0:
    return 0
  return abs(1000000.0 / speed)

def calc_direction(value):
  """
  Value can be speed or steps to go
  """
  if value > 0:
    return DIRECTION_CW
  elif value < 0:
    return DIRECTION_CCW
  else:
    return DIRECTION_NONE

def calc_speed_from_rpm(rpm, full_steps_per_rev, microsteps):
  """
  Calculates speed in steps per second from revolutions per minute.

  rpm: desired revolutions per minute.
  steps_per_rev: full steps per revolution of motor.
  microsteps: number of microsteps.
  """
  rps = rpm / 60
  microsteps_per_rev = full_steps_per_rev * microsteps
  speed_steps_per_sec = rps * microsteps_per_rev
  return speed_steps_per_sec

def calc_speed_from_step_interval(step_interval_us):
  return abs(1000000.0 / step_interval_us)
