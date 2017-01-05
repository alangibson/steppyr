import logging
from .. import DIRECTION_CW, DIRECTION_CCW

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
    self._acceleration = 0.0
    # Microstep denominator ( 1/_microsteps )
    self._microsteps = 1
    # Number of full steps for motor to turn one revolution
    self._motor_steps_per_rev = 200

  def set_target_speed(self, speed):
    """
    Set our requested ultimate cruising speed.

    Arguments:
      speed (float): Steps per second
    """
    if self._target_speed == speed:
      return
    self._target_speed = speed
    log.debug('Set speed %s _step_interval_us %s _direction %s',
      speed, self._step_interval_us, self._direction )
    self.compute_new_speed()

  def set_target_rpm(self, rpm):
    """
    Set our requested ultimate cruising speed in rpm.

    Arguments:
      rpm (float): Desired revolutions per minute for motor.
      steps_per_rev: number of full steps per revolution of motor.
    """
    self.set_target_speed(calc_speed_from_rpm(rpm, steps_per_rev, self._microsteps))

  def set_acceleration(self, acceleration):
    """
    Sets acceleration value in steps per second per second and computes new speed.

    Arguments:
      acceleration (float). Acceleration in steps per second per second.
    """
    self._acceleration = acceleration
    self.compute_new_speed()

  def set_motor_steps_per_rev(self, steps_per_rev):
    self._motor_steps_per_rev = steps_per_rev

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
    """
    self._target_steps = self._current_steps = position
    self._step_interval_us = 0
    self._current_speed = 0.0

  def set_microsteps(self, microsteps):
    """
    Set the number of microsteps and compute new speed.

    TODO recalculate our position in steps based on mew microsteps.
    """
    self._microsteps = microsteps
    self.compute_new_speed()

  """
  TODO
  def set_direction(self, direction):
    ""
    Set the direction we should step() in

    direction: one of DIRECTION_CW or DIRECTION_CCW
    ""
    self._direction = direction

  @property
  def direction(self):
    return self._direction
  """

  @property
  def distance_to_go(self):
    """
    # +ve is clockwise from curent location
    """
    return self._target_steps - self._current_steps

  def _current_direction(self):
    """
    Calculates direction based on distance to go
    """
    return DIRECTION_CW if self.distance_to_go > 0 else DIRECTION_CCW

def calc_step_interval_us(self, speed):
  if speed == 0.0:
    return 0
  return abs(1000000.0 / speed)

def calc_direction(self, value):
  """
  Value can be speed or steps to go
  """
  return DIRECTION_CW if (value > 0.0) else DIRECTION_CCW

def calc_speed_from_rpm(rpm, steps_per_rev, microsteps):
  """
  Calculates speed in steps per second from revolutions per minute.

  rpm: desired revolutions per minute.
  steps_per_rev: full steps per revolution of motor.
  microsteps: number of microsteps.
  """
  rps = rpm / 60
  microsteps_per_rev = steps_per_rev * microsteps
  speed_steps_per_sec = rps * microsteps_per_rev
  return speed_steps_per_sec
