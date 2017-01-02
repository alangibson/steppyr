from datetime import datetime
import asyncio, logging, time
import RPi.GPIO as GPIO
from .activators import stepdir as stepdir_act
from . import DIRECTION_CW, DIRECTION_CCW, micros

log = logging.getLogger(__name__)

class AccelStepper:

  def __init__(self, profile, dir_pin, step_pin, enable_pin=None, pin_mode=GPIO.BCM):
    self._profile = profile
    # Time that this objec was instantiated. Used for computing micros()
    self._start_time = datetime.now()
    # Time in microseconds that last step occured
    self._last_step_time_us = 0
    self._activator = stepdir_act.StepDirActivator(dir_pin, step_pin, enable_pin, pin_mode)

  @property
  def position(self):
    return self._profile._current_steps

  @property
  def direction(self):
    return self._profile._direction

  @property
  def acceleration(self):
    return self._profile._acceleration

  @property
  def is_moving(self):
    return self._profile.distance_to_go != 0

  @property
  def distance_to_go(self):
    return self._profile.distance_to_go

  def set_pulse_width(self, pulse_width_us):
    """
    Set the step pulse width in microseconds.
    """
    # self._pulse_width_us = pulse_width_us
    self._activator.set_pulse_width(pulse_width_us)

  def set_target_speed(self, speed):
    """
    Set our requested ultimate cruising speed.

    Arguments:
      speed (float): Steps per second
    """
    self._profile.set_target_speed(speed)

  def set_acceleration(self, acceleration):
    """
    Sets acceleration value in steps per second per second and computes new speed.
    Arguments:
      acceleration (float). Acceleration in steps per second per second.
    """
    self._profile.set_acceleration(acceleration)

  def move_to(self, absolute_steps):
    """
    Schedules move to an absolute number of steps.
    """
    if self._profile._target_steps != absolute_steps:
      self._profile._previous_target_steps = self._profile._target_steps
      self._profile._target_steps = absolute_steps
      self._profile.compute_new_speed()

  def move(self, relative_steps):
    """
    Schedules move to a number of steps relative to the current step count.
    """
    self.move_to(self._profile._current_steps + relative_steps)

  async def run_forever(self):
    """
    Continuously call run() as fast as possible.
    """
    while True:
      await self.run()
      # Without this await, we never yield back to the event loop
      await asyncio.sleep(0)

  async def run(self):
    """
    Run the motor to implement speed and acceleration in order to proceed to the target position
    You must call this at least once per step, preferably in your main loop
    If the motor is in the desired position, the cost is very small
    returns true if the motor is still running to the target position.
    """
    if await self.run_at_speed():
      self._profile.compute_new_speed()
    return self.is_moving

  async def run_at_speed(self):
    """
    Implements steps according to the current step interval.
    Differs from run() in that it does not call compute_new_speed().
    You must call this at least once per step.
    Returns true if a step occurred.
    """
    # Dont do anything unless we actually have a step interval
    # Dont do anything unless we have somewhere to go
    if not self._profile._step_interval_us or not self._profile.distance_to_go:
      return False

    # Time since this class was created in micrseconds.
    # Basically the Arduino micros() function.
    # time_us = (datetime.now() - self._start_time).total_seconds() * 1000000
    current_time_us = micros()

    next_step_time_us = self._last_step_time_us + self._profile._step_interval_us
    # if (current_time_us - self._last_step_time_us) >= self._profile._step_interval_us:
    if current_time_us >= next_step_time_us:
      # It is time to do a step

      if self._profile._direction == DIRECTION_CW:
        # Clockwise
        self._profile._current_steps += 1
      else:
        # Anticlockwise
        self._profile._current_steps -= 1
      self.step(self._profile._direction)

      self._last_step_time_us = current_time_us
      # self._next_step_time_us = current_time_us + self._step_interval_us
      return True
    else:
      # No step necessary at this time
      return False

  async def wait_on_move(self):
    """
    'blocks' until is_moving == False.
    Only use this method if you know there are no other calls to move() happening,
    or this method may never return. For example: during calibration at startup.
    """
    while self.is_moving:
      await asyncio.sleep(0)

  def step(self, direction):
    self._activator.step(self._profile._direction)

  def set_current_position(self, position):
    """
    Useful during initialisations or after initial positioning
    Sets speed to 0
    """
    self._profile.set_current_position(position)

  def abort(self):
    self.set_current_position(self._profile._current_steps)

  def reset_step_counter(self):
    self.set_current_position(0)

  # def enable(self):
  #   if self._enable_pin:
  #     GPIO.setup(self._enable_pin, GPIO.OUT, initial=GPIO.LOW)

  # def disable(self):
  #   if self._enable_pin:
  #     GPIO.setup(self._enable_pin, GPIO.OUT, initial=GPIO.HIGH)

  def start(self):
    # GPIO.setmode(self._pin_mode)
    # GPIO.setup(self._dir_pin, GPIO.OUT, initial=GPIO.HIGH)
    # GPIO.setup(self._step_pin, GPIO.OUT, initial=GPIO.LOW)
    # self.enable()
    self._activator.start()
    self._run_forever_future = asyncio.ensure_future(self.run_forever())
    
  def stop(self):
    """
    Shutdown the stepper, driver chip, etc.
    """
    self._run_forever_future.cancel()

  def predict_distance_to_go(self, target_steps):
    """
    Convenience function for any code that may want to know how many steps we will go
    """
    return target_steps - self._profile._current_steps

  def predict_direction(self, target_steps):
    """
    Convenience function for any code that may want to know what direction we would travel
    """
    return DIRECTION_CW if self.predict_distance_to_go(target_steps) > 0 else DIRECTION_CCW
