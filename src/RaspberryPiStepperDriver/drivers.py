import asyncio, logging
from RaspberryPiStepperDriver import DIRECTION_CW, DIRECTION_CCW, micros

log = logging.getLogger(__name__)

def calc_degrees_to_steps(degrees, motor_steps_per_rev, microsteps):
  return degrees * motor_steps_per_rev * microsteps / 360

class StepperDriver:
  """
  This API is based on AccelStepper (http://www.airspayce.com/mikem/arduino/AccelStepper).
  """

  def __init__(self, activator, profile):
    self._activator = activator
    self._profile = profile
    # Time in microseconds that last step occured
    self._last_step_time_us = 0
    self._next_step_time_us = None

  def start(self):
    """
    Must be called before using this driver.
    Implementing classes should use this method to initialize the hardware.
    """
    self._activator.start()
    # self._run_forever_future = asyncio.ensure_future(self.run_forever())

  def stop(self):
    """
    Must be called when this driver is no longer needed.
    Implementing classes should use this method to deactivate the hardware.
    """
    self._activator.stop()
    # self._run_forever_future.cancel()

  def abort(self):
    """
    Immediately stop the current move.
    """
    self._profile.set_current_position(self._profile._current_steps)

  def move(self, relative_steps):
    """
    Schedules move to a number of steps relative to the current step count.
    TODO copied from AccelStepper
    """
    self.move_to(self._profile._current_steps + relative_steps)

  def move_to(self, absolute_steps):
    """
    Schedules move to an absolute number of steps.
    """
    self._profile.set_target_steps(absolute_steps)
    # If we don't have a next_step_time_us, we will never take a step in the run* loop
    self._next_step_time_us = micros() + self._profile._step_interval_us

  def rotate(self, degrees):
    """
    Rotate motor a given number of degrees.
    """
    self.move(calc_degrees_to_steps(degrees, self._profile._motor_steps_per_rev, self._profile._microsteps))

  async def run_forever(self):
    """
    Continuously call run() as fast as possible.
    """
    while True:
      await self.run()
      # Without this await, we never yield back to the event loop
      await asyncio.sleep(0)

  async def run_until_done(self):
    """
    Blockingly calls run() until is_move == False
    """
    while self._profile.is_moving:
      await self.run()
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
    return self._profile.is_moving

  async def run_at_speed(self):
    """
    Implements steps according to the current step interval.
    Move 1 step if it is time for another step. Otherwise, do nothing.
    Differs from run() in that it does not call compute_new_speed().
    You must call this at least once per step.
    Returns True if a step occurred, False otherwise.
    """
    # Dont do anything unless we actually have a step interval
    # and dont do anything unless we have somewhere to go
    if not self._profile._step_interval_us or not self._profile.distance_to_go:
      return False

    # Save the current time in microseconds
    current_time_us = micros()

    # Note: because we save _next_step_time_us, a new _step_interval_us won't
    # be taken into account until after the next step is done. We could also
    # use this logic to constantly recalculate the next step time:
    # next_step_time_us = self._last_step_time_us + self._profile._step_interval_us
    # if current_time_us >= next_step_time_us:

    if self._next_step_time_us and current_time_us >= self._next_step_time_us:
      # It is time to do a step

      #log.debug('self._profile._target_steps=%s self._profile.distance_to_go=%s self._profile._direction=%s self._profile._current_steps=%s',
      #  self._profile._target_steps, self._profile.distance_to_go, self._profile._direction, self._profile._current_steps)

      # This is where we increment the profile's step counter and direction.
      if self._profile._direction == DIRECTION_CW:
        # Clockwise
        self._profile._current_steps += 1
      else:
        # Counter-Clockwise
        self._profile._current_steps -= 1

      # Tell the activator to take a step in a given direction
      self._activator.step(self._profile._direction)

      #log.debug('Taking a step. current_time_us=%s next_step_time_us=%s drift us=%s',
      #  current_time_us, self._next_step_time_us, current_time_us - self._next_step_time_us)

      self._last_step_time_us = current_time_us
      self._next_step_time_us = current_time_us + self._profile._step_interval_us
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
    while self._profile.is_moving:
      await asyncio.sleep(0)

  def reset_step_counter(self):
    self._profile.set_current_position(0)

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

  def set_microsteps(self, microsteps):
    self._activator.set_microsteps(microsteps)
    # recalculate the stepping delay by simply setting the speed again
    self._profile.compute_new_speed()

  @property
  def activator(self):
    return self._activator

  @property
  def profile(self):
    return self._profile

  #
  # Proxy profile methods
  #

  def set_target_speed(self, speed):
    """
    Set our requested ultimate cruising speed.

    Arguments:
      speed (float): Steps per second
    """
    self._profile.set_target_speed(speed)

  def set_current_position(self, position):
    """
    Useful during initialisations or after initial positioning
    Sets speed to 0
    """
    self._profile.set_current_position(position)

  @property
  def position(self):
    return self._profile._current_steps

  @property
  def direction(self):
    return self._profile._direction

  @property
  def distance_to_go(self):
    return self._profile.distance_to_go

  @property
  def is_moving(self):
    return self._profile.is_moving

  @property
  def acceleration(self):
    return self._profile._acceleration

  def set_acceleration(self, acceleration):
    """
    Sets acceleration value in steps per second per second and computes new speed.
    Arguments:
      acceleration (float). Acceleration in steps per second per second.
    """
    self._profile.set_acceleration(acceleration)

  #
  # Activator proxy methods
  #

  def step(self, direction):
    log.debug('stepping: current_steps=%s direction=%s', self._profile._current_steps, self._profile._direction)
    self._activator.step(self._profile._direction)

  def set_pulse_width(self, pulse_width_us):
    """
    Set the step pulse width in microseconds.
    """
    self._activator.set_pulse_width(pulse_width_us)

  def enable(self):
    """ Enable hardware (possibly temporarily) """
    self._activator.enable()

  def disable(self):
    """ Disable hardware (possibly temporarily) """
    self._activator.disable()
