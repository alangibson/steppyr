import asyncio, logging

log = logging.getLogger(__name__)

DIRECTION_NONE = -1 # Stationary
DIRECTION_CCW = 0   # Clockwise
DIRECTION_CW  = 1   # Counter-Clockwise

def calc_degrees_to_steps(degrees, motor_steps_per_rev, microsteps):
  return degrees * motor_steps_per_rev * microsteps / 360

class StepperController:
  """
  This API is based on AccelStepper (http://www.airspayce.com/mikem/arduino/AccelStepper).
  """

  def __init__(self, driver, profile):
    self._driver = driver
    self._profile = profile

  def activate(self):
    """
    Must be called before using this driver.
    Implementing classes should use this method to initialize the hardware.
    """
    self._driver.activate()

  def shutdown(self):
    """
    Must be called when this driver is no longer needed.
    Implementing classes should use this method to deactivate the hardware.
    """
    self._driver.shutdown()

  def stop(self):
    """
    Immediately stop the current move.
    """
    self._profile.stop()

  def move(self, relative_steps):
    """
    Schedules move to a number of steps relative to the current step count.
    TODO copied from AccelStepper
    """
    self.move_to(self._profile.current_steps + relative_steps)

  def move_to(self, absolute_steps):
    """
    Schedules move to an absolute number of steps.
    """
    self._profile.set_target_steps(absolute_steps)

  def rotate(self, degrees):
    """
    Rotate motor a given number of degrees.
    """
    self.move(calc_degrees_to_steps(degrees, self._profile.full_steps_per_rev, self._profile.microsteps))

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
    You must call this at least once per step.
    If the motor is in the desired position, the cost is very small.
    returns true if the motor is still running to the target position.
    """
    # Dont do anything unless we actually have a step interval
    # and dont do anything unless we have somewhere to go
    if self._profile.should_step():
      # It is time to do a step

      # Tell the activator to take a step in a given direction
      self._driver.step(self._profile.direction)

      # Tell profile we are taking a step
      self._profile.step()

    # else: Do nothing
    return self._profile.is_moving

  async def wait_on_move(self):
    """
    'blocks' until is_moving == False.
    Only use this method if you know there are no other calls to move() happening,
    or this method may never return. For example: during calibration at startup.
    """
    while self._profile.is_moving:
      await asyncio.sleep(0)

  def next_steps_to_go(self, target_steps):
    """
    Convenience function for any code that may want to know how many steps we will go
    """
    return target_steps - self._profile.current_steps

  def next_direction(self, target_steps):
    """
    Convenience function for any code that may want to know what direction we would travel
    """
    return DIRECTION_CW if self.next_steps_to_go(target_steps) > 0 else DIRECTION_CCW

  @property
  def activator(self):
    return self._driver

  @property
  def profile(self):
    return self._profile

  #
  # Proxy methods
  #

  def step(self, direction):
    self._driver.step(self._profile.direction)

  def set_microsteps(self, microsteps):
    self._driver.set_microsteps(microsteps)
    self._profile.set_microsteps(microsteps)

  def set_target_speed(self, speed):
    """
    Set our requested ultimate cruising speed.

    Arguments:
      speed (float): Steps per second
    """
    self._profile.set_target_speed(speed)

  @property
  def target_speed(self):
    return self._profile.target_speed

  @property
  def current_speed(self):
    return self._profile.current_speed

  def set_current_steps(self, position):
    """
    Useful during initialisations or after initial positioning
    Sets speed to 0
    """
    self._profile.set_current_steps(position)

  @property
  def current_steps(self):
    return self._profile.current_steps

  @property
  def direction(self):
    return self._profile.direction

  @property
  def steps_to_go(self):
    return self._profile.steps_to_go

  @property
  def is_moving(self):
    return self._profile.is_moving

  @property
  def current_acceleration(self):
    return self._profile.current_acceleration

  def set_target_acceleration(self, acceleration):
    """
    Sets acceleration value in steps per second per second and computes new speed.
    Arguments:
      acceleration (float). Acceleration in steps per second per second.
    """
    self._profile.set_target_acceleration(acceleration)

  @property
  def target_acceleration(self):
    return self._profile.target_acceleration

  def set_pulse_width(self, pulse_width_us):
    """
    Set the step pulse width in microseconds.
    """
    self._driver.set_pulse_width(pulse_width_us)
