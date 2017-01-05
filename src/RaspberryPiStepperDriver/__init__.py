import asyncio, logging, time

log = logging.getLogger(__name__)

DIRECTION_CCW = 0   # Clockwise
DIRECTION_CW  = 1   # Counter-Clockwise

def sleep_microseconds(us_to_sleep):
  time.sleep(us_to_sleep / float(1000000))

def constrain(value, minn, maxn):
  return max(min(maxn, value), minn)

def micros():
  """
  Mymics the Arduino micros() function.
  """
  return int(time.time() * 1000000)

class Stepper:

  def start(self):
    """
    Must be called before using this driver.
    Implementing classes should use this method to initialize the hardware.
    """
    pass

  def stop(self):
    """
    Must be called when this driver is no longer needed.
    Implementing classes should use this method to deactivate the hardware.
    """
    pass

  def abort(self):
    """
    Immediately stop the current move.
    """
    self.set_current_position(self._profile._current_steps)

  def move(self, relative_steps):
    """
    Schedules move to a number of steps relative to the current step count.
    TODO copied from AccelStepper
    """
    move_to_steps = self._profile._current_steps + relative_steps
    self.move_to(move_to_steps)

  def move_to(self, absolute_steps):
    """
    Schedules move to an absolute number of steps.
    """
    if self._profile._target_steps != absolute_steps:
      self._profile._previous_target_steps = self._profile._target_steps
      self._profile._target_steps = absolute_steps
      self._profile.compute_new_speed()

  def rotate(self, degrees):
    """
    Rotate motor a given number of degrees.
    """
    self.move(calc_degrees_to_steps(degrees, self._profile._motor_steps_per_rev, self._profile._microsteps))

  @property
  def is_moving(self):
    return self._profile.distance_to_go != 0

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
    while self.is_moving:
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
    return self.is_moving

  async def run_at_speed(self):
    """
    Implements steps according to the current step interval.
    Move 1 step if it is time for another step. Otherwise, do nothing.
    Differs from run() in that it does not call compute_new_speed().
    You must call this at least once per step.
    Returns True if a step occurred, False otherwise.
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

  def reset_step_counter(self):
    self.set_current_position(0)

  def set_current_position(self, position):
    """
    Useful during initialisations or after initial positioning
    Sets speed to 0
    """
    self._profile.set_current_position(position)

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

  @property
  def position(self):
    return self._profile._current_steps

  @property
  def direction(self):
    return self._profile._direction

  @property
  def is_moving(self):
    return self._profile.distance_to_go != 0

  @property
  def distance_to_go(self):
    return self._profile.distance_to_go

  #
  # Activator proxy methods
  #

  def step(self, direction):
    self._activator.step(self._profile._direction)

  def set_pulse_width(self, pulse_width_us):
    """
    Set the step pulse width in microseconds.
    """
    self._activator.set_pulse_width(pulse_width_us)

def calc_degrees_to_steps(degrees, motor_steps_per_rev, microsteps):
  return degrees * motor_steps_per_rev * microsteps / 360
