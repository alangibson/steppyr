from datetime import datetime
import asyncio, logging, time
import RPi.GPIO as GPIO

log = logging.getLogger(__name__)

DIRECTION_CCW = 0   # Clockwise
DIRECTION_CW  = 1   # Counter-Clockwise

def constrain(value, minn, maxn):
    return max(min(maxn, value), minn)

def sleep_microseconds(us_to_sleep):
  time.sleep(us_to_sleep / float(1000000))

class AccelStepper:

  def __init__(self, profile, dir_pin, step_pin, enable_pin=None, pin_mode=GPIO.BCM):
    # Send a reference to this object to the profile
    profile.init(self)
    self._profile = profile
    # Time that this objec was instantiated. Used for computing micros()
    self._start_time = datetime.now()
    # Current stepper position in steps.
    self._current_steps = 0
    # Last requested absolute target position in steps
    self._target_steps = 0
    # Current velocity/speed in steps per second
    self._current_speed = 0.0
    #  Max velocity/speed in steps per second
    self._target_speed = 1.0
    # Number of microseconds between steps
    self._step_interval_us = 0
    # Minimum stepper driver pulse width in microseconds.
    # This is how long logic voltage will be applied to the STEP pin.
    self._pulse_width_us = 1
    # Direction we are currently moving in.
    self._direction = DIRECTION_CCW
    # Time in microseconds that last step occured
    self._last_step_time_us = 0

    # Pins
    self._dir_pin = dir_pin
    self._step_pin = step_pin
    self._enable_pin = enable_pin
    self._pin_mode = pin_mode

  @property
  def currentPosition(self):
    return self._current_steps

  # HACK backwards compatability
  @property
  def step_counter(self):
    return self._current_steps

  @property
  def direction(self):
    return self._direction

  @property
  def pulse_width(self):
    return self._pulse_width_us

  @property
  def acceleration(self):
    return self._acceleration

  @property
  def distance_to_go(self):
    return self._target_steps - self._current_steps

  # HACK backwards compatability
  @property
  def steps_to_move(self):
    return self._target_steps - self._current_steps

  @property
  def is_moving(self):
    # return self._current_speed != 0.0 or self.distance_to_go != 0
    return self.distance_to_go != 0

  def set_pulse_width(self, pulse_width_us):
    self._pulse_width_us = pulse_width_us

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

  def _compute_new_speed(self):
      self._profile.compute_new_speed()

  def move_to(self, absolute_steps):
    """
    Move to an absolute number of steps.
    """
    if self._target_steps != absolute_steps:
      self._target_steps = absolute_steps
      self._compute_new_speed()

  def move(self, relative_steps):
    """
    Move a number of steps relative to the current step count.
    """
    self.move_to(self._current_steps + relative_steps)

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
      self._compute_new_speed()
    return self.is_moving

  async def run_at_speed(self):
    """
    Implements steps according to the current step interval.
    Differs from run() in that it does not call _compute_new_speed().
    You must call this at least once per step.
    Returns true if a step occurred.
    """
    # Dont do anything unless we actually have a step interval
    if not self._step_interval_us:
      return False
    # Dont do anything unless we have somewhere to go
    if not self.distance_to_go:
      return False

    # Time since this class was created in micrseconds.
    # Basically the Arduino micros() function.
    time_us = (datetime.now() - self._start_time).total_seconds() * 1000000

    if (time_us - self._last_step_time_us) >= self._step_interval_us:
      # It is time to do a step
      if self._direction == DIRECTION_CW:
        # Clockwise
        self._current_steps += 1
      else:
        # Anticlockwise
        self._current_steps -= 1
      self.step(self._current_steps)

      self._last_step_time_us = time_us
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

  def step(self, step):
    """
    Perform a step. Currently only supports STEP/DIR type stepper drivers.
    """
    # Set direction first else get rogue pulses
    GPIO.output(self._dir_pin, GPIO.LOW if self._direction == DIRECTION_CCW else GPIO.HIGH)
    GPIO.output(self._step_pin, GPIO.HIGH)
    # Caution 200ns setup time
    # Delay the minimum allowed pulse width
    sleep_microseconds(self._pulse_width_us)
    GPIO.output(self._step_pin, GPIO.LOW)

  def set_current_position(self, position):
    """
    Useful during initialisations or after initial positioning
    Sets speed to 0
    """
    self._profile.set_current_position(position)

  def abort(self):
    self.set_current_position(self._current_steps)

  def reset_step_counter(self):
    self.set_current_position(0)

  def enable(self):
    if self._enable_pin:
      GPIO.setup(self._enable_pin, GPIO.OUT, initial=GPIO.LOW)

  def disable(self):
    if self._enable_pin:
      GPIO.setup(self._enable_pin, GPIO.OUT, initial=GPIO.HIGH)

  def start(self):
    GPIO.setmode(self._pin_mode)
    GPIO.setup(self._dir_pin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(self._step_pin, GPIO.OUT, initial=GPIO.LOW)
    self.enable()
    self._run_forever_future = asyncio.ensure_future(self.run_forever())

  def stop(self):
    self._run_forever_future.cancel()

  def predict_distance_to_go(self, target_steps):
    """
    Convenience function for any code that may want to know how many steps we will go
    """
    return target_steps - self._current_steps

  def predict_direction(self, target_steps):
    """
    Convenience function for any code that may want to know what direction we would travel
    """
    return DIRECTION_CW if self.predict_distance_to_go(target_steps) > 0 else DIRECTION_CCW

  def calc_step_interval_us(self, speed):
    if speed == 0.0:
      return 0
    return abs(1000000.0 / speed)

  def calc_direction(self, value):
    """
    Value can be speed or steps to go
    """
    return DIRECTION_CW if (value > 0.0) else DIRECTION_CCW
