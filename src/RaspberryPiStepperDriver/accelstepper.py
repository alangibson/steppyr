"""
Based on AccelStepper (http://www.airspayce.com/mikem/arduino/AccelStepper).
Ported to Python by Alan Gibson.
"""
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

    profile.parent = self
    self._profile = profile

    # Time that this objec was instantiated. Used for computing micros()
    self._start_time = datetime.now()
    # Current stepper position in steps.
    self._current_steps = 0
    # Last requested absolute target position in steps
    self._target_steps = 0
    # Current velocity/speed in steps per second
    self._speed = 0.0
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
    # return self._speed != 0.0 or self.distance_to_go != 0
    return self.distance_to_go != 0

  def set_pulse_width(self, pulse_width_us):
    self._pulse_width_us = pulse_width_us

  #def set_speed(self, speed):
    """
    Arguments:
      speed (float): Speed/velocity in steps per second.
    """
    """
    if speed == self._speed:
      return
    speed = constrain(speed, -self._target_speed, self._target_speed)
    if speed == 0.0:
      self._step_interval_us = 0
    else:
      self._step_interval_us = abs(1000000.0 / speed)
      self._direction = DIRECTION_CW if (speed > 0.0) else DIRECTION_CCW
    self._speed = speed
    """
    #algo.set_speed(self, speed)

  def set_target_speed(self, speed):
    """
    Set our requested ultimate cruising speed.

    Arguments:
      speed (float): Steps per second
    """
    """
    if self._target_speed == speed:
      return
    self._target_speed = speed
    self._ramp_delay_min_us = 1000000.0 / speed
    # Recompute _ramp_step_number from current speed and adjust speed if accelerating or cruising
    if (self._ramp_step_number > 0):
      self._ramp_step_number = ((self._speed * self._speed) / (2.0 * self._acceleration)) # Equation 16
      self._compute_new_speed()
    """
    self._profile.set_target_speed(speed)

  def set_target_speed_from_motor(self, motor_steps_per_rev, motor_rpm):
    steps_per_sec = (motor_steps_per_rev * motor_rpm) / 60
    self.set_target_speed(steps_per_sec)

  def set_acceleration(self, acceleration):
    """
    Sets acceleration value in steps per second per second and computes new speed.
    Arguments:
      acceleration (float). Acceleration in steps per second per second.
    """
    """
    if acceleration == 0.0 or self._acceleration == acceleration:
      return
    # Recompute _ramp_step_number per Equation 17
    self._ramp_step_number = self._ramp_step_number * (self._acceleration / acceleration)
    # New c0 per Equation 7, with correction per Equation 15
    self._ramp_delay_0_us = 0.676 * math.sqrt(2.0 / acceleration) * 1000000.0 # Equation 15
    self._acceleration = acceleration
    self._compute_new_speed()
    """
    self._profile.set_acceleration(acceleration)

  def _compute_new_speed(self):
      """
      distanceTo = self.distance_to_go     # +ve is clockwise from curent location
      stepsToStop = int(((self._speed * self._speed) / (2.0 * self._acceleration))) # Equation 16

      if distanceTo == 0 and stepsToStop <= 1:
        # We are at the target and its time to stop
        self._step_interval_us = 0
        self._speed = 0.0
        self._ramp_step_number = 0
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
        self._direction = DIRECTION_CW if distanceTo > 0 else DIRECTION_CCW
      else:
        # Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
        self._ramp_delay_n_us = self._ramp_delay_n_us - ((2.0 * self._ramp_delay_n_us) / ((4.0 * self._ramp_step_number) + 1)) # Equation 13
        self._ramp_delay_n_us = max(self._ramp_delay_n_us, self._ramp_delay_min_us)

      self._ramp_step_number += 1
      self._step_interval_us = self._ramp_delay_n_us
      self._speed = 1000000.0 / self._ramp_delay_n_us
      if self._direction == DIRECTION_CCW:
        self._speed = -self._speed

      log.debug('Computed new speed. _direction=%s, _current_steps=%s, _target_steps=%s, distance_to_go=%s, _ramp_step_number=%s, _speed=%s, _step_interval_us=%s',
        self._direction, self._current_steps, self._target_steps, self.distance_to_go, self._ramp_step_number, self._speed, self._step_interval_us)
      """
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
    return abs(1000000.0 / speed)

  def calc_direction(self, speed):
    return DIRECTION_CW if (speed > 0.0) else DIRECTION_CCW
