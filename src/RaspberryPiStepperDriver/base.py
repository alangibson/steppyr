import time
import RPi.GPIO as GPIO

# Motor rotation direction
CLOCKWISE = 1
COUNTERCLOCKWISE = -1

# Motor driver state
ENABLED = 1
DISABLED = -1

def sleep_microseconds(us_to_sleep):
  # print('sleeping for seconds', us_to_sleep / float(1000000))
  time.sleep(us_to_sleep / float(1000000))

class StepperDriver:
  """
  Basic driver for all stepper drivers with DIR and STEP pins.
  """

  def __init__(self, motor_steps, dir_pin, step_pin,
               enable_pin=None, pin_mode=GPIO.BOARD, microsteps=1, rpm=60):
    """
    Parameters:
        motor_steps: number of steps per revolution
        dir_pin: pin number of DIR pin
        step_pin: pin number of STEP pin
        enable_pin: pin number of ENABLE pin
    """
    self.motor_steps = motor_steps
    self.dir_pin = dir_pin
    self.step_pin = step_pin
    self.enable_pin = enable_pin
    self.microsteps = microsteps
    self.rpm = rpm
    self._calc_step_pulse_us()

    GPIO.setmode(pin_mode)
    GPIO.setup(dir_pin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(step_pin, GPIO.OUT, initial=GPIO.LOW)

  def _calc_step_pulse_us(self):
    """
    Calculate the step pulse in microseconds for a given rpm value.
    60[s/min] * 1000000[us/s] / microsteps / steps / rpm
    """
    if self.motor_steps and self.microsteps and self.rpm:
      self.step_pulse_us = 60 * 1000000 / self.motor_steps / self.microsteps / self.rpm
      print('base.StepperDriver calculated step pulse', self.step_pulse_us,
            'motor_steps', self.motor_steps, 'microsteps', self.microsteps,
            'rpm', self.rpm)
    # TODO else error?

  def move(self, steps):
    """
    Move the motor a given number of steps.
    positive to move forward, negative to reverse
    """
    if steps == 0:
      return
    direction = 1 if steps > 0 else -1
    self.set_direction(direction)

    # We currently try to do a 50% duty cycle so it's easy to see.
    # Other option is step_high_min, pulse_duration-step_high_min.
    steps_to_move = steps * direction # so steps is always positive
    pulse_duration_us = self.step_pulse_us / 2

    print('base.StepperDriver direction', direction, 'steps', steps,
          'step_pulse_us', self.step_pulse_us, 'pulse_duration_us', pulse_duration_us)

    while steps_to_move > 0:
      #print('steps', steps, 'steps_to_move', steps_to_move)
      GPIO.output(self.step_pin, GPIO.HIGH)
      sleep_microseconds(pulse_duration_us)
      GPIO.output(self.step_pin, GPIO.LOW)
      sleep_microseconds(pulse_duration_us)
      steps_to_move -= 1

  def rotate(self, degrees):
    """
    Rotate motor a given number of degrees. Set the motor direction with
    set_direction.
    """
    steps = degrees * self.motor_steps * self.microsteps / 360
    self.move(steps)

  def enable(self):
    if self.enable_pin:
      GPIO.setup(self.enable_pin, GPIO.OUT, initial=GPIO.LOW)
    # else error

  def disable(self):
    if self.enable_pin:
      GPIO.setup(self.enable_pin, GPIO.OUT, initial=GPIO.HIGH)
    # else error

  def set_state(self, state):
    if state == ENABLED:
      self.enable()
    elif state == DISABLED:
      self.disable()

  def set_rpm(self, rpm):
    self.rpm = rpm
    self._calc_step_pulse_us()

  def set_microstep(self, microsteps):
    self.microsteps = microsteps
    self._calc_step_pulse_us()

  def set_direction(self, direction):
    """
    Counter-clockwise when direction < 0.
    Clockwise when direction > 0.
    Only applies to rotate(). move() direction is set by passing positive or
    negative numbers.

    Direction: HIGH = forward, LOW = reverse
    """
    logic_value = GPIO.LOW if direction < 0 else GPIO.HIGH
    GPIO.output(self.dir_pin, logic_value)
