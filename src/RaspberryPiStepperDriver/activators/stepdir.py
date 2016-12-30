import logging
import RPi.GPIO as GPIO
from .. import DIRECTION_CW, DIRECTION_CCW, sleep_microseconds

log = logging.getLogger(__name__)

class StepDirActivator:

  def __init__(self, dir_pin, step_pin, enable_pin=None, pin_mode=GPIO.BCM):
    # Minimum stepper driver pulse width in microseconds.
    # This is how long logic voltage will be applied to the STEP pin.
    self._pulse_width_us = 2

    # Pins
    self._dir_pin = dir_pin
    self._step_pin = step_pin
    self._enable_pin = enable_pin
    self._pin_mode = pin_mode

    self._direction = None

  @property
  def pulse_width(self):
    return self._pulse_width_us

  def set_pulse_width(self, pulse_width_us):
    """
    Set the step pulse width in microseconds.
    """
    self._pulse_width_us = pulse_width_us

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

  def step(self, direction):
    """
    Unconditionally perform a step.
    """
    # log.debug('Step direction=%s pulse_width_us=%s', direction, self._pulse_width_us)
    # Set direction first else get rogue pulses
    GPIO.output(self._dir_pin, GPIO.LOW if direction == DIRECTION_CCW else GPIO.HIGH)
    GPIO.output(self._step_pin, GPIO.HIGH)
    # Caution 200ns setup time
    # Delay the minimum allowed pulse width
    sleep_microseconds(self._pulse_width_us)
    GPIO.output(self._step_pin, GPIO.LOW)
