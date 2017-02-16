import logging
import RPi.GPIO as GPIO
from .. import DIRECTION_CW, DIRECTION_CCW
from steppyr.lib.functions import sleep_microseconds
from . import Driver

log = logging.getLogger(__name__)

class StepDirDriver(Driver):

  def __init__(self, dir_pin, step_pin, enable_pin=None, pin_mode=GPIO.BCM):
    super().__init__(pin_mode=pin_mode)
    # Pins
    self._dir_pin = dir_pin
    self._step_pin = step_pin
    self._enable_pin = enable_pin

  def enable(self):
    if self._enable_pin:
      GPIO.setup(self._enable_pin, GPIO.OUT, initial=GPIO.LOW)

  def disable(self):
    if self._enable_pin:
      GPIO.setup(self._enable_pin, GPIO.OUT, initial=GPIO.HIGH)

  def activate(self):
    GPIO.setmode(self._pin_mode)
    GPIO.setup(self._dir_pin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(self._step_pin, GPIO.OUT, initial=GPIO.LOW)
    self.enable()

  def shutdown(self):
    self.disable()

  def step(self, direction):
    # Set direction first else get rogue pulses
    GPIO.output(self._dir_pin, GPIO.LOW if direction == DIRECTION_CCW else GPIO.HIGH)
    GPIO.output(self._step_pin, GPIO.HIGH)
    # Caution 200ns setup time
    # Delay the minimum allowed pulse width
    sleep_microseconds(self._pulse_width_us)
    GPIO.output(self._step_pin, GPIO.LOW)
