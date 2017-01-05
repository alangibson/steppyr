from datetime import datetime
import asyncio, logging, time
import RPi.GPIO as GPIO
from .activators import stepdir as stepdir_act
from . import DIRECTION_CW, DIRECTION_CCW, micros, Stepper

log = logging.getLogger(__name__)

class AccelStepper(Stepper):

  def __init__(self, profile, dir_pin, step_pin, enable_pin=None, pin_mode=GPIO.BCM):
    self._profile = profile
    # Time that this objec was instantiated. Used for computing micros()
    self._start_time = datetime.now()
    # Time in microseconds that last step occured
    self._last_step_time_us = 0
    self._activator = stepdir_act.StepDirActivator(dir_pin, step_pin, enable_pin, pin_mode)

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

  def start(self):
    self._activator.start()
    self._run_forever_future = asyncio.ensure_future(self.run_forever())

  def stop(self):
    self._run_forever_future.cancel()
