import RPi.GPIO as GPIO

class Activator:
  """
  Defines the Activator API.
  """

  def __init__(self, pin_mode=GPIO.BCM, microsteps=1):
    # Minimum stepper driver pulse width in microseconds.
    # This is how long logic voltage will be applied to the STEP pin.
    self._pulse_width_us = 2
    # GPIO pin mode
    self._pin_mode = pin_mode
    # Microstep resolution
    self._microsteps = microsteps

  def start(self):
    pass

  def stop(self):
    pass

  def enable(self):
    pass

  def disable(self):
    pass

  def step(self, direction):
    """
    Unconditionally perform a step in direction.
    """
    pass

  def set_microsteps(self, microsteps):
    """
    Set microsteps for drivers that support it.
    """
    self._microsteps = microsteps

  @property
  def pulse_width(self):
    return self._pulse_width_us

  def set_pulse_width(self, pulse_width_us):
    """
    Set the step pulse width in microseconds.
    """
    self._pulse_width_us = pulse_width_us
