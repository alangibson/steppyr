import time
import RPi.GPIO as GPIO

print GPIO.VERSION

def micros():
  """
  Mymics the Arduino micros() function
  """
  return time.time() * float(1000000)

def micro_wait_until(target_micros):
  """
  A sleep function that does a busy loop to try to get microsecond accuracy.
  """
  start = micros()
  while micros() < target_micros:
    pass
  print 'waited', micros() - start

def sleep_microseconds(us_to_sleep):
  time.sleep(us_to_sleep / float(1000000))

class BasicStepperDriver:

  MAX_MICROSTEP = 128  
  VALID_MICROSTEPS = [1, 16, 32, 64, 128]

  def __init__(self, motor_steps, dir_pin, step_pin, enable_pin=None):
    self.microsteps = 1
    self.rpm = None
    self.motor_steps = motor_steps
    self.dir_pin = dir_pin
    self.step_pin = step_pin
    self.enable_pin = enable_pin
    self.init()

  def init(self):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(self.dir_pin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(self.step_pin, GPIO.OUT, initial=GPIO.LOW)
    self.set_microstep(1)
    self.set_rpm(60)
    self.enable()

  def set_rpm(self, rpm):
    self.rpm = rpm
    self.calc_step_pulse()

  #
  # calculate the step pulse in microseconds for a given rpm value.
  # 60[s/min] * 1000000[us/s] / microsteps / steps / rpm
  #
  def calc_step_pulse(self):
    if self.motor_steps and self.microsteps and self.rpm:
      self.step_pulse = 60 * 1000000 / self.motor_steps / self.microsteps / self.rpm
      print 'calculated step pulse', self.step_pulse
    # TODO else error?

  def set_microstep(self, microsteps):
    if microsteps in self.VALID_MICROSTEPS:
      self.microsteps = microsteps
    # TODO else error
    self.calc_step_pulse()
    return self.microsteps

  def set_direction(self, direction):
    """
    DIR: forward HIGH, reverse LOW
    """
    logic_value = GPIO.LOW if direction < 0 else GPIO.HIGH
    print 'setting direction', logic_value
    GPIO.output(self.dir_pin, logic_value)

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
    steps = steps * direction # so steps is always positive
    pulse_duration = self.step_pulse / 2

    print 'direction', direction, 'steps', steps, 'pulse duration', pulse_duration

    while steps > 0:
      # next_edge = micros() + pulse_duration
      # print 'doing step', steps, 'next edge', next_edge
      GPIO.output(self.step_pin, GPIO.HIGH)
      # micro_wait_until(next_edge)
      sleep_microseconds(pulse_duration)
      GPIO.output(self.step_pin, GPIO.LOW)
      sleep_microseconds(pulse_duration)
      steps -= 1

  def rotate(self, degrees):
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
