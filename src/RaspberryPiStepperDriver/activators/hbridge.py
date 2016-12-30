import RPi.GPIO as GPIO
import time

class Adafruit350ma:
  
  enable_pin = 16
  enable2_pin = 12
  in1_pin = 18
  in2_pin = 22
  in3_pin = 24
  in4_pin = 26

  steps_per_second = 600
  delay = 1.0 / steps_per_second / 4
  print(delay)
  # delay = 0.00055
 
  forward_full_steps = [
    {in1_pin: GPIO.HIGH, in2_pin: GPIO.LOW, in3_pin: GPIO.LOW, in4_pin: GPIO.HIGH},
    {in1_pin: GPIO.LOW, in2_pin: GPIO.HIGH, in3_pin: GPIO.LOW, in4_pin: GPIO.HIGH},
    {in1_pin: GPIO.LOW, in2_pin: GPIO.HIGH, in3_pin: GPIO.HIGH, in4_pin: GPIO.LOW},
    {in1_pin: GPIO.HIGH, in2_pin: GPIO.LOW, in3_pin: GPIO.HIGH, in4_pin: GPIO.LOW},
  ]

  reverse_full_steps = [
    {in1_pin: GPIO.HIGH, in2_pin: GPIO.LOW, in3_pin: GPIO.HIGH, in4_pin: GPIO.LOW},
    {in1_pin: GPIO.LOW, in2_pin: GPIO.HIGH, in3_pin: GPIO.HIGH, in4_pin: GPIO.LOW},
    {in1_pin: GPIO.LOW, in2_pin: GPIO.HIGH, in3_pin: GPIO.LOW, in4_pin: GPIO.HIGH},
    {in1_pin: GPIO.HIGH, in2_pin: GPIO.LOW, in3_pin: GPIO.LOW, in4_pin: GPIO.HIGH},
  ]

  def __init__(self):
    GPIO.cleanup() 
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(self.enable_pin, GPIO.OUT)
    GPIO.setup(self.enable2_pin, GPIO.OUT)
    GPIO.setup(self.in1_pin, GPIO.OUT)
    GPIO.setup(self.in2_pin, GPIO.OUT)
    GPIO.setup(self.in3_pin, GPIO.OUT)
    GPIO.setup(self.in4_pin, GPIO.OUT)

  def forward(self):
    """
    Do a forward step sequence
    """
    for step in self.forward_full_steps:
      for key, value in step.iteritems():
        print(key, value)
        GPIO.output(key, value)
        time.sleep(self.delay)

  def reverse(self):
    for step in self.reverse_full_steps:
      for key, value in step.iteritems():
        GPIO.output(key, value)
        time.sleep(self.delay)

  def enable(self):
    GPIO.output(self.enable_pin, GPIO.HIGH)
    GPIO.output(self.enable2_pin, GPIO.HIGH)

stepper = Adafruit350ma()

try:
  stepper.enable()
  while True:
    cmd = raw_input("Command, f or r:")
    direction = cmd[0]
    if direction == "f":
      while True:
        stepper.forward()
    else: 
      while True:
        stepper.reverse()
except Exception as e:
  print(e)
finally:
  GPIO.cleanup() 

