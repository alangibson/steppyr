import time

DEBUG = False

HIGH         = 1
LOW          = 0

OUTPUT, OUT  = 0, 0
INPUT, IN    = 1, 1

MODE_UNKNOWN = -1
BOARD        = 10
BCM          = 11
SERIAL       = 40
SPI          = 41
I2C          = 42
PWM          = 43

def setmode(mode):
  if DEBUG:
    print(time.time(), 'setmode', mode)

def setup(pin, level, initial=HIGH):
  if DEBUG:
    print(time.time(), 'setup', pin, level, initial)

def output(pin, level):
  if DEBUG:
    print(time.time(), 'output', pin, level)

def cleanup():
  if DEBUG:
    print(time.time(), 'cleanup')
