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

PUD_UP       = 1

def setmode(mode):
  if DEBUG:
    print(time.time(), 'setmode', mode)

def setup(pin, level, initial=HIGH, pull_up_down=PUD_UP):
  if DEBUG:
    print(time.time(), 'setup', pin, level, initial)

def output(pin, level):
  if DEBUG:
    print(time.time(), 'output', pin, level)

def input(pin):
    return 1

def cleanup():
  if DEBUG:
    print(time.time(), 'cleanup')
