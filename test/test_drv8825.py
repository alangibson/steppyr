import unittest
import RPi.GPIO as GPIO
from steppyr.activators.drv8825 import DRV8825Activator

GPIO.DEBUG = True

class TestSuite(unittest.TestCase):
  def test(self):
    activator = DRV8825Activator(dir_pin=1, step_pin=2,
                                 ms1_pin=3, ms2_pin=4, ms3_pin=5)
    activator.set_microsteps(0)
    activator.set_microsteps(1)
    activator.set_microsteps(2)
    activator.set_microsteps(4)
    activator.set_microsteps(8)
    activator.set_microsteps(16)
    activator.set_microsteps(32)

if __name__ == '__main__':
  unittest.main()
