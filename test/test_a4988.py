import unittest
from steppyr.drivers.a4988 import A4988Driver
import RPi.GPIO as GPIO

GPIO.DEBUG = True

class TestSuite(unittest.TestCase):
  def test_1(self):
    activator = A4988Driver(dir_pin=1, step_pin=2,
                            ms1_pin=3, ms2_pin=4, ms3_pin=5)

    activator.set_microsteps(0)
    activator.set_microsteps(1)
    activator.set_microsteps(2)
    activator.set_microsteps(4)
    activator.set_microsteps(8)
    activator.set_microsteps(16)

if __name__ == '__main__':
  unittest.main()
