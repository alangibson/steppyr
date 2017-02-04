import unittest
from RaspberryPiStepperDriver.activators.tmc4361.registers import mask, Representation

class TestRepresentation(unittest.TestCase):

  def test_float(self):
    num = 44.443355
    rep = Representation(24, 8)
    register_value = rep.to_register_value(num)
    self.assertEqual(register_value, 11377)

  def test_float_2(self):
    num = 44.443355
    rep = Representation(24, 2)
    register_value = rep.to_register_value(num)
    self.assertEqual(register_value, 177)

  def test_float_from_register_value(self):
    register_value = 11268 # already fixed-point encoded
    rep = Representation(24, 8)
    value = rep.from_register_value(register_value)
    # Note: fixed to floating point conversion is inexact
    self.assertEqual(value, 44.015625)

if __name__ == '__main__':
  unittest.main()
