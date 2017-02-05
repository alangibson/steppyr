import unittest
from RaspberryPiStepperDriver.activators.tmc4361.registers import mask, Representation

class TestRepresentation(unittest.TestCase):

  def test_float(self):
    num = 44.443355
    rep = Representation(0, 0, 24, 8)
    register_value = rep.to_register_value(num)
    self.assertEqual(register_value, 11377)

  def test_float_2(self):
    num = 44.443355
    rep = Representation(0, 0, 24, 2)
    register_value = rep.to_register_value(num)
    self.assertEqual(register_value, 177)

  def test_float_from_register_value(self):
    register_value = 11268 # already fixed-point encoded
    rep = Representation(0, 31, 24, 8)
    value = rep.from_register_value(register_value)
    # Note: fixed to floating point conversion is inexact
    self.assertEqual(value, 44.015625)

  def test_like_mask(self):
    register_value = 0b1111111111 # 10 bits
    rep = Representation(0, 7) # an 8 bit int
    value = rep.from_register_value(register_value)
    self.assertEqual(value, 255)

  def test_like_bv(self):
    """Test that Representation covers case of _BV, ie picking out bit flags"""
    rep = Representation(3) #
    self.assertTrue(rep.from_register_value(0b01000))
    self.assertFalse(rep.from_register_value(0b00100))

if __name__ == '__main__':
  unittest.main()
