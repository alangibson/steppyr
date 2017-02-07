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
    register_value = 0b1101111111
    rep = Representation(0, 7) # an 8 bit int
    value = rep.from_register_value(register_value)
    self.assertEqual(value, 127)

  def test_like_bv(self):
    """Test that Representation covers case of _BV, ie picking out bit flags"""
    rep = Representation(3) #
    self.assertTrue(rep.from_register_value(0b01000))
    self.assertFalse(rep.from_register_value(0b00100))

  def test_mismatch(self):
    """Test when first and second pair of values dont agree"""
    rep = Representation(0, 31, 24, 0, True)
    register_value = 0b11111111111111111111111111111111 # signed value 4294967295
    value = rep.from_register_value(register_value)
    self.assertEqual(value, -1)
    # 0b11111110111111111111111111111111 # signed decoded value 4278190079

if __name__ == '__main__':
  unittest.main()
