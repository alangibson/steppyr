import unittest
from steppyr.activators.tmc26x.registers import *

class TestSuite(unittest.TestCase):
  def test_Register_1(self):
    # Given
    header = 0b111
    register_value = 0b11111111111111111
    mask = 0b111100
    value = 0b0101
    register = Register(header=header, register_value=register_value)
    # When
    register.set(mask, value)
    # Then
    self.assertEqual(register.data, 0b11111111111010111)
    self.assertEqual(register.datagram, 0b11111111111111010111)

  def test_Register_header_len(self):
    # Given
    header = 0b10
    register_value = 0b111111111111111111
    mask = 0b111100
    value = 0b0101
    register = Register(header=header, register_value=register_value, header_len=2)
    # When
    register.set(mask, value)
    # Then
    self.assertEqual(register.data, 0b111111111111010111)
    self.assertEqual(register.datagram, 0b10111111111111010111)

  def test_Register_get(self):
    register = Register(header=0, register_value=0b11111010)
    val = register.get(0b1111)
    self.assertEqual(val, 0b1010)

if __name__ == '__main__':
  unittest.main()
