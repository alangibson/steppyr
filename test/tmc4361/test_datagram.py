import unittest
from RaspberryPiStepperDriver.motion.tmc4361 import io

class TestDatagram(unittest.TestCase):

  def test_datagram_1(self):
    # Given
    header = 0b11111111
    data = 0b00000000000000000000000000000001
    # When
    datagram = io.Datagram(header, data)
    # Then
    self.assertEqual(datagram.datagram, 0b1111111100000000000000000000000000000001)
    self.assertEqual(datagram.header, header)
    self.assertEqual(datagram.data, data)
    self.assertEqual(datagram.to_list(), [0b11111111, 0b00000000, 0b00000000, 0b00000000, 0b00000001])

  def test_datagram_2(self):
    # Given
    header = 0b11111111
    data = [0b00000000, 0b00000000, 0b00000000, 0b00000001]
    # When
    datagram = io.Datagram(header, data)
    # Then
    self.assertEqual(datagram.data, 0b00000000000000000000000000000001)

if __name__ == '__main__':
  unittest.main()
