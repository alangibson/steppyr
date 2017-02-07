import unittest
from steppyr.activators.tmc4361.io import Status, Datagram
from steppyr.activators.tmc4361.registers import \
  StatusEventRegister, SpiStatusSelectionRegister

class TestIo(unittest.TestCase):
  def test_1(self):
    # Given
    status_code = 0b11
    spi_status_selection_register = SpiStatusSelectionRegister(0)
    spi_status_selection_register.set(StatusEventRegister.bits.VACTUAL_EQ_0)
    spi_status_selection_register.set(StatusEventRegister.bits.POS_COMP_REACHED)
    # When
    status_obj = Status(status_code, spi_status_selection_register)
    status_set = status_obj.get_values()
    # Then
    self.assertEqual(len(status_set), 2)

  def test_datagram_as_response(self):
    request_datagram = Datagram(0b110, 0b110011)
    # In real usage we would send via spi here
    status_code = 0b1010
    response_data = [0b111, 0b000, 0b111, 0b000]
    response_datagram = request_datagram.as_response([status_code]+response_data)
    self.assertEqual(response_datagram.status, status_code)

  def test_datagram_register(self):
    """
    Ensure header changes, but register doesnt, when setting read or write
    """
    register = 0b01010101
    datagram = Datagram(register, 0)
    datagram.set_write()
    self.assertEqual(datagram.header, 0b11010101)
    self.assertEqual(datagram.register, register)
    datagram.set_read()
    self.assertEqual(datagram.header, register)
    self.assertEqual(datagram.register, register)

if __name__ == '__main__':
  unittest.main()
