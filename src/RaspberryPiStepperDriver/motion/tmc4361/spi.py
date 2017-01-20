import logging
import spidev

log = logging.getLogger(__name__)

class SPI:
  """
  http://www.takaitra.com/posts/492
  """
  def __init__(self, bus=0, device=0):
    self._spi = spidev.SpiDev()
    self._spi.open(bus, device)
    self._spi.bits_per_word = 8
    self._spi.cshigh = False
    self._spi.lsbfirst = False
    self._spi.max_speed_hz = 1953000
    self._spi.mode = 0b11

  def transfer(self, payload):
    return self._spi.xfer2(payload)

  def writeRegister(self, the_register, datagram):
    """
    Arguments:
    unsigned const char cs_squirrel
    unsigned const char the_register
    unsigned const long datagram
    """
    self.sendRegister(the_register | 0x80, datagram)

  def readRegister(self, the_register):
    """
    Arguments:
    unsigned const char cs_squirrel
    unsigned const char the_register

    Returns: (unsigned long)
    """
    self.sendRegister(the_register, 0)
    result = self.sendRegister(the_register & 0x7F, 0)

    return result

  def sendRegister(self, the_register, datagram):
    """
    Arguments:
    (unsigned const char) motor_nr
    (unsigned const char) the_register
    (unsigned const long) datagram

    Returns: (unsigned long)
    """
    # the first value is ignored
    # unsigned char spi_status
    spi_status = self.transfer(the_register)

    message = [
      (datagram >> 24) & 0xff,
      (datagram >> 16) & 0xff,
      (datagram >>  8) & 0xff,
      (datagram) & 0xff
    ]

    i_datagram = self.transfer(message)

    # Serial.print(F("Status :"));
    # Serial.println(spi_status,BIN);
    # Serial.print(F("Received "));
    # Serial.println(i_datagram,HEX);
    # Serial.println();

    return i_datagram
