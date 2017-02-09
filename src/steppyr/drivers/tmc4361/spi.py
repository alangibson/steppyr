import logging
import spidev
from steppyr.lib.bits import unset_bit, set_bit, tobin
from steppyr.drivers.tmc4361 import registers
from steppyr.drivers import spi as activator_spi
from steppyr.drivers.tmc4361.registers import CoverLowRegister

log = logging.getLogger(__name__)

class SPI(activator_spi.SPI):

  def writeRegister(self, the_register, datagram):
    """
    Arguments:
    unsigned const char cs_squirrel
    unsigned const char the_register
    unsigned const long datagram
    """
    return self.sendRegister(the_register | registers.WRITE_MASK, datagram)

  def readRegister(self, the_register):
    """
    Arguments:
    unsigned const char cs_squirrel
    unsigned const char the_register

    Returns: (unsigned long)
    """
    self.sendRegister(the_register, 0)
    result = self.sendRegister(the_register & registers.READ_MASK, 0)
    return result

  def sendRegister(self, the_register, datagram):
    """
    Arguments:
    (unsigned const char) the_register
    (unsigned const long) datagram

    Returns: (unsigned long)
    """
    # "Whenever data is read from or written to the TMC4361A, the first eight
    # bits that are delivered back contain the SPI status SPI_STATUS that consists of
    # eight user-selected event bits. The selection of these bits are explained in
    # chapter 5.2"
    # the first value is ignored
    # unsigned char spi_status
    # spi_status = self.transfer([the_register])
    # print('spi_status:', spi_status)
    message = [
      the_register,
      (datagram >> 24) & 0xff,
      (datagram >> 16) & 0xff,
      (datagram >>  8) & 0xff,
      (datagram) & 0xff
    ]

    i_datagram = self.transfer(message)

    return i_datagram

  def write(self, datagram):
    return datagram.as_response(
      self.transfer(datagram.set_write().to_list()) )

  def read(self, datagram):
    datagram.set_read()
    # Per docs, we always send payload data == 0
    self.transfer([datagram.header, 0, 0, 0, 0])
    return datagram.as_response(
      self.transfer([datagram.header, 0, 0, 0, 0]) )

class TMC26xCoverSPI(activator_spi.SPI):
  """
  An adapter that allows the TMC26x activator to talk to the TMC26x chip
  transparently by using the TMC4361 cover register.
  """
  def __init__(self, spi):
    self._spi = spi

  def write(self, tmc26x_datagram):
    datagram = CoverLowRegister(tmc26x_datagram.datagram)
    self._spi.write(datagram)
