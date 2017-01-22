import logging
import spidev
from RaspberryPiStepperDriver import tobin, set_bit, unset_bit
from RaspberryPiStepperDriver.motion.tmc4361 import registers

log = logging.getLogger(__name__)

class SPI:
  """
  http://www.takaitra.com/posts/492

  core_clock_hz defaults to 250MHz on all Raspberry Pis
  clock_div must be a multiple of 2
  max_speed_hz = core_clock_hz / clock_div

  https://www.raspberrypi.org/documentation/hardware/raspberrypi/spi/README.md
  http://raspberrypi.stackexchange.com/a/700
  http://dangerousprototypes.com/docs/Logic_analyzer_mode
  """
  def __init__(self, bus=0, device=0):
    self._bus = bus
    self._device = device
    self._spi = spidev.SpiDev()
    self._spi.open(bus, device)
    self._spi.bits_per_word = 8
    self._spi.cshigh = False
    self._spi.lsbfirst = False
    self._spi.max_speed_hz = 1953000 # clock_div = 128
    self._spi.mode = 0b11

  def transfer(self, payload):
    out = self._spi.xfer2(payload)
    log.debug('spi(%s, %s) >>> %s %s', self._bus, self._device, tobin(payload), payload)
    log.debug('spi(%s, %s) <<< %s %s', self._bus, self._device, tobin(out), out)
    return out

  def writeRegister(self, the_register, datagram):
    """
    Arguments:
    unsigned const char cs_squirrel
    unsigned const char the_register
    unsigned const long datagram
    """
    self.sendRegister(the_register | registers.WRITE_MASK, datagram)

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
    (unsigned const char) motor_nr
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
