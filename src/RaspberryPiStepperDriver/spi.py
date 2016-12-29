import logging
import spidev

log = logging.getLogger(__name__)

class SPI:
  """
  http://www.takaitra.com/posts/492
  """
  def __init__(self, bus=0, device=0):
    # Convenience behavior for debugging
    if bus == None or device == None:
      self._spi = None
      return

    self._spi = spidev.SpiDev()
    self._spi.open(bus, device)
    self._spi.bits_per_word = 8
    self._spi.cshigh = False
    self._spi.lsbfirst = False
    self._spi.max_speed_hz = 1953000
    self._spi.mode = 0b11

  def transfer(self, payload):
    if self._spi:
      return self._spi.xfer2(payload)
    else:
      # Convenience behavior for debugging
      log.debug(payload)
      return [0 for x in payload]
