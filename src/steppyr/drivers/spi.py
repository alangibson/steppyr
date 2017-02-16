import logging
import spidev
from steppyr.lib.bits import unset_bit, set_bit, tobin

log = logging.getLogger(__name__)

class SPI:
  """
  SPI interface for the Raspberry Pi.

  core_clock_hz defaults to 250MHz on all Raspberry Pis
  clock_div must be a multiple of 2
  max_speed_hz = core_clock_hz / clock_div

  Warning: max_speed_hz > 244KHz are horribly unstable (at least on Raspberry Pi 2)

  Controlling an SPI device with the Raspberry Pi
    http://www.takaitra.com/posts/492
  SPI - Raspberry Pi Documentation
    https://www.raspberrypi.org/documentation/hardware/raspberrypi/spi/README.md
  What SPI frequencies does Raspberry Pi support?
    http://raspberrypi.stackexchange.com/a/700
  SPI driver latency and a possible solution
    https://www.raspberrypi.org/forums/viewtopic.php?f=44&t=19489
  Logic analyzer mode - Dangerous Prototypes
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
    self._spi.max_speed_hz = 244000 # clock div = 1024
    self._spi.mode = 0b11

  def transfer(self, payload):
    out = self._spi.xfer2(payload)
    log.debug('spi(%s, %s)  >> %s %s', self._bus, self._device, tobin(payload), payload)
    log.debug('spi(%s, %s) <<  %s %s', self._bus, self._device, tobin(out), out)
    return out
