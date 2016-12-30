import asyncio, logging
from RaspberryPiStepperDriver import spi, tmc26x
from RaspberryPiStepperDriver.profiles import rectangle as rectangle_profile

logging.basicConfig(level=logging.DEBUG)
log = logging.getLogger(__name__)

spi_dev = spi.SPI(0, 0)
profile = rectangle_profile.RectangleProfile()
driver = tmc26x.TMC26XStepper(spi_dev, profile,
  dir_pin=23, step_pin=18, current=300, resistor=150)

driver.start()

loop = asyncio.get_event_loop()
