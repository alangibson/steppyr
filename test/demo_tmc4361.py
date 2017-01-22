import logging
from RaspberryPiStepperDriver.motion.tmc4361 import driver as drv
from RaspberryPiStepperDriver.motion.tmc4361.spi import SPI

logging.basicConfig(level=logging.DEBUG)

spi = SPI(bus=0, device=1)

driver = drv.TMC4361(
  spi=spi,
  start_signal_pin=0,
  target_reached_interrupt_pin=0
)

driver.prepareTMC4361()

driver.initialze()
