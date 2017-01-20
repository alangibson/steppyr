from RaspberryPiStepperDriver.motion.tmc4361.driver import TCM4361
from RaspberryPiStepperDriver.motion.tmc4361.spi import SPI

spi = SPI(bus=0, device=1)

driver = TMC4361(
  spi=spi,
  start_signal_pin=0,
  target_reached_interrupt_pin=0
)

driver.prepareTMC4361()
