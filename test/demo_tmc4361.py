import logging
import RPi.GPIO as GPIO
from RaspberryPiStepperDriver.motion.tmc4361 import driver as drv
from RaspberryPiStepperDriver.motion.tmc4361.spi import SPI

logging.basicConfig(level=logging.DEBUG)

spi = SPI(bus=0, device=1)

driver = drv.TMC4361(
  spi=spi,
  start_signal_pin=16,
  target_reached_interrupt_pin=0
)

driver.start()

GPIO.cleanup()
