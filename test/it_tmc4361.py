import logging
import RPi.GPIO as GPIO
from steppyr import sleep_microseconds
from steppyr.motion.tmc4361 import driver as drv
from steppyr.motion.tmc4361.spi import SPI

logging.basicConfig(level=logging.DEBUG)

spi = SPI(bus=0, device=1)

driver = drv.TMC4361(
  spi=spi,
  start_signal_pin=16,
  target_reached_interrupt_pin=0
)

driver.start()

driver.move_to(-2000)
for i in range(1, 5):
  print('get_current_speed', driver.get_current_speed())
  print('get_target_position', driver.get_target_position())
  print('get_current_position', driver.get_current_position())
  sleep_microseconds(100)

GPIO.cleanup()
