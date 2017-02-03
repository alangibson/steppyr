import logging, logging.config
import RPi.GPIO as GPIO
from RaspberryPiStepperDriver.activators import spi, tmc26x
from RaspberryPiStepperDriver import drivers, sleep_microseconds
from RaspberryPiStepperDriver.profiles.rectangle import RectangleProfile
from RaspberryPiStepperDriver.activators.tmc4361.driver import *
from RaspberryPiStepperDriver.activators.tmc4361.spi import SPI as TMC4361SPI

# logging.config.fileConfig('logging.ini')
logging.basicConfig(level=logging.DEBUG)
log = logging.getLogger(__name__)

spi1 = TMC4361SPI(bus=0, device=1)
tmc4361 = TMC4361(
  spi=spi1,
  start_signal_pin=0,
  target_reached_interrupt_pin=0
)
tmc4361.start()
tmc4361.set_microsteps(1)

tmc4361.enable_tmc26x()
tmc4361._tmc26x.set_current(1000)

tmc4361.move_to(10000, v_max=5000)

try:
  for i in range(0,10):
    print('Report for iteration', i)
    print('    get_status_events', tmc4361.get_status_events())
    print('    get_status_flags', tmc4361.get_status_flags())
    print('    get_current_speed', tmc4361.get_current_speed())
    print('    get_target_position', tmc4361.get_target_position())
    print('    get_current_position', tmc4361.get_current_position())
    print('    get_target_speed', tmc4361.get_target_speed())
    print('    get_tmc2660_response', tmc4361.get_tmc2660_response().data)
    # print('    transfer_to_tmc2660', tmc4361.transfer_to_tmc2660(0b00000000000000000000).data)
    sleep_microseconds(500000)
except Exception as e:
  print(e)
finally:
  # driver.stop()
  tmc4361.stop()
  GPIO.cleanup()
