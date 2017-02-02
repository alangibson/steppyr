import logging, logging.config
import RPi.GPIO as GPIO
from RaspberryPiStepperDriver.activators import spi, tmc26x
from RaspberryPiStepperDriver import drivers, sleep_microseconds
from RaspberryPiStepperDriver.profiles.rectangle import RectangleProfile
from RaspberryPiStepperDriver.motion.tmc4361.driver import *
from RaspberryPiStepperDriver.motion.tmc4361.spi import SPI as TMC4361SPI

logging.config.fileConfig('logging.ini')
log = logging.getLogger(__name__)

spi1 = TMC4361SPI(bus=0, device=1)

"""
spi0 = spi.SPI(bus=0, device=0)
driver = drivers.StepperDriver(
  activator=tmc26x.TMC26XActivator(
    spi=spi0,
    dir_pin=23,
    step_pin=18,
    current=300
  ),
  profile=RectangleProfile()
)
driver.set_target_speed(1000)
driver.set_microsteps(1)
driver.activator.set_current(1000)
driver.set_acceleration(40000) # steps per second per second
driver.set_pulse_width(2)
driver.start()
"""

tmc4361 = TMC4361(
  spi=spi1,
  start_signal_pin=0,
  target_reached_interrupt_pin=0
)

tmc4361.start()

tmc4361.enable_tmc26x()

tmc4361.move_to(10000)

# print(tmc4361.transfer_to_tmc2660(0x901B4))
# print(tmc4361.transfer_to_tmc2660(0x94557))
# print(tmc4361.transfer_to_tmc2660(0xD001F))
# print(tmc4361.transfer_to_tmc2660(0xE0010))
# print(tmc4361.transfer_to_tmc2660(0x00000))

try:
  for i in range(0,100):
    print('Report for iteration', i)
    print('    get_status_events', tmc4361.get_status_events())
    print('    get_status_flags', tmc4361.get_status_flags())
    print('    get_current_speed', tmc4361.get_current_speed())
    print('    get_target_position', tmc4361.get_target_position())
    print('    get_current_position', tmc4361.get_current_position())
    print('    get_target_speed', tmc4361.get_target_speed())
    print('    STATUS_EVENT_COVER_DONE', STATUS_EVENT_COVER_DONE in tmc4361.get_status_events())
    print('    ping', tmc4361.transfer_to_tmc2660(0b11100000000000000000))
    sleep_microseconds(1000)
except Exception as e:
  print(e)
finally:
  # driver.stop()
  tmc4361.stop()
  GPIO.cleanup()
