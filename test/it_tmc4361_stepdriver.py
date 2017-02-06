import asyncio, logging, time
import RPi.GPIO as GPIO
from RaspberryPiStepperDriver.drivers import StepperDriver
from RaspberryPiStepperDriver.activators.tmc4361.driver import TMC4361
from RaspberryPiStepperDriver.activators.tmc4361.spi import SPI
"""
Demonstrates integration between TCM4361 and StepperDriver.
"""

logging.basicConfig(level=logging.INFO)

loop = asyncio.get_event_loop()

spi = SPI(bus=0, device=1)
tmc4361 = TMC4361(
  spi=spi,
  reset_pin=26
)
driver = StepperDriver(
  activator=tmc4361,
  profile=tmc4361
)

driver.start()
driver.set_microsteps(1)
# driver.set_target_speed(100)
# driver.set_acceleration(10)
driver.profile.set_ramp_trapezoid(v_max=100, a_max=10, d_max=10)
#driver.profile.set_ramp_scurve(
#  v_max=10000, a_max=100*10, d_max=100*10,
#  bow1, bow2, bow3, bow4,
#  a_start=0, d_final=0, v_start=0, v_stop=0)
driver.profile.set_motor_steps_per_rev(200)
driver.profile.tmc26x.set_current(1000)

driver.move_to(1000)
driver.move_to(-1000)

async def report(driver):
  print('Report loop starting')
  while await driver.run():
    print('Report for iteration')
    print('    get_status_events', driver.profile.get_status_events())
    print('    get_status_flags', driver.profile.get_status_flags())
    print('    current_speed', driver.profile.current_speed)
    print('    target_steps', driver.profile.target_steps)
    print('    current_steps', driver.profile.current_steps)
    print('    distance_to_go', driver.profile.distance_to_go)
    print('    target_speed', driver.profile.target_speed)
    print('    acceleration', driver.profile.acceleration)
    print('    target_acceleration', driver.profile.target_acceleration)
    print('    target_deceleration', driver.profile.target_deceleration)
    print('    microsteps', driver.profile.microsteps)
    await asyncio.sleep(1)
  print('Reporting done')

try:
  f = asyncio.ensure_future(report(driver))
  loop.run_forever()
except Exception as e:
  print(e)
finally:
  driver.stop()
  GPIO.cleanup()
