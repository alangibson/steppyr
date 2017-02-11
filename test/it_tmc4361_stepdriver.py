import asyncio, logging, time
import RPi.GPIO as GPIO
from steppyr import StepperController
from steppyr.drivers.tmc4361 import TMC4361Driver
from steppyr.drivers.tmc4361.spi import SPI
"""
Demonstrates integration between TCM4361 and StepperController.
"""

logging.basicConfig(level=logging.INFO)

loop = asyncio.get_event_loop()

spi = SPI(bus=0, device=1)
tmc4361 = TMC4361Driver(
  spi=spi,
  reset_pin=26
)
stepper = StepperController(
  driver=tmc4361,
  profile=tmc4361
)
stepper.set_microsteps(1)
# driver.set_target_speed(100)
# driver.set_target_acceleration(10)
stepper.profile.set_ramp_trapezoid(target_speed=200, target_acceleration=400, target_deceleration=400)
#driver.profile.set_ramp_scurve(
#  v_max=10000, a_max=100*10, d_max=100*10,
#  bow1, bow2, bow3, bow4,
#  a_start=0, d_final=0, v_start=0, v_stop=0)
stepper.profile.set_full_steps_per_rev(200)
stepper.profile.tmc26x.set_current(1000)
stepper.profile.tmc26x.set_constant_off_time_chopper(
  constant_off_time=7,
  blank_time=54,
  fast_decay_time_setting=13,
  sine_wave_offset=12,
  use_current_comparator=1)

stepper.activate()

stepper.move_to(1000)
stepper.move_to(-1000)

async def report(stepper):
  print('Report loop starting')
  while await stepper.run():
    print('Report for iteration')
    print('    get_status_events', stepper.profile.get_status_events())
    print('    get_status_flags', stepper.profile.get_status_flags())
    print('    current_speed', stepper.profile.current_speed)
    print('    target_steps', stepper.profile.target_steps)
    print('    current_steps', stepper.profile.current_steps)
    print('    distance_to_go', stepper.profile.steps_to_go)
    print('    target_speed', stepper.profile.target_speed)
    print('    acceleration', stepper.profile.current_acceleration)
    print('    target_acceleration', stepper.profile.target_acceleration)
    print('    target_deceleration', stepper.profile.target_deceleration)
    print('    microsteps', stepper.profile.microsteps)
    await asyncio.sleep(1)
  print('Reporting done')

try:
  f = asyncio.ensure_future(report(stepper))
  loop.run_forever()
except Exception as e:
  print(e)
finally:
  stepper.shutdown()
  GPIO.cleanup()
