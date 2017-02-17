from steppyr.drivers.tmc4361 import *
from steppyr.drivers.tmc4361.spi import SPI as TMC4361SPI

# logging.config.fileConfig('logging.ini')
logging.basicConfig(level=logging.INFO)
log = logging.getLogger(__name__)

spi1 = TMC4361SPI(bus=0, device=1)
tmc4361 = TMC4361Driver(
  spi=spi1,
  reset_pin=26
)
# Configure tmc4361
tmc4361.set_microsteps(1)
tmc4361.set_ramp_trapezoid(target_speed=100, target_acceleration=10, target_deceleration=10)
# Configure tmc26x
tmc4361.tmc26x.set_current(1000)
tmc4361.tmc26x.set_constant_off_time_chopper(
  constant_off_time=7,
  blank_time=54,
  fast_decay_time_setting=13,
  sine_wave_offset=12,
  use_current_comparator=1)
tmc4361.tmc26x.set_microsteps(1)
# Or from config files
# tmc4361.load_registers_from_ini('test/tmc4361/data/20170210_02.55.36_TMC4361_Settings.ini')
# tmc4361.tmc26x.load_registers_from_ini('test/tmc4361/data/20170210_02.56.32_TMC26x_Settings.ini')

tmc4361.activate()

# set_target_steps() aka move_to()
tmc4361.set_target_steps(10000)

try:
  for i in range(0,10):
    print('Report for iteration', i)
    print('    get_status_events', tmc4361.get_status_events())
    print('    get_status_flags', tmc4361.get_status_flags())
    print('    get_current_speed', tmc4361.current_speed)
    print('    get_target_position', tmc4361.target_steps)
    print('    get_current_position', tmc4361.current_steps)
    print('    get_target_speed', tmc4361.target_speed)
    sleep_microseconds(500000)
except Exception as e:
  print(e)
finally:
  # driver.stop()
  tmc4361.shutdown()
  GPIO.cleanup()
