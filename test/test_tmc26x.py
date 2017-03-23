import unittest
from steppyr import StepperController
from steppyr.drivers.spi import SPI
from steppyr.drivers import tmc26x
from steppyr.profiles.rectangle import RectangleProfile

def init():
  try:
    spi_dev = SPI(0, 0)
  except FileNotFoundError:
    # No SPI bus is available. Use a fake
    spi_dev = spi.SPI(None, None)
  stepper = StepperController(
    driver=tmc26x.TMC26XDriver(
      spi=spi_dev,
      step_pin=18,
      dir_pin=23,
      current=300,
      resistor=150
    ),
    profile=RectangleProfile()
  )
  return (spi_dev, stepper)

def debug_last_status(stepper, read_status_value=None):
  print('INFO: driver started', stepper.activator.started)
  stepper.activator.read_status(read_status_value)
  if stepper.activator.get_over_temperature() & tmc26x.TMC26X_OVERTEMPERATURE_PREWARING:
    print("WARNING: Overtemperature Prewarning!")
  elif stepper.activator.get_over_temperature() & tmc26x.TMC26X_OVERTEMPERATURE_SHUTDOWN:
    print("ERROR: Overtemperature Shutdown!")
  if stepper.activator.is_short_to_ground_a():
    print("ERROR: SHORT to ground on channel A!")
  if stepper.activator.is_short_to_ground_b():
    print("ERROR: SHORT to ground on channel A!")
  if stepper.activator.is_open_load_a():
    print("ERROR: Channel A seems to be unconnected!")
  if stepper.activator.is_open_load_b():
    print("ERROR: Channel B seems to be unconnected!")
  if stepper.activator.is_stall_guard_reached():
    print("INFO: Stall Guard level reached!")
  if stepper.activator.is_stand_still():
    print("INFO: Motor is standing still.")
  readout_config = stepper.activator.driver_configuration_register_value & tmc26x.DRIVER_CONTROL_REGISTER['READ_SELECTION_PATTERN']
  value = stepper.activator.get_readout_value()
  if readout_config == tmc26x.DRIVER_CONTROL_REGISTER['READ_MICROSTEP_POSTION']:
    print("INFO: Microstep postion phase A: ", value)
  elif readout_config == tmc26x.DRIVER_CONTROL_REGISTER['READ_STALL_GUARD_READING']:
    print("INFO: Stall Guard value:", value)
  elif readout_config == tmc26x.DRIVER_CONTROL_REGISTER['READ_STALL_GUARD_AND_COOL_STEP']:
    stallGuard = value & 0xF
    current = value & 0x1F0
    print("INFO: Approx Stall Guard:", stallGuard)
    print("INFO: Current level", current)

class TestSuite(unittest.TestCase):

  def test_dump(self):

    print('REGISTERS')
    for key, value in tmc26x.REGISTERS.items():
      print('  ', key, value, bin(value)[2:])

    print('DRIVER_CONTROL_REGISTER')
    for key, value in tmc26x.DRIVER_CONTROL_REGISTER.items():
      print('  ', key, value, bin(value)[2:])

    spi_dev, stepper = init()
    stepper.activate()

    # Enable or disable chopper current flow
    stepper.activator.disable()
    stepper.activator.enable()

    print('INITIAL_MICROSTEPPING')
    print('  ', bin(tmc26x.INITIAL_MICROSTEPPING))

    print('_stepper_motor.driver_control_register_value')
    print('  ', tmc26x.tobin(stepper.activator.driver_control_register_value, 20))

    print('MICROSTEP_RESOLUTION')
    print('  ', tmc26x.MICROSTEP_RESOLUTION)

    print('microsteps')
    print('  ', bin(stepper.activator.microsteps))

    print('Register Values')
    print('  driver_control_register_value       ', tmc26x.tobin(stepper.activator.driver_control_register_value, 20))
    print('  chopper_config_register             ', tmc26x.tobin(stepper.activator.chopper_config_register, 20))
    print('  cool_step_register_value            ', tmc26x.tobin(stepper.activator.cool_step_register_value, 20))
    print('  stall_guard2_current_register_value ', tmc26x.tobin(stepper.activator.stall_guard2_current_register_value, 20))
    print('  driver_configuration_register_value ', tmc26x.tobin(stepper.activator.driver_configuration_register_value, 20))

    print('steps left', stepper.steps_to_go)
    stepper.step(100)
    print('steps left', stepper.steps_to_go)

  def test_get_current(self):
    spi_dev, stepper = init()
    current = stepper.activator.get_current()
    print('current', current)

  def test_set_stall_guard_threshold(self):
    spi_dev, stepper = init()
    stepper.activator.set_stall_guard_threshold(20, True)
    stepper.activator.set_stall_guard_threshold(999, False)

  def test_get_stall_guard_threshold(self):
    spi_dev, stepper = init()
    # In range
    t_in = 20
    stepper.activator.set_stall_guard_threshold(t_in, True)
    t_out = stepper.activator.get_stall_guard_threshold()
    assert(t_in == t_out)
    # FIXME doesnt work correctly
    # out of range
    # t_in = -100
    # stepper.set_stall_guard_threshold(t_in, True)
    # t_out = stepper.get_stall_guard_threshold()
    # print('stall_guard_threshold', t_in, t_out)
    # assert(t_in == 63)

  def dump(self):
    spi_dev, stepper = init()
    debug_last_status(stepper)
    debug_last_status(stepper, tmc26x.TMC26X_READOUT_STALLGUARD)
    debug_last_status(stepper, tmc26x.TMC26X_READOUT_CURRENT)

if __name__ == '__main__':
  unittest.main()
