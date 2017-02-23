import logging
import RPi.GPIO as GPIO

from steppyr.lib.bits import mask
from steppyr.lib.functions import constrain
from steppyr.lib.trinamic import MICROSTEP_RESOLUTION, parse_ini
from steppyr.drivers.stepdir import StepDirDriver
from steppyr.drivers.tmc26x.registers import *

"""
Ported to Python from https://github.com/trinamic/TMC26XStepper
"""

log = logging.getLogger(__name__)

def calc_current_scaling(resistor_value, current_ma, vsense_v):
  """
  resistor_value: resistor value in milli-ohms
  vsense_v:  vsense = 0.310V (VSENSE not set) or vsense = 0.165V (VSENSE set)
  """
  # This is derived from I=(cs+1)/32*(Vsense/Rsense)
  # leading to cs = CS = 32*R*I/V (with V = 0,31V oder 0,165V and I = 1000*current)
  # with Rsense = 0,15
  # for vsense = 0,310V (VSENSE not set) or vsense = 0,165V (VSENSE set)
  # Last value below is theoretically - 1.0 for better rounding it is 0.5
  return ( resistor_value * current_ma * 32.0 / ( vsense_v * 1000.0 * 1000.0 ) ) - 0.5

class TMC26XDriver(StepDirDriver):

  def __init__(self, spi, dir_pin, step_pin, pin_mode=GPIO.BCM, current=300, resistor=150):
    """
    Arguments
      dir_pin - the pin where the direction pin is connected
      step_pin - the pin where the step pin is connected
      current - chopper current in milliamps
      resistor - sense resistor value
    """
    super().__init__(dir_pin=dir_pin, step_pin=step_pin, pin_mode=pin_mode)
    self._microsteps = 1
    self._spi = spi
    # store the current and sense resistor value for later use
    self._resistor = resistor
    # we are not started yet
    self._started = False
    # by default cool step is not enabled
    self._cool_step_enabled = False
    # Holds the last result read from the spi bus
    self._driver_status_result = None
    # setting the default register values
    # self.driver_control_register_value = set_bit(REGISTERS['DRIVER_CONTROL_REGISTER'], INITIAL_MICROSTEPPING)
    # self._microsteps = (1 << INITIAL_MICROSTEPPING)
    # self.chopper_config_register = REGISTERS['CHOPPER_CONFIG_REGISTER']
    # self.cool_step_register_value = REGISTERS['COOL_STEP_REGISTER']
    # self.stall_guard2_current_register_value = REGISTERS['STALL_GUARD2_LOAD_MEASURE_REGISTER']
    # self.driver_configuration_register_value = set_bit(REGISTERS['DRIVER_CONFIG_REGISTER'], DRIVER_CONTROL_REGISTER['READ_STALL_GUARD_READING'])
    # self.driver_control_register = DriverControlRegister()
    # self.chopper_config_register = ChopperControllRegister()
    # self.cool_step_register = CoolStepControlRegister()
    # self.stall_guard2_register = StallGuard2ControlRegister()
    # self.driver_config_register = DriverConfigRegister()
    self._registers = {
      DriverControlRegister: DriverControlRegister(),
      ChopperControllRegister: ChopperControllRegister(),
      CoolStepControlRegister: CoolStepControlRegister(),
      StallGuard2ControlRegister: StallGuard2ControlRegister(),
      DriverConfigRegister: DriverConfigRegister()
    }

  def flush_registers(self):
    for register in self._registers.values():
      self._spi.write(register)

  def load_registers_from_ini(self, path):
    for register_code, register_value in parse_ini(path):
      fixed_register_code = register_code & 0b111
      # Look up register
      for register_class, register in self._registers.items():
        if register_class.REGISTER == fixed_register_code:
          # Chop header off of register value
          bitmask = mask(0, (register._datagram_len - register._header_len)-1)
          fixed_register_value = register_value & bitmask
          self._registers[register_class] = register_class(fixed_register_value)

  #
  # Driver API methods
  #

  def activate(self):
    self._started = True
    self.flush_registers()
    super().activate()

  def shutdown(self):
    super().shutdown()
    self.disable()
    self._started = False

  def enable(self):
    """ Enable hardware (possibly temporarily) """
    # self._registers[ChopperControllRegister].set(ChopperControllRegister.bits.TOFF, self.constant_off_time)
    # if not enabled we don't have to do anything since we already delete t_off from the register
    if self._started:
      # self._spi.write(self.chopper_config_register)
      # We are assuing that Chopper settings have already been set
      # self._spi.write(self._registers[ChopperControllRegister])
      self.flush_registers()

  def disable(self):
    """ Disable hardware (possibly temporarily) """
    self._registers[ChopperControllRegister].set(ChopperControllRegister.bits.TOFF, 0)
    # if not enabled we don't have to do anything since we already delete t_off from the register
    if self._started:
      # self._spi.write(self.chopper_config_register)
      # self._spi.write(self._registers[ChopperControllRegister])
      self.flush_registers()

  def set_microsteps(self, microsteps):
    # setting_pattern = 0
    # poor mans log
    if microsteps >= 256:
      # setting_pattern = 0
      self._microsteps = 256
    elif microsteps >= 128:
      # setting_pattern = 1
      self._microsteps = 128
    elif microsteps >= 64:
      # setting_pattern = 2
      self._microsteps = 64
    elif microsteps >= 32:
      # setting_pattern = 3
      self._microsteps = 32
    elif microsteps>=16:
      # setting_pattern = 4
      self._microsteps = 16
    elif microsteps >= 8:
      # setting_pattern = 5
      self._microsteps = 8
    elif microsteps >= 4:
      # setting_pattern = 6
      self._microsteps = 4
    elif microsteps >= 2:
      # setting_pattern = 7
      self._microsteps = 2
      # 1 and 0 lead to full step
    elif microsteps <= 1:
      # setting_pattern = 8
      self._microsteps = 1

    # delete the old value
    # self.driver_control_register_value &= 0xFFFF0
    # set the new value
    # self.driver_control_register_value = set_bit(self.driver_control_register_value, setting_pattern)
    # self.driver_control_register.set(DriverControlRegister.bits.MRES, MICROSTEP_RESOLUTION[self._microsteps])
    self._registers[DriverControlRegister].set(DriverControlRegister.bits.MRES, MICROSTEP_RESOLUTION[self._microsteps])

    # if started we directly send it to the motor
    if self._started:
      # self.send262(self.driver_control_register_value)
      # self._spi.write(self.driver_control_register)
      self._spi.write(self._registers[DriverControlRegister])

  @property
  def microsteps(self):
    return self._microsteps

  #
  # Non-API methods
  #

  def set_stepdir_off(self, value=1):
    # DRVCONF SDOFF 1
    self._registers[DriverConfigRegister].set(DriverConfigRegister.bits.SDOFF, value)
    if self._started:
      self._spi.write(self._registers[DriverConfigRegister])

  def send262(self, datagram):
    """
    send register settings to the stepper driver via SPI
    returns the current status
    """
    # Send datagram to driver
    msg = [ ((datagram >> 16) & 0xff), ((datagram >>  8) & 0xff), ((datagram) & 0xff) ]
    out = self._spi.transfer(msg)
    # Process and save the response
    response = out[0]
    response <<= 8
    response |= out[1]
    response <<= 8
    response |= out[2]
    response <<= 4
    self._driver_status_result = response
    return out

  def is_enabled(self):
    """ Returns true if hardware is enabled, False otherwise """
    toff = self._registers[ChopperControllRegister].get(ChopperControllRegister.bits.TOFF)
    if toff != 0:
      return True
    else:
      return False

  def set_current(self, current_ma):
    """
    current_ma: current in milliamps

    Datasheet ch. 8: The low sensitivity (high sense resistor voltage, VSENSE=0) brings best and most robust current
    regulation, while high sensitivity (low sense resistor voltage; VSENSE=1) reduces power dissipation in the sense resistor.
    """
    self._current_ma = current_ma
    resistor_value = self._resistor

    current_scaling = int(calc_current_scaling(resistor_value, current_ma, 0.31))

    # Set Vsense register
    # check if the current scaling is too low
    if current_scaling < 16:
      log.debug('current_scaling < 16. Enabling vsense and recaculating')
      # set the csense bit to get a use half the sense voltage (to support lower motor currents)
      self._registers[DriverConfigRegister].set(DriverConfigRegister.bits.VSENSE)
      # and recalculate the current setting
      # Last value below is theoretically - 1.0 for better rounding it is 0.5
      current_scaling = int( calc_current_scaling(resistor_value, current_ma, 0.165) )
    else:
      # remove vesense flag
      log.debug('current_scaling >= 16. Disabling vsense')
      self._registers[DriverConfigRegister].unset(DriverConfigRegister.bits.VSENSE)

    # do some sanity checks
    if current_scaling > 31:
      current_scaling = 31

    # set the new current scaling
    self._registers[StallGuard2ControlRegister].set(StallGuard2ControlRegister.bits.CS, current_scaling)

    if self._started:
      self._spi.write(self._registers[StallGuard2ControlRegister])

    log.debug('Final current scaling is: %s', current_scaling)

    # if started we directly send it to the motor
    if self._started:
      self._spi.write(self._registers[DriverConfigRegister])
      self._spi.write(self._registers[StallGuard2ControlRegister])

  def set_current_scaling(self, current_scaling):
    self._registers[StallGuard2ControlRegister].set(StallGuard2ControlRegister.bits.CS, current_scaling)
    if self._started:
      self._spi.write(self._registers[StallGuard2ControlRegister])

  def get_current(self):
    # we calculate the current according to the datasheet to be on the safe side
    # this is not the fastest but the most accurate and illustrative way
    result = self._registers[StallGuard2ControlRegister].get(StallGuard2ControlRegister.bits.CS)
    resistor_value = self._resistor
    vsense_on = self._registers[DriverConfigRegister].get(DriverConfigRegister.bits.VSENSE)
    voltage = 0.165 if vsense_on else 0.31
    result = ( result + 1.0 ) / 32.0 * voltage / resistor_value * 1000.0 * 1000.0
    return result

  def set_spreadcycle_chopper(self,
      off_time, blanking_time, hysteresis_start, hysteresis_end, hysteresis_decrement, chopper_mode=0):
    """
    Set chopper mode.
    :param chopper_mode: 0 spreadCycle mode, 1 Constant off time mode
    :param off_time: 0 = chopper off, 1 .. 15 = Off time setting
    :param blanking_time: 16, 24, 36, or 54 system clock cycles
    :return:
    """
    # Sanity check and map values
    off_time = constrain(off_time, 0, 15)
    blank_value = lookup_blanking_time_value(blanking_time)
    hysteresis_start = constrain(hysteresis_start, 0, 7)
    hysteresis_end = constrain(hysteresis_end, 0, 15)
    hysteresis_decrement = constrain(hysteresis_decrement, 0, 3)
    # Set registers
    # SpreadCycle and Constant Off choppers share registers, so always start with a clean register
    self._registers[ChopperControllRegister] = ChopperControllRegister()\
      .set(ChopperControllRegister.bits.CHM, chopper_mode)\
      .set(ChopperControllRegister.bits.TOFF, off_time)\
      .set(ChopperControllRegister.bits.TB, blank_value)\
      .set(ChopperControllRegister.bits.HSTRT, hysteresis_start)\
      .set(ChopperControllRegister.bits.HEND, hysteresis_end)\
      .set(ChopperControllRegister.bits.HDEC, hysteresis_decrement)\
    # Send message if we are active
    if self._started:
      # self._spi.write(self._registers[ChopperControllRegister])
      self.flush_registers()

  def set_constant_off_time_chopper(self, constant_off_time, blank_time,
                                    fast_decay_time_setting, sine_wave_offset, use_current_comparator, chopper_mode=1):
    """
    constant_off_time: The off time setting controls the minimum chopper frequency.
    For most applications an off time within  the range of 5μs to 20μs will fit.
       2...15: off time setting
    blank_time: Selects the comparator blank time. This time needs to safely cover the switching event and the
      duration of the ringing on the sense resistor. For
      0: min. setting 3: max. setting
    fast_decay_time_setting: Fast decay time setting. With CHM=1, these bits control the portion of fast decay for each chopper cycle.
       0: slow decay only
       1...15: duration of fast decay phase
    sine_wave_offset: Sine wave offset. With CHM=1, these bits control the sine wave offset.
    A positive offset corrects for zero crossing error.
      -3..-1: negative offset 0: no offset 1...12: positive offset
    use_current_comparator: Selects usage of the current comparator for termination of the fast decay cycle.
    If current comparator is enabled, it terminates the fast decay cycle in case the current
    reaches a higher negative value than the actual positive value.
      1: enable comparator termination of fast decay cycle
      0: end by time only
    """
    # SpreadCycle and Constant Off choppers share registers, so always start with a clean register
    self._registers[ChopperControllRegister] = ChopperControllRegister()

    # perform some sanity checks
    constant_off_time = constrain(constant_off_time, 2, 15)
    # calculate the value acc to the clock cycles
    blank_value = lookup_blanking_time_value(blank_time)
    fast_decay_time_setting = constrain(fast_decay_time_setting, 0, 15)
    sine_wave_offset = constrain(sine_wave_offset, -3, 12)
    # shift the sine_wave_offset
    sine_wave_offset += 3

    # set the constant off pattern
    # Constant tOFF with fast decay time
    self._registers[ChopperControllRegister].set(ChopperControllRegister.bits.CHM, chopper_mode)
    # set the blank timing value
    self._registers[ChopperControllRegister].set(ChopperControllRegister.bits.TB, blank_value)
    # setting the constant off time
    self._registers[ChopperControllRegister].set(ChopperControllRegister.bits.TOFF, constant_off_time)
    # set the fast decay time
    # set msb
    self._registers[ChopperControllRegister].set(ChopperControllRegister.bits.HDEC, fast_decay_time_setting & 0x8)
    # other bits
    self._registers[ChopperControllRegister].set(ChopperControllRegister.bits.HSTRT, fast_decay_time_setting & 0x7)
    # set the sine wave offset
    self._registers[ChopperControllRegister].set(ChopperControllRegister.bits.HEND, sine_wave_offset)
    # using the current comparator?
    if not use_current_comparator:
      # TODO not sure if this bitmask is right
      self._registers[ChopperControllRegister].set(1<<12)

    #if started we directly send it to the motor
    if self._started:
      self._spi.write(self._registers[ChopperControllRegister])
      # self.flush_registers()

  def set_stallguard(self, stallguard_threshold, stallguard_filter_enabled=0):
    stallguard_threshold = constrain(stallguard_threshold, -64, 63)
    self._registers[StallGuard2ControlRegister].set(StallGuard2ControlRegister.bits.SGT, stallguard_threshold)
    self._registers[StallGuard2ControlRegister].set(StallGuard2ControlRegister.bits.SFILT, stallguard_filter_enabled)
    if self._started:
      self._spi.write(self._registers[StallGuard2ControlRegister])

  # TODO Update this unmaintained code
  """
  def set_random_off_time(self, value):
    if value:
      self.chopper_config_register |= CHOPPER_CONFIG_REGISTER['RANDOM_TOFF_TIME']
    else:
      self.chopper_config_register &= ~ CHOPPER_CONFIG_REGISTER['RANDOM_TOFF_TIME']
    # if started we directly send it to the motor
    if self._started:
      self.send262(self.driver_control_register_value)
  """

  # TODO Update this unmaintained code
  """
  def set_step_interpolation(self, enable=True):
    ""
    Enable or disable STEP interpolation.

    False: Disable STEP pulse interpolation.
    True: Enable STEP pulse multiplication by 16.
    ""
    if enable:
      self.driver_configuration_register_value = set_bit(self.driver_configuration_register_value, DRIVER_CONTROL_REGISTER['STEP_INTERPOLATION'])
    else:
      self.driver_configuration_register_value = unset_bit(self.driver_configuration_register_value, DRIVER_CONTROL_REGISTER['STEP_INTERPOLATION'])
    if self._started:
      self.send262(self.driver_configuration_register_value)
  """

  # TODO Update this unmaintained code
  """
  def get_stall_guard_threshold(self):
    stall_guard_threshold = self.stall_guard2_current_register_value & STALL_GUARD_REGISTER['STALL_GUARD_VALUE_PATTERN']
    # shift it down to bit 0
    stall_guard_threshold >>= 8
    # convert the value to an int to correctly handle the negative numbers
    result = stall_guard_threshold
    # check if it is negative and fill it up with leading 1 for proper negative number representation
    if result & ( 1 << 6 ):
      result |= 0xC0
    return result
  """

  # TODO Update this unmaintained code
  """
  def get_stall_guard_filter(self):
    if self.stall_guard2_current_register_value & STALL_GUARD_REGISTER['STALL_GUARD_FILTER_ENABLED']:
      return True
    else:
      return False
  """

  # TMC26X Status methods

  # TODO Update this unmaintained code
  '''
  def read_status(self, read_value=None):
    """
    read_value: One of TMC26X_READOUT_STALLGUARD or TMC26X_READOUT_CURRENT.
                All other cases are ignored to prevent funny values.

    Returns self._driver_status_result for convenience.
    """
    old_driver_configuration_register_value = self.driver_configuration_register_value
    # reset the readout configuration
    self.driver_configuration_register_value &= ~ DRIVER_CONTROL_REGISTER['READ_SELECTION_PATTERN']
    # this now equals TMC26X_READOUT_POSITION - so we just have to check the other two options
    if read_value == TMC26X_READOUT_STALLGUARD:
      self.driver_configuration_register_value |= DRIVER_CONTROL_REGISTER['READ_STALL_GUARD_READING']
    elif read_value == TMC26X_READOUT_CURRENT:
      self.driver_configuration_register_value |= DRIVER_CONTROL_REGISTER['READ_STALL_GUARD_AND_COOL_STEP']
    # all other cases are ignored to prevent funny values
    # check if the readout is configured for the value we are interested in
    if self.driver_configuration_register_value != old_driver_configuration_register_value:
      # because then we need to write the value twice - one time for configuring, second time to get the value, see below
      self.send262(self.driver_configuration_register_value)
    # write the configuration to get the last status
    self.send262(self.driver_configuration_register_value)
    return self._driver_status_result
  '''

  # TODO Update this unmaintained code
  """
  def get_readout_value(self):
    ""
    reads the stall guard setting from last status
    returns -1 if stallguard inforamtion is not present
    ""
    return self.get_driver_status() >> 10
  """

  # TODO Update this unmaintained code
  """
  def get_driver_status(self):
    if self._driver_status_result == None:
      raise ValueError('driver_status_result is empty. Call read_status() first.')
    return self._driver_status_result
  """

  # TODO Update this unmaintained code
  """
  def get_motor_position(self):
    # we read it out even if we are not started yet - perhaps it is useful information for somebody
    self.read_status(TMC26X_READOUT_POSITION)
    return self.get_readout_value()
  """

  # TODO Update this unmaintained code
  '''
  def get_current_stall_guard_reading(self):
    """
    reads the stall guard setting from last status
    returns -1 if stallguard information is not present
    """
    # if we don't yet started there cannot be a stall guard value
    if not self._started:
      return -1
    # not time optimal, but solution optiomal:
    # first read out the stall guard value
    self.read_status(TMC26X_READOUT_STALLGUARD)
    return self.get_readout_value()
  '''

  # TODO Update this unmaintained code
  """
  def get_current_cs_reading(self):
    # if we don't yet started there cannot be a stall guard value
    if not started:
      return False
    # not time optimal, but solution optiomal:
    # first read out the stall guard value
    self.read_status(TMC26X_READOUT_CURRENT)
    return ( self.get_readout_value() & 0x1f )
  """

  # TODO Update this unmaintained code
  """
  def get_current_current(self):
    result = self.get_current_cs_reading()
    resistor_value = self._resistor
    voltage = 0.165 if (self.driver_configuration_register_value & DRIVER_CONTROL_REGISTER['VSENSE']) else 0.31
    result = ( result + 1.0 ) / 32.0 * voltage / resistor_value * 1000.0 * 1000.0
    return result
  """

  # TODO Update this unmaintained code
  """
  def is_stall_guard_over_threshold(self):
    ""
    return true if the stallguard threshold has been reached
    ""
    if self._started:
      return False
    return (self.get_driver_status() & STATUS_STALL_GUARD_STATUS)
  """

  # TODO Update this unmaintained code
  """
  def get_over_temperature(self):
    ""
    returns if there is any over temperature condition:
    OVER_TEMPERATURE_PREWARING if pre warning level has been reached
    OVER_TEMPERATURE_SHUTDOWN if the temperature is so hot that the driver is shut down
    Any of those levels are not too good.
    ""
    if self._started:
      return False
    if self.get_driver_status() & STATUS_OVER_TEMPERATURE_SHUTDOWN:
      return TMC26X_OVERTEMPERATURE_SHUTDOWN
    if self.get_driver_status() & STATUS_OVER_TEMPERATURE_WARNING:
      return TMC26X_OVERTEMPERATURE_PREWARING
    return False
  """

  # TODO Update this unmaintained code
  '''
  def is_short_to_ground_a(self):
    """
    is motor channel A shorted to ground
    """
    if not self._started:
      return False
    return ( self.get_driver_status() & STATUS_SHORT_TO_GROUND_A )
  '''

  # TODO Update this unmaintained code
  '''
  def is_short_to_ground_b(self):
    """
    is motor channel B shorted to ground
    """
    if not self._started:
      return False
    return ( self.get_driver_status() & STATUS_SHORT_TO_GROUND_B )
  '''

  # TODO Update this unmaintained code
  '''
  def is_open_load_a(self):
    """
    is motor channel A connected
    """
    if not self._started:
      return False
    return ( self.get_driver_status() & STATUS_OPEN_LOAD_A )
  '''

  # TODO Update this unmaintained code
  '''
  def is_open_load_b(self):
    """
    is motor channel B connected
    """
    if not self._started:
      return False
    return ( self.get_driver_status() & STATUS_OPEN_LOAD_B )
  '''

  # TODO Update this unmaintained code
  '''
  def is_stand_still(self):
    """
    is chopper inactive since 2^20 clock cycles - defaults to ~0,08s
    """
    if not self._started:
      return False
    return ( self.get_driver_status() & STATUS_STAND_STILL )
  '''

  # TODO Update this unmaintained code
  '''
  def is_stall_guard_reached(self):
    """
    is chopper inactive since 2^20 clock cycles - defaults to ~0,08s
    """
    if not self._started:
      return False
    return ( self.get_driver_status() & STATUS_STALL_GUARD_STATUS )
  '''

  # TODO Update this unmaintained code
  '''
  def is_current_scaling_halfed(self):
    if self.driver_configuration_register_value & DRIVER_CONTROL_REGISTER['VSENSE']:
      return True
    else:
      return False
  '''

def lookup_blanking_time_value(blank_time):
  """
  0 16 system clock cycles
  1 24 system clock cycles
  2 36 system clock cycles
  3 54 system clock cycles
  :param blank_time:
  :return:
  """
  if blank_time >= 54:
    blank_value = 3
  elif blank_time >= 36:
    blank_value = 2
  elif blank_time >= 24:
    blank_value = 1
  else:
    blank_value = 0
  return blank_value
