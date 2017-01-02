import asyncio, logging, time
from . import DIRECTION_CW, DIRECTION_CCW, micros
from .activators import stepdir as stepdir_act
"""
Ported to python from https://github.com/trinamic/TMC26XStepper
"""

log = logging.getLogger(__name__)

# some default values used in initialization
DEFAULT_MICROSTEPPING_VALUE = 32

REGISTER_BIT_PATTERN = 0xFFFFF

# return value for TMC26XStepper.getOverTemperature() if there is a overtemperature situation in the TMC chip
# This warning indicates that the TCM chip is too warm.
# It is still working but some parameters may be inferior.
# You should do something against it.
TMC26X_OVERTEMPERATURE_PREWARING = 1
# return value for TMC26XStepper.getOverTemperature() if there is a overtemperature shutdown in the TMC chip
# This warning indicates that the TCM chip is too warm to operate and has shut down to prevent damage.
# It will stop working until it cools down again.
# If you encouter this situation you must do something against it. Like reducing the current or improving the PCB layout
# and/or heat management.
TMC26X_OVERTEMPERATURE_SHUTDOWN = 2

# Which values can be read out
# Selects to readout the microstep position from the motor.
TMC26X_READOUT_POSITION = 0
# Selects to read out the StallGuard value of the motor.
TMC26X_READOUT_STALLGUARD = 1
# Selects to read out the current current setting (acc. to CoolStep) and the
# upper bits of the StallGuard value from the motor.
TMC26X_READOUT_CURRENT = 3

# definitions for the input from the TCM260
STATUS_STALL_GUARD_STATUS = 0x1
STATUS_OVER_TEMPERATURE_SHUTDOWN = 0x2
STATUS_OVER_TEMPERATURE_WARNING = 0x4
STATUS_SHORT_TO_GROUND_A = 0x8
STATUS_SHORT_TO_GROUND_B = 0x10
STATUS_OPEN_LOAD_A = 0x20
STATUS_OPEN_LOAD_B = 0x40
STATUS_STAND_STILL = 0x80
READOUT_VALUE_PATTERN = 0xFFC00

# TMC26X register definitions
REGISTERS = {
  'DRIVER_CONTROL_REGISTER': 0x00000,
  'CHOPPER_CONFIG_REGISTER': 0x80000,
  'COOL_STEP_REGISTER': 0xA0000,
  'STALL_GUARD2_LOAD_MEASURE_REGISTER': 0xC0000,
  'DRIVER_CONFIG_REGISTER': 0xE0000
}

# definitions for the driver control (DRVCTRL) register
DRIVER_CONTROL_REGISTER = {
  'MICROSTEPPING_PATTERN': 0xF,
  'STEP_INTERPOLATION': 0x200,
  'DOUBLE_EDGE_STEP': 0x100,
  'VSENSE': 0x40,
  'READ_MICROSTEP_POSTION': 0x0,
  'READ_STALL_GUARD_READING': 0x10,
  'READ_STALL_GUARD_AND_COOL_STEP': 0x20,
  'READ_SELECTION_PATTERN': 0x30
}

# definitions for stall guard2 current register
STALL_GUARD_REGISTER = {
  'STALL_GUARD_FILTER_ENABLED': 0x10000,
  'STALL_GUARD_TRESHHOLD_VALUE_PATTERN': 0x17F00,
  'CURRENT_SCALING_PATTERN': 0x1F,
  'STALL_GUARD_CONFIG_PATTERN': 0x17F00,
  'STALL_GUARD_VALUE_PATTERN': 0x7F00,
}

# definitions for the chopper config register
CHOPPER_CONFIG_REGISTER = {
  'CHOPPER_MODE_STANDARD': 0X0,
  'CHOPPER_MODE_T_OFF_FAST_DECAY': 0X4000,
  'T_OFF_PATTERN': 0Xf,
  'RANDOM_TOFF_TIME': 0X2000,
  'BLANK_TIMING_PATTERN': 0X18000,
  'BLANK_TIMING_SHIFT': 15,
  'HYSTERESIS_DECREMENT_PATTERN': 0X1800,
  'HYSTERESIS_DECREMENT_SHIFT': 11,
  'HYSTERESIS_LOW_VALUE_PATTERN': 0X780,
  'HYSTERESIS_LOW_SHIFT': 7,
  'HYSTERESIS_START_VALUE_PATTERN': 0X78,
  'HYSTERESIS_START_VALUE_SHIFT': 4,
  'T_OFF_TIMING_PATERN': 0XF
  }

# Microstep resolution for STEP/DIR mode.
# Microsteps per 90°
MICROSTEP_RESOLUTION = {
  256:  0b0000,
  128:  0b0001,
  64:   0b0010,
  32:   0b0011,
  16:   0b0100,
  4:    0b0110,
  8:    0b0101,
  2:    0b0111,  # halfstep
  1:    0b1000   # fullstep
}

# default values
INITIAL_MICROSTEPPING = MICROSTEP_RESOLUTION[32]

tobin = lambda x, n: format(x, 'b').ljust(n, '0')

class TMC26XStepper:
  """
  Arguments
    dir_pin - the pin where the direction pin is connected
    step_pin - the pin where the step pin is connected
    current - chopper current in milliamps
    resistor - sense resistor value
  """
  def __init__(self, spi, profile, dir_pin, step_pin, current, resistor=150):
    self._profile = profile
    self._activator = stepdir_act.StepDirActivator(dir_pin, step_pin)
    self.spi = spi
    # store the current and sense resistor value for later use
    self.resistor = resistor
    self.current = current
    # Both of these are needed to avoid div by 0 error in set_speed
    # Set a default speed in rpm
    self._last_step_time_us = micros()
    # we are not started yet
    self.started = False
    # by default cool step is not enabled
    self.cool_step_enabled = False
    # Holds the last result read from the spi bus
    self.driver_status_result = None
    # setting the default register values
    self.driver_control_register_value = set_bit(REGISTERS['DRIVER_CONTROL_REGISTER'], INITIAL_MICROSTEPPING)
    self.microsteps = (1 << INITIAL_MICROSTEPPING)
    self.chopper_config_register = REGISTERS['CHOPPER_CONFIG_REGISTER']
    self.cool_step_register_value = REGISTERS['COOL_STEP_REGISTER']
    self.stall_guard2_current_register_value = REGISTERS['STALL_GUARD2_LOAD_MEASURE_REGISTER']
    self.driver_configuration_register_value = set_bit(REGISTERS['DRIVER_CONFIG_REGISTER'], DRIVER_CONTROL_REGISTER['READ_STALL_GUARD_READING'])

  def send262(self, datagram):
    """
    send register settings to the stepper driver via SPI
    returns the current status
    """
    # Send datagram to driver
    msg = [ ((datagram >> 16) & 0xff), ((datagram >>  8) & 0xff), ((datagram) & 0xff) ]
    log.debug('spi  >> %s', tobin(datagram, 20))
    out = self.spi.transfer(msg)
    log.debug('spi <<  %s', tobin(out[0], 20))

    # Process and save the response
    response = out[0]
    response <<= 8
    response |= out[1]
    response <<= 8
    response |= out[2]
    response <<= 4
    self.driver_status_result = response

    return out

  #
  # Driver API
  #

  def start(self):
    # set the current
    self.set_current(self.current)
    # set to a conservative start value
    self.set_constant_off_time_chopper(7, 54, 13, 12, 1)
    # set a nice microstepping value
    self.set_microsteps(self.microsteps)

    # Send configuration to the driver chip
    self.send262(self.driver_control_register_value)
    self.send262(self.chopper_config_register)
    self.send262(self.cool_step_register_value)
    self.send262(self.stall_guard2_current_register_value)
    self.send262(self.driver_configuration_register_value)
    self.started = True
    # TODO Below here copied from AccelStepper
    self._activator.start()
    self._run_forever_future = asyncio.ensure_future(self.run_forever())

  def shutdown(self):
    self.disable()

  def enable(self):
    self.set_enabled(True)

  def disable(self):
    self.set_enabled(False)

  def is_enabled(self):
    if self.chopper_config_register & CHOPPER_CONFIG_REGISTER['T_OFF_PATTERN']:
      return True
    else:
      return False

  def set_target_speed(self, speed):
    """
    Set our requested ultimate cruising speed.

    Arguments:
      speed (float): Steps per second
    """
    self._profile.set_target_speed(speed)

  def set_pulse_width(self, pulse_width_us):
    """
    Set the step pulse width in microseconds.
    """
    # self._pulse_width_us = pulse_width_us
    self._activator.set_pulse_width(pulse_width_us)

  def step(self, direction):
    """
    TODO copied from AccelStepper
    """
    self._activator.step(self._profile._direction)

  async def run_forever(self):
    """
    Continuously call run() as fast as possible.

    TODO Copied from AccelStepper
    """
    while True:
      await self.run()
      # Without this await, we never yield back to the event loop
      await asyncio.sleep(0)

  async def run_until_done(self):
    """
    Blockingly calls run() until is_move == False
    """
    while self.is_moving:
      await self.run()
      await asyncio.sleep(0)

  async def run(self):
    """
    Run the motor to implement speed and acceleration in order to proceed to the target position
    You must call this at least once per step, preferably in your main loop
    If the motor is in the desired position, the cost is very small
    returns true if the motor is still running to the target position.

    TODO Copied from AccelStepper
    """
    if await self.run_at_speed():
      self._profile.compute_new_speed()
    return self.is_moving

  async def run_at_speed(self):
    """
    Move 1 step if it is time for another step. Otherwise, do nothing.
    Pulse duty cycle is apparently undefined. Pulse ends as quickly as possible.
    Returns -1 if a step occured, 0 otherwise.

    TODO functionally the same as AccelStepper
    """
    # Dont do anything unless we actually have a step interval
    # Dont do anything unless we have somewhere to go
    if not self._profile._step_interval_us or not self._profile.distance_to_go:
      return False

    current_time_us = micros()

    next_step_time_us = self._last_step_time_us + self._profile._step_interval_us
    # move only if the appropriate delay has passed:
    if current_time_us >= next_step_time_us:
      # Its time to take a step
      log.debug('Should step current_time_us=%s next_step_time_us=%s step_interval_us=%s direction=%s',
        current_time_us, next_step_time_us, self._profile._step_interval_us, self._profile._direction)

      # decrement the steps left:
      if self._profile._direction == DIRECTION_CW:
        # Clockwise
        self._profile._current_steps += 1
      else:
        # Anticlockwise
        self._profile._current_steps -= 1

      # step forward or back, depending on direction:
      self.step(self._profile._direction)

      # get the timeStamp of when you stepped:
      self._last_step_time_us = current_time_us
      self._next_step_time_us = current_time_us + self._profile._step_interval_us
      return True
    else:
      # No step necessary at this time
      return False

  async def wait_on_move(self):
    """
    'blocks' until is_moving == False.
    Only use this method if you know there are no other calls to move() happening,
    or this method may never return. For example: during calibration at startup.

    TODO copied from AccelStepper
    """
    log.debug('wait_on_move is_moving=%s', self.is_moving)
    while self.is_moving:
      await asyncio.sleep(0)

  def move(self, relative_steps):
    """
    Schedules move to a number of steps relative to the current step count.
    TODO copied from AccelStepper
    """
    move_to_steps = self._profile._current_steps + relative_steps
    log.debug('Moving %s relative steps from %s to %s',
      relative_steps, self._profile._current_steps, move_to_steps)
    self.move_to(move_to_steps)

  def move_to(self, absolute_steps):
    """
    Schedules move to an absolute number of steps.

    TODO copied from AccelStepper
    """
    if self._profile._target_steps != absolute_steps:
      self._profile._previous_target_steps = self._profile._target_steps
      self._profile._target_steps = absolute_steps
      self._profile.compute_new_speed()

  @property
  def is_moving(self):
    return self._profile.distance_to_go != 0

  @property
  def distance_to_go(self):
    return self._profile.distance_to_go

  def stop(self):
    self.abort()
    self.disable()

  def abort(self):
    """
    TODO copied from AccelStepper
    """
    self.set_current_position(self._profile._current_steps)

  def reset_step_counter(self):
    """
    TODO copied from AccelStepper
    """
    self.set_current_position(0)

  def set_current_position(self, position):
    """
    Useful during initialisations or after initial positioning
    Sets speed to 0

    TODO copied from AccelStepper
    """
    self._profile.set_current_position(position)

  def set_microsteps(self, microsteps):
    setting_pattern = 0
    # poor mans log
    if microsteps >= 256:
      setting_pattern = 0
      self.microsteps = 256
    elif microsteps >= 128:
      setting_pattern = 1
      self.microsteps = 128
    elif microsteps >= 64:
      setting_pattern = 2
      self.microsteps = 64
    elif microsteps >= 32:
      setting_pattern = 3
      self.microsteps = 32
    elif microsteps>=16:
      setting_pattern = 4
      self.microsteps = 16
    elif microsteps >= 8:
      setting_pattern = 5
      self.microsteps = 8
    elif microsteps >= 4:
      setting_pattern = 6
      self.microsteps = 4
    elif microsteps >= 2:
      setting_pattern = 7
      self.microsteps = 2
      # 1 and 0 lead to full step
    elif microsteps <= 1:
      setting_pattern = 8
      self.microsteps = 1

    # delete the old value
    self.driver_control_register_value &= 0xFFFF0
    # set the new value
    self.driver_control_register_value = set_bit(self.driver_control_register_value, setting_pattern)

    # if started we directly send it to the motor
    if self.started:
      self.send262(self.driver_control_register_value)

    # recalculate the stepping delay by simply setting the speed again
    self._profile.compute_new_speed()

  @property
  def position(self):
    """
    TODO copied from AccelStepper
    """
    return self._profile._current_steps

  @property
  def direction(self):
    """
    TODO copied from AccelStepper
    """
    return self._profile._direction

  def predict_direction(self, target_steps):
    """
    TODO copied from AccelStepper
    """
    return DIRECTION_CW if self.predict_distance_to_go(target_steps) > 0 else DIRECTION_CCW

  def predict_distance_to_go(self, target_steps):
    """
    TODO copied from AccelStepper
    """
    return target_steps - self._profile._current_steps

  #
  # Configuration / setter methods
  #

  def set_enabled(self, enabled):
    # delete the t_off in the chopper config to get sure
    self.chopper_config_register = unset_bit(self.chopper_config_register, CHOPPER_CONFIG_REGISTER['T_OFF_PATTERN'])
    if enabled:
      # and set the t_off time
      self.chopper_config_register = set_bit(self.chopper_config_register, self.constant_off_time)
    # if not enabled we don't have to do anything since we already delete t_off from the register
    if self.started:
      self.send262(self.chopper_config_register)

  def set_current(self, current_ma):
    """
    current_ma: current in milliamps
    """
    self.current = current_ma
    current_scaling = 0

    resistor_value = self.resistor
    # remove vesense flag
    self.driver_configuration_register_value = unset_bit(self.driver_configuration_register_value, DRIVER_CONTROL_REGISTER['VSENSE'])

    # This is derrived from I=(cs+1)/32*(Vsense/Rsense)
    # leading to cs = CS = 32*R*I/V (with V = 0,31V oder 0,165V  and I = 1000*current)
    # with Rsense=0,15
    # for vsense = 0,310V (VSENSE not set)
    # or vsense = 0,165V (VSENSE set)
    current_scaling = ( ( resistor_value * current_ma * 32.0 / ( 0.31 * 1000.0 * 1000.0 ) ) - 0.5) # theoretically - 1.0 for better rounding it is 0.5
    # check if the current scalingis too low
    if current_scaling < 16:
      # set the csense bit to get a use half the sense voltage (to support lower motor currents)
      self.driver_configuration_register_value = set_bit(self.driver_configuration_register_value, DRIVER_CONTROL_REGISTER['VSENSE'])
      # and recalculate the current setting
      current_scaling = int(( ( resistor_value * current_ma * 32.0 / ( 0.165 * 1000.0 * 1000.0 ) ) - 0.5) )# theoretically - 1.0 for better rounding it is 0.5

    # do some sanity checks
    if current_scaling > 31:
      current_scaling = 31

    # delete the old value
    self.stall_guard2_current_register_value = unset_bit(self.stall_guard2_current_register_value, STALL_GUARD_REGISTER['CURRENT_SCALING_PATTERN'])
    # set the new current scaling
    self.stall_guard2_current_register_value = set_bit(self.stall_guard2_current_register_value, current_scaling)

    # if started we directly send it to the motor
    if self.started:
      self.send262(self.driver_configuration_register_value)
      self.send262(self.stall_guard2_current_register_value)

  def get_current(self):
    # we calculate the current according to the datasheet to be on the safe side
    # this is not the fastest but the most accurate and illustrative way
    result = self.stall_guard2_current_register_value & STALL_GUARD_REGISTER['CURRENT_SCALING_PATTERN']
    resistor_value = self.resistor
    voltage = 0.165 if (self.driver_configuration_register_value & DRIVER_CONTROL_REGISTER['VSENSE']) else 0.31
    result = ( result + 1.0 ) / 32.0 * voltage / resistor_value * 1000.0 * 1000.0
    return result

  def set_constant_off_time_chopper(self, constant_off_time, blank_time, fast_decay_time_setting, sine_wave_offset, use_current_comparator):
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
    # perform some sanity checks
    if constant_off_time < 2:
      constant_off_time = 2
    elif constant_off_time > 15:
      constant_off_time = 15

    # save the constant off time
    self.constant_off_time = constant_off_time

    # calculate the value acc to the clock cycles
    if blank_time >= 54:
      blank_value = 3
    elif blank_time >= 36:
      blank_value = 2
    elif blank_time >= 24:
      blank_value = 1
    else:
      blank_value = 0

    if fast_decay_time_setting < 0:
      fast_decay_time_setting = 0
    elif fast_decay_time_setting > 15:
      fast_decay_time_setting = 15

    if sine_wave_offset < -3:
      sine_wave_offset = -3
    elif sine_wave_offset > 12:
      sine_wave_offset = 12
    # shift the sine_wave_offset
    sine_wave_offset += 3

    # calculate the register setting
    # first of all delete all the values for this
    self.chopper_config_register = unset_bit(
      self.chopper_config_register,
      ( (1 << 12) | CHOPPER_CONFIG_REGISTER['BLANK_TIMING_PATTERN']
                  | CHOPPER_CONFIG_REGISTER['HYSTERESIS_DECREMENT_PATTERN']
                  | CHOPPER_CONFIG_REGISTER['HYSTERESIS_LOW_VALUE_PATTERN']
                  | CHOPPER_CONFIG_REGISTER['HYSTERESIS_START_VALUE_PATTERN']
                  | CHOPPER_CONFIG_REGISTER['T_OFF_TIMING_PATERN'] ) )

    # set the constant off pattern
    self.chopper_config_register = set_bit(self.chopper_config_register, CHOPPER_CONFIG_REGISTER['CHOPPER_MODE_T_OFF_FAST_DECAY'])
    # set the blank timing value
    self.chopper_config_register = set_bit(self.chopper_config_register, blank_value << CHOPPER_CONFIG_REGISTER['BLANK_TIMING_SHIFT'])
    #setting the constant off time
    self.chopper_config_register = set_bit(self.chopper_config_register, constant_off_time)
    #set the fast decay time
    #set msb
    self.chopper_config_register = set_bit(self.chopper_config_register,
      (fast_decay_time_setting & 0x8) << CHOPPER_CONFIG_REGISTER['HYSTERESIS_DECREMENT_SHIFT'])
    #other bits
    self.chopper_config_register = set_bit(self.chopper_config_register,
      (fast_decay_time_setting & 0x7) << CHOPPER_CONFIG_REGISTER['HYSTERESIS_START_VALUE_SHIFT'])
    #set the sine wave offset
    self.chopper_config_register = set_bit(self.chopper_config_register,
      sine_wave_offset << CHOPPER_CONFIG_REGISTER['HYSTERESIS_LOW_SHIFT'])
    #using the current comparator?
    if not use_current_comparator:
      self.chopper_config_register = set_bit(self.chopper_config_register, (1<<12))
    #if started we directly send it to the motor
    # TODO is this right? Shouldnt we send self.chopper_config_register?
    if self.started:
      self.send262(self.driver_control_register_value)

  def set_random_off_time(self, value):
    if value:
      self.chopper_config_register |= CHOPPER_CONFIG_REGISTER['RANDOM_TOFF_TIME']
    else:
      self.chopper_config_register &= ~ CHOPPER_CONFIG_REGISTER['RANDOM_TOFF_TIME']
    # if started we directly send it to the motor
    if self.started:
      self.send262(self.driver_control_register_value)

  def set_stall_guard_threshold(self, stall_guard_threshold, stall_guard_filter_enabled):
    if stall_guard_threshold < -64:
      stall_guard_threshold = -64
    # We just have 5 bits
    elif stall_guard_threshold > 63:
      stall_guard_threshold = 63

    # add trim down to 7 bits
    stall_guard_threshold &= 0x7f
    # delete old stall guard settings
    self.stall_guard2_current_register_value &= ~ STALL_GUARD_REGISTER['STALL_GUARD_CONFIG_PATTERN']
    if stall_guard_filter_enabled:
      self.stall_guard2_current_register_value |= STALL_GUARD_REGISTER['STALL_GUARD_FILTER_ENABLED']

    # Set the new stall guard threshold
    self.stall_guard2_current_register_value |= (( stall_guard_threshold << 8 ) & STALL_GUARD_REGISTER['STALL_GUARD_CONFIG_PATTERN'] )
    # if started we directly send it to the motor
    if self.started:
      self.send262(self.stall_guard2_current_register_value)

  def set_step_interpolation(self, enable=True):
    """
    Enable or disable STEP interpolation.

    False: Disable STEP pulse interpolation.
    True: Enable STEP pulse multiplication by 16.
    """
    if enable:
      self.driver_configuration_register_value = set_bit(self.driver_configuration_register_value, DRIVER_CONTROL_REGISTER['STEP_INTERPOLATION'])
    else:
      self.driver_configuration_register_value = unset_bit(self.driver_configuration_register_value, DRIVER_CONTROL_REGISTER['STEP_INTERPOLATION'])
    if self.started:
      self.send262(self.driver_configuration_register_value)

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

  def get_stall_guard_filter(self):
    if self.stall_guard2_current_register_value & STALL_GUARD_REGISTER['STALL_GUARD_FILTER_ENABLED']:
      return True
    else:
      return False

  #
  # Status methods
  #

  def read_status(self, read_value=None):
    """
    read_value: One of TMC26X_READOUT_STALLGUARD or TMC26X_READOUT_CURRENT.
                All other cases are ignored to prevent funny values.

    Returns self.driver_status_result for convenience.
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
    return self.driver_status_result

  def get_readout_value(self):
    """
    reads the stall guard setting from last status
    returns -1 if stallguard inforamtion is not present
    """
    return self.get_driver_status() >> 10

  def get_driver_status(self):
    if self.driver_status_result == None:
      raise ValueError('driver_status_result is empty. Call read_status() first.')
    return self.driver_status_result

  def get_motor_position(self):
    # we read it out even if we are not started yet - perhaps it is useful information for somebody
    self.read_status(TMC26X_READOUT_POSITION)
    return self.get_readout_value()

  def get_current_stall_guard_reading(self):
    """
    reads the stall guard setting from last status
    returns -1 if stallguard information is not present
    """
    # if we don't yet started there cannot be a stall guard value
    if not self.started:
      return -1
    # not time optimal, but solution optiomal:
    # first read out the stall guard value
    self.read_status(TMC26X_READOUT_STALLGUARD)
    return self.get_readout_value()

  def get_current_cs_reading(self):
    # if we don't yet started there cannot be a stall guard value
    if not started:
      return False
    # not time optimal, but solution optiomal:
    # first read out the stall guard value
    self.read_status(TMC26X_READOUT_CURRENT)
    return ( self.get_readout_value() & 0x1f )

  def get_current_current(self):
    result = self.get_current_cs_reading()
    resistor_value = self.resistor
    voltage = 0.165 if (self.driver_configuration_register_value & DRIVER_CONTROL_REGISTER['VSENSE']) else 0.31
    result = ( result + 1.0 ) / 32.0 * voltage / resistor_value * 1000.0 * 1000.0
    return result

  def is_stall_guard_over_threshold(self):
    """
    return true if the stallguard threshold has been reached
    """
    if self.started:
      return False
    return (self.get_driver_status() & STATUS_STALL_GUARD_STATUS)

  def get_over_temperature(self):
    """
    returns if there is any over temperature condition:
    OVER_TEMPERATURE_PREWARING if pre warning level has been reached
    OVER_TEMPERATURE_SHUTDOWN if the temperature is so hot that the driver is shut down
    Any of those levels are not too good.
    """
    if self.started:
      return False
    if self.get_driver_status() & STATUS_OVER_TEMPERATURE_SHUTDOWN:
      return TMC26X_OVERTEMPERATURE_SHUTDOWN
    if self.get_driver_status() & STATUS_OVER_TEMPERATURE_WARNING:
      return TMC26X_OVERTEMPERATURE_PREWARING
    return False

  def is_short_to_ground_a(self):
    """
    is motor channel A shorted to ground
    """
    if not self.started:
      return False
    return ( self.get_driver_status() & STATUS_SHORT_TO_GROUND_A )

  def is_short_to_ground_b(self):
    """
    is motor channel B shorted to ground
    """
    if not self.started:
      return False
    return ( self.get_driver_status() & STATUS_SHORT_TO_GROUND_B )

  def is_open_load_a(self):
    """
    is motor channel A connected
    """
    if not self.started:
      return False
    return ( self.get_driver_status() & STATUS_OPEN_LOAD_A )

  def is_open_load_b(self):
    """
    is motor channel B connected
    """
    if not self.started:
      return False
    return ( self.get_driver_status() & STATUS_OPEN_LOAD_B )

  def is_stand_still(self):
    """
    is chopper inactive since 2^20 clock cycles - defaults to ~0,08s
    """
    if not self.started:
      return False
    return ( self.get_driver_status() & STATUS_STAND_STILL )

  def is_stall_guard_reached(self):
    """
    is chopper inactive since 2^20 clock cycles - defaults to ~0,08s
    """
    if not self.started:
      return False
    return ( self.get_driver_status() & STATUS_STALL_GUARD_STATUS )

  def is_current_scaling_halfed(self):
    if self.driver_configuration_register_value & DRIVER_CONTROL_REGISTER['VSENSE']:
      return True
    else:
      return False

def unset_bit(value, mask):
  new_value = value & ~ mask
  log.debug('unsetting bit(s) %s %s -> %s', bin(value), bin(mask), bin(new_value))
  return new_value

def set_bit(value, mask):
  new_value = value | mask
  log.debug('setting bit(s) %s %s -> %s', bin(value), bin(mask), bin(new_value))
  return new_value
