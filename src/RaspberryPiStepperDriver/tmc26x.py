import time

"""
Ported to python from https://github.com/trinamic/TMC26XStepper
"""

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

# definitions for the driver control register
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

def micros():
  """
  Mymics the Arduino micros() function.
  """
  return int(time.time() * 1000000)

tobin = lambda x, n: format(x, 'b').ljust(n, '0')

class TMC26XStepper:
  """
  int number_of_steps, int cs_pin, int dir_pin, int step_pin, unsigned int current, unsigned int resistor
  """
  def __init__(self, spi, number_of_steps, dir_pin, step_pin, current, resistor):
    self.spi = spi

    # save the number of steps
    self.number_of_steps = number_of_steps
    # we are not started yet
    self.started = False
    # by default cool step is not enabled
    self.cool_step_enabled = False

    # save the pins for later use
    self.dir_pin = dir_pin
    self.step_pin = step_pin
    # store the current sense resistor value for later use
    self.resistor = resistor

    # Both of these are needed to avoid div by 0 error in set_speed
    # Set a default speed in rpm
    self.speed = 60
    self.last_step_time = micros()
    self.driver_status_result = None

    # initizalize our status values
    self.steps_left = 0
    self.direction = 0
    self.spi_steps = 0

    # initialize register values
    self.driver_control_register_value = REGISTERS['DRIVER_CONTROL_REGISTER'] | INITIAL_MICROSTEPPING
    self.chopper_config_register = REGISTERS['CHOPPER_CONFIG_REGISTER']

    # setting the default register values
    self.driver_control_register_value = REGISTERS['DRIVER_CONTROL_REGISTER'] | INITIAL_MICROSTEPPING
    self.microsteps = (1 << INITIAL_MICROSTEPPING)
    self.chopper_config_register = REGISTERS['CHOPPER_CONFIG_REGISTER']
    self.cool_step_register_value = REGISTERS['COOL_STEP_REGISTER']
    self.stall_guard2_current_register_value = REGISTERS['STALL_GUARD2_LOAD_MEASURE_REGISTER']
    self.driver_configuration_register_value = REGISTERS['DRIVER_CONFIG_REGISTER'] | DRIVER_CONTROL_REGISTER['READ_STALL_GUARD_READING']

    # set the current
    self.set_current(current)
    # set to a conservative start value
    self.set_constant_off_time_chopper(7, 54, 13, 12, 1)
    # set a nice microstepping value
    self.set_microsteps(DEFAULT_MICROSTEPPING_VALUE)

  def send262(self, datagram):
    """
    send register settings to the stepper driver via SPI
    returns the current status
    """
    # Send datagram to driver
    msg = [ ((datagram >> 16) & 0xff), ((datagram >>  8) & 0xff), ((datagram) & 0xff) ]
    print('send262  >> ', tobin(datagram, 20))
    out = self.spi.transfer(msg)
    print('send262 <<  ', tobin(out[0], 20))

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
  # Initialization methods
  #

  def start(self):
    # Send configuration to the driver chip
    self.send262(self.driver_control_register_value)
    self.send262(self.chopper_config_register)
    self.send262(self.cool_step_register_value)
    self.send262(self.stall_guard2_current_register_value)
    self.send262(self.driver_configuration_register_value)

    self.started = True

  def unstart(self):
    self.started = False

  def set_enabled(self, enabled):
    # delete the t_off in the chopper config to get sure
    self.chopper_config_register &= ~ CHOPPER_CONFIG_REGISTER['T_OFF_PATTERN']
    if enabled:
      # and set the t_off time
      self.chopper_config_register |= self.constant_off_time
    # if not enabled we don't have to do anything since we already delete t_off from the register
    if self.started:
      self.send262(self.chopper_config_register)

  def is_enabled(self):
    if self.chopper_config_register & CHOPPER_CONFIG_REGISTER['T_OFF_PATTERN']:
      return True
    else:
      return False

  #
  # Configuration / setter methods
  #

  def set_current(self, current_ma):
    """
    current_ma: current in milliamps
    """
    current_scaling = 0

    resistor_value = self.resistor
    # remove vesense flag
    print('driver_configuration_register_value before ', bin(self.driver_configuration_register_value))
    self.driver_configuration_register_value &= ~ DRIVER_CONTROL_REGISTER['VSENSE']
    print('driver_configuration_register_value after ', bin(self.driver_configuration_register_value))

    # This is derrived from I=(cs+1)/32*(Vsense/Rsense)
    # leading to cs = CS = 32*R*I/V (with V = 0,31V oder 0,165V  and I = 1000*current)
    # with Rsense=0,15
    # for vsense = 0,310V (VSENSE not set)
    # or vsense = 0,165V (VSENSE set)
    current_scaling = ( ( resistor_value * current_ma * 32.0 / ( 0.31 * 1000.0 * 1000.0 ) ) - 0.5) # theoretically - 1.0 for better rounding it is 0.5
    # check if the current scalingis too low
    if current_scaling < 16:
      # set the csense bit to get a use half the sense voltage (to support lower motor currents)
      self.driver_configuration_register_value |= DRIVER_CONTROL_REGISTER['VSENSE']
      # and recalculate the current setting
      current_scaling = int(( ( resistor_value * current_ma * 32.0 / ( 0.165 * 1000.0 * 1000.0 ) ) - 0.5) )# theoretically - 1.0 for better rounding it is 0.5

    # do some sanity checks
    if current_scaling > 31:
      current_scaling = 31

    # delete the old value
    self.stall_guard2_current_register_value &= ~STALL_GUARD_REGISTER['CURRENT_SCALING_PATTERN']
    # set the new current scaling
    self.stall_guard2_current_register_value |= current_scaling
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
    result = ( result + 1.0 ) / 32.0 * voltage / resistor_value * 1000.0 *1000.0
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
    sine_wave_offset +=3

    # calculate the register setting
    # first of all delete all the values for this
    self.chopper_config_register &= ~ ( (1 << 12)
      | CHOPPER_CONFIG_REGISTER['BLANK_TIMING_PATTERN']
      | CHOPPER_CONFIG_REGISTER['HYSTERESIS_DECREMENT_PATTERN']
      | CHOPPER_CONFIG_REGISTER['HYSTERESIS_LOW_VALUE_PATTERN']
      | CHOPPER_CONFIG_REGISTER['HYSTERESIS_START_VALUE_PATTERN']
      | CHOPPER_CONFIG_REGISTER['T_OFF_TIMING_PATERN'] )

    # set the constant off pattern
    self.chopper_config_register |= CHOPPER_CONFIG_REGISTER['CHOPPER_MODE_T_OFF_FAST_DECAY']
    # set the blank timing value
    self.chopper_config_register |= blank_value << CHOPPER_CONFIG_REGISTER['BLANK_TIMING_SHIFT']
    #setting the constant off time
    self.chopper_config_register |= constant_off_time
    #set the fast decay time
    #set msb
    self.chopper_config_register |= (fast_decay_time_setting & 0x8) << CHOPPER_CONFIG_REGISTER['HYSTERESIS_DECREMENT_SHIFT']
    #other bits
    self.chopper_config_register |= (fast_decay_time_setting & 0x7) << CHOPPER_CONFIG_REGISTER['HYSTERESIS_START_VALUE_SHIFT']
    #set the sine wave offset
    self.chopper_config_register |= sine_wave_offset << CHOPPER_CONFIG_REGISTER['HYSTERESIS_LOW_SHIFT']
    #using the current comparator?
    if not use_current_comparator:
      self.chopper_config_register |= (1<<12);
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
      return -1
    else:
      return 0

  def set_microsteps(self, number_of_steps):
    setting_pattern = 0
    # poor mans log
    if number_of_steps >= 256:
      setting_pattern = 0
      self.microsteps = 256
    elif number_of_steps >= 128:
      setting_pattern = 1
      self.microsteps = 128
    elif number_of_steps >= 64:
      setting_pattern = 2
      self.microsteps = 64
    elif number_of_steps >= 32:
      setting_pattern = 3
      self.microsteps = 32
    elif number_of_steps>=16:
      setting_pattern = 4
      self.microsteps = 16
    elif number_of_steps >= 8:
      setting_pattern = 5
      self.microsteps = 8
    elif number_of_steps >= 4:
      setting_pattern = 6
      self.microsteps = 4
    elif number_of_steps >= 2:
      setting_pattern = 7
      self.microsteps = 2
      # 1 and 0 lead to full step
    elif number_of_steps <= 1:
      setting_pattern = 8
      self.microsteps = 1

    # delete the old value
    self.driver_control_register_value &= 0xFFFF0
    # set the new value
    self.driver_control_register_value |= setting_pattern

    # if started we directly send it to the motor
    if self.started:
      self.send262(driver_control_register_value)

    # recalculate the stepping delay by simply setting the speed again
    self.set_speed(self.speed)

  def set_speed(self, what_speed):
    self.speed = what_speed;
    self.step_delay = (60 * 1000 * 1000) / ( self.number_of_steps * what_speed * self.microsteps)
    # update the next step time
    self.next_step_time = self.last_step_time + self.step_delay

  #
  # Movement methods
  #

  def step(self, steps_to_move):
    """
    Moves the motor steps_to_move steps.  If the number is negative,
    the motor moves in the reverse direction.
    """
    if self.steps_left == 0:
      self.steps_left = abs(steps_to_move)  # how many steps to take
      # determine direction based on whether steps_to_mode is + or -:
      if steps_to_move > 0:
        self.direction = 1
      elif steps_to_move < 0:
        self.direction = 0
      return 0
    else:
      return -1

  def move(self):
    # decrement the number of steps, moving one step each time:
    if self.steps_left > 0:
      current_time = micros();
      # move only if the appropriate delay has passed:
      if current_time >= self.next_step_time:

        # step forward or back, depending on direction:
        if self.direction == 1:
          digitalWrite(self.step_pin, HIGH)
        else:
          digitalWrite(self.dir_pin, HIGH)
          digitalWrite(self.step_pin, HIGH)

        # get the timeStamp of when you stepped:
        self.last_step_time = current_time
        self.next_step_time = current_time + self.step_delay

        # decrement the steps left:
        self.steps_left = self.steps_left - 1

        # disable the step & dir pins
        digitalWrite(self.step_pin, LOW);
        digitalWrite(self.dir_pin, LOW);
      return -1
    return 0

  def is_moving(self):
    return self.steps_left > 0

  def stop(self):
    # note to self if the motor is currently moving
    state = self.is_moving()
    # stop the motor
    self.steps_left = 0
    self.direction = 0
    # return if it was moving
    return state

  #
  # Status methods
  #

  def read_status(self, read_value=None):
    """
    read_value: One of TMC26X_READOUT_STALLGUARD or TMC26X_READOUT_CURRENT.
                All other cases are ignored to prevent funny values.
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

  def get_readout_value(self):
    """
    reads the stall guard setting from last status
    returns -1 if stallguard inforamtion is not present
    """
    return self.get_driver_status() >> 10

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
    if not started:
      return -1
    # not time optimal, but solution optiomal:
    # first read out the stall guard value
    self.read_status(TMC26X_READOUT_STALLGUARD)
    return self.get_readout_value()

  def get_current_cs_reading(self):
    # if we don't yet started there cannot be a stall guard value
    if not started:
      return 0
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

  def get_driver_status(self):
    if self.driver_status_result == None:
      raise ValueError('driver_status_result is empty. Call read_status() first.')
    return self.driver_status_result

  def get_over_temperature(self):
    """
    returns if there is any over temperature condition:
    OVER_TEMPERATURE_PREWARING if pre warning level has been reached
    OVER_TEMPERATURE_SHUTDOWN if the temperature is so hot that the driver is shut down
    Any of those levels are not too good.
    """
    if self.started:
      return 0
    if self.get_driver_status() & STATUS_OVER_TEMPERATURE_SHUTDOWN:
      return TMC26X_OVERTEMPERATURE_SHUTDOWN
    if self.get_driver_status() & STATUS_OVER_TEMPERATURE_WARNING:
      return TMC26X_OVERTEMPERATURE_PREWARING
    return 0

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
