import RPi.GPIO as GPIO
from RaspberryPiStepperDriver import sleep_microseconds, tobin, set_bit, unset_bit, datagram_to_int
from . import registers, _BV

# TODO different from standard values
DIRECTION_CW = 1
DIRECTION_CCW = -1

CLOCK_FREQUENCY = 16000000

# Defines ported to Python functions
# simple FP math see https://ucexperiment.wordpress.com/2012/10/28/fixed-point-math-on-the-arduino-platform/
FIXED_23_8_MAKE = lambda a: a * (1 << 8)
FIXED_22_2_MAKE = lambda a: a * (1 << 2)

RAMP_MODE_POSITIONING_MODE = 0b100
RAMP_MODE_VELOCITY_MODE = 0b000
RAMP_MODE_NO_RAMP = 0b000
RAMP_MODE_TRAPEZOIDAL_RAMP = 0b001
RAMP_MODE_S_SHAPED_RAMP = 0b010

# (const unsigned long) default_4361_start_config
# _BV(0) x_target requires start
# _BV(4) use shaddow motion profiles
#  _BV(5) external start is an start
default_4361_start_config = 0 | _BV(0) | _BV(4) | _BV(5)

class TMC4361:

  def __init__(self, spi, start_signal_pin, target_reached_interrupt_pin, motor_steps_per_rev=200):
    self._spi = spi
    self._start_signal_pin = start_signal_pin
    self._target_reached_interrupt_pin = target_reached_interrupt_pin
    self._steps_per_revolution = motor_steps_per_rev
    # TODO make configurable
    GPIO.setmode(GPIO.BCM)

  def start(self):
    """
    "Per default, the voltage level transition from high to low triggers a start
    signal (START is an input), or START output indicates an active START event
    by switching from high to low level." Datasheet 9.1.3
    """
    GPIO.setup(self._start_signal_pin, GPIO.OUT)
    GPIO.output(self._start_signal_pin, GPIO.LOW)
    # will be released after setup is complete
    GPIO.setup(self._target_reached_interrupt_pin, GPIO.OUT)
    GPIO.output(self._target_reached_interrupt_pin, GPIO.LOW)

    # _BV(5) external start is an start (to ensure start is an input)
    # "External start signal is enabled as timer trigger. START pin is assigned as input."
    self._spi.writeRegister(registers.TMC4361_START_CONFIG_REGISTER, 0 | _BV(5))

    GPIO.output(self._start_signal_pin, GPIO.HIGH)
    # GPIO.setup(self._start_signal_pin, GPIO.IN)

    # preconfigure the TMC4361

    # see datasheet 19.13-19.14
    self._spi.writeRegister(registers.TMC4361_SPI_STATUS_SELECTION_REGISTER, 0 | _BV(0) | _BV(1))

    # we don't use direct values
    self._spi.writeRegister(registers.TMC4361_GENERAL_CONFIG_REGISTER, 0 | _BV(5))
    # Set positioning and ramp shape
    self._spi.writeRegister(registers.TMC4361_RAMP_MODE_REGISTER, RAMP_MODE_POSITIONING_MODE | RAMP_MODE_S_SHAPED_RAMP)
    # trapezoidal positioning
    self._spi.writeRegister(registers.TMC4361_SH_RAMP_MODE_REGISTER, _BV(2) | 1)
    # self._spi.writeRegister(registers.TMC4361_RAMP_MODE_REGISTER,_BV(2) | 2); //we want to go to positions in nice S-Ramps)
    # self._spi.writeRegister(registers.TMC4361_SH_RAMP_MODE_REGISTER,_BV(2) | 2); //we want to go to positions in nice S-Ramps)
    self._spi.writeRegister(registers.TMC4361_CLK_FREQ_REGISTER, CLOCK_FREQUENCY)
    # NEEDED so THAT THE SQUIRREL CAN RECOMPUTE EVERYTHING!
    self._spi.writeRegister(registers.TMC4361_START_DELAY_REGISTER, 512)
    self.set_motor_steps_per_rev(self._steps_per_revolution)
    self._spi.writeRegister(registers.TMC4361_START_CONFIG_REGISTER, default_4361_start_config);
    # filter start (unsigned long filter)
    # 0x20000 = 2 << 16, 0x400000 = 4 << 20
    filter = 0x20000 | 0x400000
    # filter ref
    filter |= (2<<8) | 0x4000
    self._spi.writeRegister(registers.TMC4361_INPUT_FILTER_REGISTER, filter)

  def set_motor_steps_per_rev(self, steps_per_rev):
    self._motor_steps_per_rev = steps_per_rev
    # Configure the motor type
    # FIXME dont asume 256 microsteps
    # unsigned long motorconfig
    motorconfig = 0x00 # we want 256 microsteps
    motorconfig |= steps_per_rev << 4
    self._spi.writeRegister(registers.TMC4361_STEP_CONF_REGISTER, motorconfig)

  # TODO const char homeMotorTMC4361

  def move_to(self, position):
    """
    Arguments:
    long position
    """
    # TODO if we are already at the target, do nothing

    # FIXME We want an immediate start, so temporarily modify the start register.
    # oldStartRegister = self._spi.readRegister(registers.TMC4361_START_CONFIG_REGISTER)
    # self._spi.writeRegister(registers.TMC4361_START_CONFIG_REGISTER, _BV(5)) # keep START as an input

    # We write x_actual, x_target and pos_comp to the same value to be safe
    self._spi.writeRegister(registers.TMC4361_V_MAX_REGISTER, 0)
    self._spi.writeRegister(registers.TMC4361_X_TARGET_REGISTER, position)
    self._spi.writeRegister(registers.TMC4361_X_ACTUAL_REGISTER, position)
    self._spi.writeRegister(registers.TMC4361_POSITION_COMPARE_REGISTER, position)
    self._last_target = position

    # Put the start register back the way it was
    # FIXME self._spi.writeRegister(registers.TMC4361_START_CONFIG_REGISTER, oldStartRegister)

  def update_move(self):
    # FIXME What is this code supposed to be doing?
    # Modify shadow registers
    # trapezoidal positioning
    self._spi.writeRegister(registers.TMC4361_SH_RAMP_MODE_REGISTER, _BV(2) | 1)
    # set the velocity
    self._spi.writeRegister(registers.TMC4361_SH_V_MAX_REGISTER, FIXED_23_8_MAKE(vMax))
    # set maximum acceleration
    self._spi.writeRegister(registers.TMC4361_SH_A_MAX_REGISTER, fixed_a_max)
    # set maximum deceleration
    self._spi.writeRegister(registers.TMC4361_SH_D_MAX_REGISTER, fixed_a_max)
    # self._spi.writeRegister(registers.TMC4361_SH_V_START_REGISTER,FIXED_23_8_MAKE(vMax/2)); //set start velocity
    # self._spi.writeRegister(registers.TMC4361_SH_V_STOP_REGISTER,FIXED_23_8_MAKE(vMax/2)); //set stop velocity
    # set start velocity
    self._spi.writeRegister(registers.TMC4361_SH_V_START_REGISTER,FIXED_23_8_MAKE(0))
    # set stop velocity
    self._spi.writeRegister(registers.TMC4361_SH_V_STOP_REGISTER,FIXED_23_8_MAKE(0))
    # set start velocity
    self._spi.writeRegister(registers.TMC4361_SH_V_START_REGISTER,FIXED_23_8_MAKE(vStart))
    # set stop velocity
    self._spi.writeRegister(registers.TMC4361_SH_V_STOP_REGISTER,FIXED_23_8_MAKE(vStop))

  def get_current_position(self):
    datagram = self._spi.readRegister(registers.TMC4361_X_ACTUAL_REGISTER)
    value = datagram_to_int(datagram[1:])
    return value

  def get_current_speed(self):
    datagram = self._spi.readRegister(registers.TMC4361_V_ACTUAL_REGISTER)
    value = datagram_to_int(datagram[2:])
    return value

  def get_target_position(self):
    datagram = self._spi.readRegister(registers.TMC4361_X_TARGET_REGISTER)
    value = datagram_to_int(datagram[1:])
    return value

  def signal_start(self):
    #prepare the pos compr registers
    # clear the event register
    self._spi.readRegister(registers.TMC4361_EVENTS_REGISTER)
    pos_comp = next_pos_comp
    # write the new pos_comp
    self._spi.writeRegister(registers.TMC4361_POSITION_COMPARE_REGISTER, next_pos_comp)
    # and mark it written
    next_pos_comp = 0
    direction = next_direction

    # TODO this block is for demonstration only
    # See if motor has reached target position
    mn = 0 # previously a counter variable
    # unsigned char positions_reached
    positions_reached = 0
    # unsigned long motor_pos
    motor_pos = self._spi.readRegister(registers.TMC4361_X_ACTUAL_REGISTER)
    # unsigned long motor_trg
    motor_trg = self._spi.readRegister(registers.TMC4361_X_TARGET_REGISTER)
    if motor_pos == motor_trg:
      # all coordinated motors have reached target position
      pass

    if enable_start:
      if is_new_position:
        self._spi.writeRegister(registers.TMC4361_X_TARGET_REGISTER, next_targets)
        is_new_position = False
        last_targets = next_targets
        # TODO make boolean flag: active_motors |= ( 1 << i )

      # self._spi.readRegister(registers.TMC4361_START_CONFIG_REGISTER)

      # start the motors, please
      GPIO.output(self._start_signal_pin, GPIO.LOW)
      sleep_microseconds(3) # the call by itself may have been enough
      GPIO.output(self._start_signal_pin, GPIO.HIGH)
      enable_start = False

    # self._spi.readRegister(TMC4361_X_ACTUAL_REGISTER)
    # self._spi.readRegister(TMC4361_X_TARGET_REGISTER)
    # self._spi.readRegister(TMC4361_POSITION_COMPARE_REGISTER)
    # self._spi.readRegister(TMC4361_V_MAX_REGISTER)
    # self._spi.readRegister(TMC4361_A_MAX_REGISTER)
    # self._spi.readRegister(TMC4361_D_MAX_REGISTER)

  def configureEndstop(self, left, active_high):
    """
    Arguments:
    (boolean) left
    (boolean) active_high
    """
    # unsigned long endstop_config
    endstop_config = self.getClearedEndStopConfig(left)
    if left:
      if active_high:
        # _BV(0) //STOP_LEFT enable
        # _BV(2) //positive Stop Left stops motor
        # _BV(11) //X_LATCH if stopl becomes active ..
        endstop_config |= 0 | _BV(0) | _BV(2) | _BV(11)
      else:
        # _BV(0) //STOP_LEFT enable
        # _BV(10) //X_LATCH if stopl becomes inactive ..
        endstop_config |= 0 | _BV(0) | _BV(10)
    else: # right
      if active_high:
        # _BV(1) //STOP_RIGHT enable
        # _BV(3) //positive Stop right stops motor
        # _BV(13) //X_LATCH if storl becomes active ..
        endstop_config |= 0 | _BV(1) | _BV(3) | _BV(13)
      else:
        # _BV(1) //STOP_LEFT enable
        # _BV(12) //X_LATCH if stopr becomes inactive ..
        endstop_config |= 0 | _BV(1) | _BV(12)
    self.writeRegister(TMC4361_REFERENCE_CONFIG_REGISTER, endstop_config)

  def configureVirtualEndstop(self, left, positions):
    """
    Arguments:
    (boolean) left
    (long) positions
    """
    # unsigned long endstop_config
    endstop_config = self.getClearedEndStopConfig(left)
    # unsigned long position_register
    if left:
      endstop_config |= _BV(6)
      position_register = registers.TMC4361_VIRTUAL_STOP_LEFT_REGISTER
      # we doe not latch since we know where they are??
    else:
      endstop_config |= _BV(7)
      position_register = registers.TMC4361_VIRTUAL_STOP_RIGHT_REGISTER
      # we doe not latch since we know where they are??
    self.writeRegister(registers.TMC4361_REFERENCE_CONFIG_REGISTER, endstop_config)
    self.writeRegister(position_register, positions)

  def getClearedEndStopConfig(left):
    """
    Arguments:
    (boolean) left

    Returns:
    (unsigned long)
    """
    # unsigned long endstop_config
    endstop_config = self._spi.readRegister(registersTMC4361_REFERENCE_CONFIG_REGISTER)
    # clear everything
    # unsigned long clearing_pattern
    # a trick to ensure the use of all 32 bits
    # TODO make sure we "ensure the use of all 32 bits"
    if left:
      clearing_pattern = registers.TMC4361_LEFT_ENDSTOP_REGISTER_PATTERN
    else: # right
      clearing_pattern = registers.TMC4361_RIGHT_ENDSTOP_REGISTER_PATTERN
    clearing_pattern = ~ clearing_pattern
    endstop_config &= clearing_pattern
    return endstop_config

  def reset(self, shutdown, bringup):
    """
    Arguments:
    (boolean) shutdown
    (boolean) bringup
    """
    if shutdown:
      # Use HWBE pin to reset motion controller TMC4361
      # to check (reset for motion controller)
      PORTE &= ~(_BV(2))
    if shutdown and bringup:
      delay(1);
    if bringup:
      PORTE |= _BV(2)
