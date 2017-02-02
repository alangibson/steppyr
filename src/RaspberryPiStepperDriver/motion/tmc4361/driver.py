import RPi.GPIO as GPIO
from RaspberryPiStepperDriver import sleep_microseconds, tobin, set_bit, unset_bit, datagram_to_int
from RaspberryPiStepperDriver.motion.tmc4361.registers import *
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
# General Conf Register
# GENERAL_CONF_SERIAL_ENC_OUT_ENABLE = _BV(24)
# Start Switch Configuration Register
START_CONFIG_START_EN_NONE = 0b00000
START_CONFIG_TRIGGER_EVENTS_DISABLED = 0b0000
START_CONFIG_TRIGGER_EVENTS_START_PIN_OUTPUT = 0b0000
START_CONFIG_TRIGGER_EVENTS_START_PIN_TRIGGER = 0b0001
START_CONFIG_TRIGGER_EVENTS_TARGET_REACHED_TRIGGER = 0b0010
START_CONFIG_TRIGGER_EVENTS_VELOCITY_REACHED_TRIGGER = 0b0100
START_CONFIG_TRIGGER_EVENTS_POSCOMP_REACHED_TRIGGER = 0b1000
# Status Event Register
STATUS_EVENT_TARGET_REACHED = _BV(0)
STATUS_EVENT_POS_COMP_REACHED = _BV(1)
STATUS_EVENT_VEL_REACHED = _BV(2)
# TODO finish these
STATUS_EVENT_COVER_DONE = _BV(25)
# TODO finish these
STATUS_EVENT_RST_EV = _BV(31)
# Status Flag Register
STATUS_FLAG_TARGET_REACHED_F = _BV(0)
STATUS_FLAG_POS_COMP_REACHED_F = _BV(1)
STATUS_FLAG_VEL_REACHED_F = _BV(3)
STATUS_FLAG_VEL_STATE_F = _BV(4) | _BV(3)
STATUS_FLAG_RAMP_STATE_F = _BV(6) | _BV(5)
STATUS_FLAG_STOPL_ACTIVE_F = _BV(7)
STATUS_FLAG_STOPR_ACTIVE_F = _BV(8)
STATUS_FLAG_VSTOPL_ACTIVE_F = _BV(9)
# TODO finish missing
STATUS_FLAG_DRIVER_STALLGUARD2 = _BV(24)
STATUS_FLAG_DRIVER_OT_SHUTDOWN = _BV(25)
STATUS_FLAG_DRIVER_OT_WARNING = _BV(26)
STATUS_FLAG_DRIVER_S2GA = _BV(27)
STATUS_FLAG_DRIVER_S2GB = _BV(28)
STATUS_FLAG_DRIVER_OLA = _BV(29)
STATUS_FLAG_DRIVER_OLB = _BV(30)
STATUS_FLAG_DRIVER_STST_OR_OCHS = _BV(31)
# SPI Out Conf Register
SPI_OUT_CONFIG_TMC26X_SPI = 0b1010
SPI_OUT_CONFIG_TMC26X_SD = 0b1011
SPI_OUT_CONFIG_SPI_OUTPUT_FORMAT_OFF = 0b0000
SPI_OUT_CONFIG_SPI_OUTPUT_FORMAT_SPIDAC_MAPPED = 0b0001
SPI_OUT_CONFIG_SPI_OUTPUT_FORMAT_SPIDAC_ABS_POS = 0b0010
SPI_OUT_CONFIG_SPI_OUTPUT_FORMAT_SPIDAC_ABS_NEG = 0b0011
# TODO finish these
SPI_OUT_CONFIG_SPI_OUTPUT_FORMAT_TMC26X_SPI = 0b1010
SPI_OUT_CONFIG_SPI_OUTPUT_FORMAT_TMC26X_SD = 0b1011

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
    self._reset_pin = 26
    # TODO make configurable
    GPIO.setmode(GPIO.BCM)

  def start(self):
    """
    "Per default, the voltage level transition from high to low triggers a start
    signal (START is an input), or START output indicates an active START event
    by switching from high to low level." Datasheet 9.1.3
    """
    GPIO.setup(self._reset_pin, GPIO.OUT)
    # GPIO.setup(self._start_signal_pin, GPIO.OUT)
    # GPIO.output(self._start_signal_pin, GPIO.LOW)
    # will be released after setup is complete
    # GPIO.setup(self._target_reached_interrupt_pin, GPIO.OUT)
    # GPIO.output(self._target_reached_interrupt_pin, GPIO.LOW)

    # _BV(5) external start is an start (to ensure start is an input)
    # "External start signal is enabled as timer trigger. START pin is assigned as input."
    # self._spi.writeRegister(registers.TMC4361_START_CONFIG_REGISTER,
    #   0 | START_CONFIG_TRIGGER_EVENTS_DISABLED)

    # GPIO.output(self._start_signal_pin, GPIO.HIGH)
    # GPIO.setup(self._start_signal_pin, GPIO.IN)

    # preconfigure the TMC4361

    self.reset(hard=True)

    # Select the events we want reported with every status code.
    # see datasheet 19.13-19.14
    # Old api: self._spi.writeRegister(registers.TMC4361_SPI_STATUS_SELECTION_REGISTER, 0 | _BV(0) | _BV(1))
    self._spi.write(SpiStatusSelectionRegister()
      .set(StatusEventRegister.bits.TARGET_REACHED)
      .set(StatusEventRegister.bits.POS_COMP_REACHED)
    )

    # self._spi.writeRegister(registers.TMC4361_GENERAL_CONFIG_REGISTER, 0 | _BV(5) | GENERAL_CONF_SERIAL_ENC_OUT_ENABLE)
    self._spi.write(GeneralConfigurationRegister()
      .set(GeneralConfigurationRegister.bits.POL_DIR_OUT)
    )

    # Set positioning and ramp shape
    # self._spi.writeRegister(registers.TMC4361_RAMP_MODE_REGISTER, RAMP_MODE_POSITIONING_MODE | RAMP_MODE_S_SHAPED_RAMP)
    # self._spi.writeRegister(registers.TMC4361_SH_RAMP_MODE_REGISTER, RAMP_MODE_POSITIONING_MODE | RAMP_MODE_S_SHAPED_RAMP)
    # Section 6.3.4 No Ramp Motion Profile
    # self._spi.writeRegister(registers.TMC4361_RAMP_MODE_REGISTER, RAMP_MODE_POSITIONING_MODE | RAMP_MODE_NO_RAMP)
    self._spi.write(RampModeRegister()
      # .set(RampModeRegister.bits.MOTION_PROFILE, 2) # S-shaped ramp
      .set(RampModeRegister.bits.MOTION_PROFILE, 0) # no ramp
      .set(RampModeRegister.bits.OPERATION_MODE, 1) # positioning mode
    )
    # TODO ? self._spi.writeRegister(registers.TMC4361_SH_RAMP_MODE_REGISTER, RAMP_MODE_POSITIONING_MODE | RAMP_MODE_NO_RAMP)

    # self._spi.writeRegister(registers.TMC4361_CLK_FREQ_REGISTER, CLOCK_FREQUENCY)
    self._spi.write(ExternalClockFrequencyRegister(CLOCK_FREQUENCY))

    # NEEDED so THAT THE SQUIRREL CAN RECOMPUTE EVERYTHING!
    # self._spi.writeRegister(registers.TMC4361_START_DELAY_REGISTER, 512)
    self.set_motor_steps_per_rev(self._steps_per_revolution)
    # self._spi.writeRegister(registers.TMC4361_START_CONFIG_REGISTER, default_4361_start_config);
    # filter start (unsigned long filter)
    # 0x20000 = 2 << 16, 0x400000 = 4 << 20
    # filter = 0x20000 | 0x400000
    # filter ref
    # filter |= (2<<8) | 0x4000
    # self._spi.writeRegister(registers.TMC4361_INPUT_FILTER_REGISTER, filter)

  def stop(self):
    self.reset(hard=True)

  def abort(self):
    # TODO In order to stop the motion during positioning, do as follows:
    # Action: Set VMAX = 0 (register 0x24).
    pass

  def set_motor_steps_per_rev(self, steps_per_rev):
    self._motor_steps_per_rev = steps_per_rev
    # Configure the motor type
    # FIXME dont asume 0 microsteps
    # unsigned long motorconfig
    # motorconfig = 0x00 # we want 256 microsteps
    # motorconfig |= steps_per_rev << 4
    # self._spi.writeRegister(registers.TMC4361_STEP_CONF_REGISTER, motorconfig)
    self._spi.write(MotorDriverSettingsRegister()
      .set(MotorDriverSettingsRegister.bits.MSTEP_PER_FS, 8) # full stepping
      .set(MotorDriverSettingsRegister.bits.FS_PER_REV, 200) # 1.8 degree per step
    )

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
    # self._spi.writeRegister(registers.TMC4361_V_MAX_REGISTER, 0)

    # 24 bits integer part, 8 bits decimal places
    v_max = 500
    v_max = v_max << 8
    self._spi.writeRegister(registers.TMC4361_V_MAX_REGISTER, v_max)
    self._spi.writeRegister(registers.TMC4361_X_TARGET_REGISTER, position)
    # self._spi.writeRegister(registers.TMC4361_X_ACTUAL_REGISTER, position)
    # self._spi.writeRegister(registers.TMC4361_POSITION_COMPARE_REGISTER, position)
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

  def get_target_speed(self):
    datagram = self._spi.readRegister(registers.TMC4361_V_MAX_REGISTER)
    value = datagram_to_int(datagram[1:])
    # 24 bits integer part, 8 bits decimal places
    value = value >> 8
    return value

  def get_status_flags(self):
    parsed_datagram = self._spi.readRegister(registers.TMC4361_STATUS_REGISTER)
    datagram = datagram_to_int(parsed_datagram[1:])
    out = []
    for i in range(0,31):
      if datagram & _BV(i):
        out.append(_BV(i))
    return out

  def get_status_events(self):
    """
    Read and parse the Status Event Register (EVENTS 0x0E).
    """
    parsed_datagram = self._spi.readRegister(registers.TMC4361_EVENTS_REGISTER)
    datagram = datagram_to_int(parsed_datagram[1:])
    out = []
    for i in range(0,31):
      if datagram & _BV(i):
        out.append(_BV(i))
    return out

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

  def reset(self, hard=False):
    if hard:
      GPIO.output(self._reset_pin, GPIO.LOW)
      sleep_microseconds(10)
      GPIO.output(self._reset_pin, GPIO.HIGH)
    else:
      # FIXME soft reset doesnt seem to work
      # Do software reset by setting RESET_REG = 0x525354 (Bits31:8 of register 0x4F)
      rst = 0x525354
      rst = rst << 8
      out = self._spi.writeRegister(registers.TMC4361_RESET_CLK_GATING_REGISTER, rst)
      # print(bin(out[0]))
      # print(self._spi.readRegister(registers.TMC4361_RESET_CLK_GATING_REGISTER))
      # print('resetting')
      # print(STATUS_EVENT_RST_EV in self.get_status_events())

  def enable_tmc26x(self):
    """
    Enable special support for TCM26x drivers.
    See section 10.6 in the TMC4361 datasheet.
    """
    # autorepeat_cover_en
    datagram = 0 | _BV(7)
    datagram = datagram | SPI_OUT_CONFIG_TMC26X_SPI

    """
    Set the number of internal clock cycles the serial clock should stay low at
    SPI_OUT_LOW_TIME = SPIOUT_CONF (23:20).
     Set the number of internal clock cycles the serial clock should stay high at
    SPI_OUT_HIGH_TIME = SPIOUT_CONF (27:24).
     Also, an SPI_OUT_BLOCK_TIME = SPIOUT_CONF(31:28) can be set for a
    minimum time period during which no new datagram is sent after the last SPI
    output datagram.
    """
    spi_out_block_time = 2 # clock cycles
    spi_out_low_time = 2 # clock cycles
    spi_out_high_time = 2 # clock cycles
    datagram = spi_out_block_time << 28
    datagram |= spi_out_high_time << 24
    datagram |= spi_out_low_time << 20
    datagram |= SPI_OUT_CONFIG_TMC26X_SD
    self._spi.writeRegister(registers.TMC4361_SPIOUT_CONF_REGISTER, datagram)

  def transfer_to_tmc2660(self, datagram):
    """
    Datagram should be 20 bits.
    """
    spiout_conf = self._spi.readRegister(registers.TMC4361_SPIOUT_CONF_REGISTER)
    spiout_conf = datagram_to_int(spiout_conf[1:])
    cover_data_length = 20
    new_spiout_conf = spiout_conf | ( cover_data_length << 13 )
    print('spiout_conf', bin(spiout_conf), '->', bin(new_spiout_conf))

    # donk = ( new_spiout_conf & ( 0b111111 << 13 ) ) >> 13
    # print('donk', donk)

    self._spi.writeRegister(registers.TMC4361_SPIOUT_CONF_REGISTER, new_spiout_conf)

    self._spi.writeRegister(registers.TMC4361_COVER_LOW_REGISTER, datagram)
    resp = self._spi.readRegister(registers.TMC4361_COVER_DRV_LOW_REGISTER)

    self._spi.writeRegister(registers.TMC4361_SPIOUT_CONF_REGISTER, spiout_conf)

    return resp
