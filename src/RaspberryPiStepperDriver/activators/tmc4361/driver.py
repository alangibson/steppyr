import logging
import RPi.GPIO as GPIO
from RaspberryPiStepperDriver import sleep_microseconds, tobin, set_bit, unset_bit, datagram_to_int, _BV
from RaspberryPiStepperDriver.activators.tmc4361.registers import *
from RaspberryPiStepperDriver.activators.tmc26x import registers as tmc26x_registers, TMC26XActivator
from RaspberryPiStepperDriver.activators.tmc4361.spi import TMC26xCoverSPI

log = logging.getLogger(__name__)

# TODO different from standard values
DIRECTION_CW = 1
DIRECTION_CCW = -1

CLOCK_FREQUENCY = 16000000

# Defines ported to Python functions
# simple FP math see https://ucexperiment.wordpress.com/2012/10/28/fixed-point-math-on-the-arduino-platform/
FIXED_23_8_MAKE = lambda a: a * (1 << 8)
FIXED_22_2_MAKE = lambda a: a * (1 << 2)

class TMC4361:

  def __init__(self, spi, reset_pin=None,
      motor_steps_per_rev=200, microsteps=1, pin_mode=GPIO.BCM):
    self._spi = spi
    self._steps_per_revolution = motor_steps_per_rev
    self._reset_pin = reset_pin
    self._microsteps = microsteps

    ### Copied from RampProfile ###
    # Max velocity/speed in steps per second
    # Defaults to 1 to avoid ZeroDivisionError
    self._target_speed = 1.0

    GPIO.setmode(pin_mode)

    # Initialize Registers
    self.spi_status_selection_register = SpiStatusSelectionRegister()
    self.general_configuration_register = GeneralConfigurationRegister()
    self.ramp_mode_register = RampModeRegister()
    self.external_clock_frequency_register = ExternalClockFrequencyRegister(CLOCK_FREQUENCY)
    self.motor_driver_settings_register = MotorDriverSettingsRegister()
    self.v_max_register = VMaxRegister()
    self.v_break_register = VBreakRegister()
    self.a_max_register = AMaxRegister()
    self.d_max_register = DMaxRegister()
    self.v_start_register = VStartRegister()
    self.v_stop_register = VStopRegister()
    self.bow1_register = Bow1Register()
    self.bow2_register = Bow2Register()
    self.bow3_register = Bow3Register()
    self.bow4_register = Bow4Register()
    self.a_start_register = AStartRegister()
    self.d_final_register = DFinalRegister()

  def flush_registers(self):
    self._spi.write(self.spi_status_selection_register)
    self._spi.write(self.general_configuration_register)
    self._spi.write(self.ramp_mode_register)
    self._spi.write(self.external_clock_frequency_register)
    self._spi.write(self.motor_driver_settings_register)
    self._spi.write(self.v_max_register)
    self._spi.write(self.v_break_register)
    self._spi.write(self.a_max_register)
    self._spi.write(self.d_max_register)
    self._spi.write(self.v_start_register)
    self._spi.write(self.v_stop_register)
    self._spi.write(self.bow1_register)
    self._spi.write(self.bow2_register)
    self._spi.write(self.bow3_register)
    self._spi.write(self.bow4_register)
    self._spi.write(self.a_start_register)
    self._spi.write(self.d_final_register)

  #
  # Activator API methods
  #

  def start(self):
    """
    "Per default, the voltage level transition from high to low triggers a start
    signal (START is an input), or START output indicates an active START event
    by switching from high to low level." Datasheet 9.1.3

    Implements Activator.start() method.
    """
    if self._reset_pin:
      GPIO.setup(self._reset_pin, GPIO.OUT)
    self.reset(hard=True)
    # Select the events we want reported with every status code.
    # see datasheet 19.13-19.14
    self._spi.write(self.spi_status_selection_register
      .set(StatusEventRegister.bits.TARGET_REACHED)
      .set(StatusEventRegister.bits.POS_COMP_REACHED)
    )
    self._spi.write(self.general_configuration_register
      .set(GeneralConfigurationRegister.bits.POL_DIR_OUT)
    )

    # TODO ? self._spi.writeRegister(registers.TMC4361_SH_RAMP_MODE_REGISTER, RAMP_MODE_POSITIONING_MODE | RAMP_MODE_NO_RAMP)

    self._spi.write(self.external_clock_frequency_register)

    # TODO ? NEEDED so THAT THE SQUIRREL CAN RECOMPUTE EVERYTHING!
    # self._spi.writeRegister(registers.TMC4361_START_DELAY_REGISTER, 512)
    self.set_motor_steps_per_rev(self._steps_per_revolution)

    # TODO needed?
    # filter start (unsigned long filter)
    # 0x20000 = 2 << 16, 0x400000 = 4 << 20
    # filter = 0x20000 | 0x400000
    # filter ref
    # filter |= (2<<8) | 0x4000
    # self._spi.writeRegister(registers.TMC4361_INPUT_FILTER_REGISTER, filter)

    self.set_ramp_trapezoid(v_max=0, a_max=0, d_max=0)

    self.enable_tmc26x()

  def stop(self):
    """
    Implements Activator.stop() method.
    """
    self.disable_tmc26x()
    self.reset(hard=True)

  def enable(self):
    """
    Implements Activator.enable() method.
    """
    # TODO implement
    pass

  def disable(self):
    """
    Implements Activator.disable() method.
    """
    # TODO implement
    pass

  def step(self, direction=None):
    """
    Implements Activator.step(direction) method.
    Implements Profile.step() method.
    """
    pass

  # FIXME need to do lookup for microsteps
  def set_microsteps(self, microsteps):
    """
    Implements Activator.set_microsteps(microsteps) method.
    Implements Profile.set_microsteps(microsteps) method.
    """
    self._spi.write(self.motor_driver_settings_register
      .set(MotorDriverSettingsRegister.bits.MSTEP_PER_FS, 8) # full stepping
    )

  @property
  def pulse_width(self):
    """
    Implements Activator.pulse_width method.
    """
    return 0

  def set_pulse_width(self, pulse_width_us):
    """
    Implements Activator.set_pulse_width(pulse_width_us) method.
    """
    pass

  #
  # Profile API methods
  #

  def set_target_speed(self, speed):
    """
    Implements Profile.set_target_speed(speed) method.
    """
    self._target_speed = speed
    # 24 bits integer part, 8 bits decimal places
    # v_max = speed << 8
    # self._spi.writeRegister(TMC4361_V_MAX_REGISTER, v_max)
    self._spi.write(self.v_max_register.set(VMaxRegister.bits.VMAX, speed))

  def set_target_steps(self, absolute_steps):
    """
    Implements Profile.set_target_steps(absolute_steps) method.
    Acts like StepperDriver.move_to(position) method
    """
    # FIXME We want an immediate start, so temporarily modify the start register.
    # oldStartRegister = self._spi.readRegister(registers.TMC4361_START_CONFIG_REGISTER)
    # self._spi.writeRegister(registers.TMC4361_START_CONFIG_REGISTER, _BV(5)) # keep START as an input

    self._spi.writeRegister(TMC4361_X_TARGET_REGISTER, absolute_steps)
    # self._spi.writeRegister(registers.TMC4361_X_ACTUAL_REGISTER, position)
    # self._spi.writeRegister(registers.TMC4361_POSITION_COMPARE_REGISTER, position)
    self._last_target = absolute_steps

    # Put the start register back the way it was
    # FIXME self._spi.writeRegister(registers.TMC4361_START_CONFIG_REGISTER, oldStartRegister)

  def set_acceleration(self, acceleration):
    """
    Implements Profile.set_acceleration(acceleration) method.

    acceleration: (int) steps / sec / sec
    """
    # "22 digits and 2 decimal places:"
    # self._spi.writeRegister(TMC4361_A_MAX_REGISTER, acceleration << 2)
    # self._spi.writeRegister(TMC4361_D_MAX_REGISTER, acceleration << 2)
    self._spi.write(self.a_max_register.set(AMaxRegister.bits.AMAX, acceleration))
    self._spi.write(self.d_max_register.set(DMaxRegister.bits.DMAX, acceleration))

  def set_motor_steps_per_rev(self, steps_per_rev):
    """
    Implements Profile.set_motor_steps_per_rev(steps_per_rev) method.
    """
    self._motor_steps_per_rev = steps_per_rev
    self._spi.write(self.motor_driver_settings_register
      .set(MotorDriverSettingsRegister.bits.FS_PER_REV, self._steps_per_revolution) )

  def set_current_position(self, position):
    """
    Implements Profile.set_current_position(position) method.
    """
    # TODO return self._spi.read(XActualRegister.bits.XACTUAL)
    pass

  @property
  def acceleration(self):
    datagram = self._spi.readRegister(AActualRegister.REGISTER)
    return datagram_to_int(datagram[1:])

  @property
  def distance_to_go(self):
    """
    Implements Profile.distance_to_go method.
    """
    return self.target_steps - self.current_steps

  @property
  def is_moving(self):
    """
    Implements Profile.is_moving method.
    """
    return 'TARGET_REACHED_F' not in [t[0] for t in self.get_status_flags()]

  @property
  def direction(self):
    """
    Implements Profile.direction method.
    """
    # TODO Implement
    pass

  @property
  def current_steps(self):
    # datagram = self._spi.readRegister(XActualRegister.REGISTER)
    # return datagram_to_int(datagram[1:])
    return self._spi.read(XActualRegister()).get(XActualRegister.bits.XACTUAL)

  def compute_new_speed(self):
    pass

  def should_step(self):
    # Profile should keep calling step() while we are moving.
    return self.is_moving

  #
  # StepperDriver API methods
  #

  def abort(self):
    """
    Implements StepperDriver.abort() method.
    """
    # TODO In order to stop the motion during positioning, do as follows:
    # Action: Set VMAX = 0 (register 0x24).
    pass

  # TODO const char homeMotorTMC4361

  #
  # Non-API methods
  #

  @property
  def target_acceleration(self):
    return self._spi.read(AMaxRegister()).get(AMaxRegister.bits.AMAX)

  @property
  def target_deceleration(self):
    return self._spi.read(DMaxRegister()).get(DMaxRegister.bits.DMAX)

  def set_ramp_scurve(self, v_max, a_max, d_max, bow1, bow2, bow3, bow4,
    a_start=0, d_final=0,
    motion_profile=2, operation_mode=1,
    v_start=0, v_stop=0):
    """
    In order to make use of S-shaped ramps, do as follows:
    Action:
      - Set RAMPMODE(1:0)=b’10 (register 0x20).
      - Set proper BOW1 … BOW4 registers 0x2C…0x30.
      - Set proper AMAX register 0x28 and DMAX register 0x29.
      - Set ASTART = 0 (register 0x2A).
      - Set DFINAL = 0 (register 0x2B).
      - Set proper VMAX register 0x24.
      - (Optional) Set proper VSTART > 0 (register 0x25).
      - (Optional) Set propert VSTOP > 0 (register 0x26).
    Result:
      The internal velocity VACTUAL is changed successively to VMAX with S-shaped ramps.
      The acceleration/deceleration values are altered on the basis of the bow values.

    In order to configure S-shaped ramps with starting and finishing values for
    acceleration or deceleration, do as follows:
    Action:
      - Set S-Shaped ramp as explained above
      - Set proper ASTART register 0x2A.
      - Set proper DFINAL register 0x2B.
    Result:
      The internal velocity VACTUAL is changed successively to VMAX with S-Shaped ramps.

    Definitions for S-shaped Ramps
    - The acceleration/deceleration values are altered, based on the bow values.
    - The start phase and the end phase of an S-shaped ramp is
      accelerated/decelerated by ASTART and DFINAL.
    - The ramp starts with ASTART and stops with DFINAL.
    - DFINAL becomes valid when AACTUAL reaches the chosen DFINAL value.
    - The parameter DFINAL is not considered during positioning mode.

    Rising slope (absolute lower velocities to absolute higher velocities):
      - BOW1 determines the value which increases the absolute acceleration value.
      - BOW2 determines the value which decreases the absolute acceleration value.
      - AMAX determines the maximum acceleration value.
    Falling slope (absolute higher velocities to absolute lower velocities):
      - BOW3 determines the value which increases the absolute deceleration value.
      - BOW4 determines the value which decreases the absolute deceleration value.
      - DMAX determines the maximum absolute deceleration value.
    """
    self._target_speed = v_max

    self.ramp_mode_register\
      .set(RampModeRegister.bits.MOTION_PROFILE, motion_profile)\
      .set(RampModeRegister.bits.OPERATION_MODE, operation_mode)
    self.bow1_register.set(Bow1Register.bits.BOW1, bow1)
    self.bow2_register.set(Bow2Register.bits.BOW2, bow2)
    self.bow3_register.set(Bow3Register.bits.BOW3, bow3)
    self.bow4_register.set(Bow4Register.bits.BOW4, bow4)
    self.v_max_register.set(VMaxRegister.bits.VMAX, v_max)
    self.a_max_register.set(AMaxRegister.bits.AMAX, a_max)
    self.d_max_register.set(DMaxRegister.bits.DMAX, d_max)
    self.v_start_register.set(VStartRegister.bits.VSTART, v_start)
    self.v_stop_register.set(VStopRegister.bits.VSTOP, v_stop)
    self.a_start_register.set(AStartRegister.bits.ASTART, a_start)
    self.d_final_register.set(DFinalRegister.bits.DFINAL, d_final)
    self.flush_registers()

  def set_ramp_none(self):
    """
    Motion profile No Ramp follows VMAX only (rectangular velocity shape).

    In Velocity mode
    Action:
      - Set RAMPMODE(1:0) =b’00 (register 0x20).
      - Set proper VMAX register 0x24.
    Result:
      The internal velocity VACTUAL is immediately set to VMAX.

    In Positioning mode
    Determines that the ramp holds VMAX until XTARGET is reached.
    The motion direction depends on XTARGET.
    Action:
      - Set RAMPMODE(2:0) =b’100.
      - Set proper VMAX register 0x24.
      - Set proper XTARGET register 0x37.
    Result:
      VACTUAL is set instantly to 0 in case the target position is reached.

    TODO Do NOT exceed VMAX ≤ fCLK / 4 pulses for positioning mode.
    """
    self._spi.write(self.ramp_mode_register
      .set(RampModeRegister.bits.MOTION_PROFILE, 0) # no ramp
      .set(RampModeRegister.bits.OPERATION_MODE, 1) # positioning mode
    )

  def set_ramp_trapezoid(self, v_max, a_max, d_max,
    motion_profile=1, operation_mode=1, v_break=0,
    v_start=0, v_stop=0):
    """
    Consideration of acceleration and deceleration values
    without adaptation of these acceleration values.

    Action:
      - Set RAMPMODE(1:0) =b’01 (register 0x20).
      - Set VBREAK =0 (register 0x27).
      - Set proper AMAX register 0x28 and DMAX register 0x29.
      - Set proper VMAX register 0x24.
      - (Optional) Set proper VSTART > 0 (register 0x25).
      - (Optional) Set propert VSTOP > 0 (register 0x26).
    Result:
      The internal velocity VACTUAL is changed successively to VMAX with a linear ramp.
      Only AMAX and DMAX define the acceleration/deceleration slopes.

    AMAX determines the rising slope from absolute low to absolute high velocities,
    whereas DMAX determines the falling slope from absolute high to absolute low
    velocities.
    Acceleration slope and deceleration slopes have only one acceleration and
    deceleration value each.
    """
    self.ramp_mode_register\
      .set(RampModeRegister.bits.MOTION_PROFILE, motion_profile)\
      .set(RampModeRegister.bits.OPERATION_MODE, operation_mode)
    self.v_max_register.set(VMaxRegister.bits.VMAX, v_max)
    self.v_break_register.set(VBreakRegister.bits.VBREAK, 0)
    self.a_max_register.set(AMaxRegister.bits.AMAX, a_max)
    self.d_max_register.set(DMaxRegister.bits.DMAX, d_max)
    self.v_start_register.set(VStartRegister.bits.VSTART, v_start)
    self.v_stop_register.set(VStopRegister.bits.VSTOP, v_stop)
    self.flush_registers()

  def set_ramp_sixpoint(self, v_max, v_break, a_start, a_max, d_max, d_final,
    motion_profile=1, operation_mode=1,
    v_start=0, v_stop=0):
    """
    Consideration of acceleration and deceleration values
    without adaptation of these acceleration values

    Action:
      - Set RAMPMODE(1:0)=b’01 (register 0x20).
      - Set proper VBREAK register 0x27.
      - Set proper AMAX register 0x28 and DMAX register 0x29.
      - Set proper ASTART register 0x2A and DFINAL register 0x2B.
      - Set proper VMAX register 0x24.
      - (Optional) Set proper VSTART > 0 (register 0x25).
      - (Optional) Set propert VSTOP > 0 (register 0x26).
    Result:
      The internal velocity VACTUAL is changed successively to VMAX with a linear ramp. In
      addition to AMAX and DMAX, ASTART and DFINAL define the acceleration or
      deceleration slopes (see Figure above).

    AMAX and ASTART determines the rising slope from absolute low to absolute
    high velocities.
    DMAX and DFINAL determines the falling slope from absolute high to absolute
    low velocities.
    The acceleration/deceleration factor alters at VBREAK. ASTART and DFINAL are
    valid below VBREAK, whereas AMAX and DMAX are valid beyond VBREAK.
    """
    pass

  def update_move(self):
    # FIXME What is this code supposed to be doing?
    # Modify shadow registers
    # trapezoidal positioning
    self._spi.writeRegister(TMC4361_SH_RAMP_MODE_REGISTER, _BV(2) | 1)
    # set the velocity
    self._spi.writeRegister(TMC4361_SH_V_MAX_REGISTER, FIXED_23_8_MAKE(vMax))
    # set maximum acceleration
    self._spi.writeRegister(TMC4361_SH_A_MAX_REGISTER, fixed_a_max)
    # set maximum deceleration
    self._spi.writeRegister(TMC4361_SH_D_MAX_REGISTER, fixed_a_max)
    # self._spi.writeRegister(registers.TMC4361_SH_V_START_REGISTER,FIXED_23_8_MAKE(vMax/2)); //set start velocity
    # self._spi.writeRegister(registers.TMC4361_SH_V_STOP_REGISTER,FIXED_23_8_MAKE(vMax/2)); //set stop velocity
    # set start velocity
    self._spi.writeRegister(TMC4361_SH_V_START_REGISTER,FIXED_23_8_MAKE(0))
    # set stop velocity
    self._spi.writeRegister(TMC4361_SH_V_STOP_REGISTER,FIXED_23_8_MAKE(0))
    # set start velocity
    self._spi.writeRegister(TMC4361_SH_V_START_REGISTER,FIXED_23_8_MAKE(vStart))
    # set stop velocity
    self._spi.writeRegister(TMC4361_SH_V_STOP_REGISTER,FIXED_23_8_MAKE(vStop))

  @property
  def current_speed(self):
    datagram = self._spi.readRegister(VActualRegister.REGISTER)
    value = datagram_to_int(datagram[2:])
    return value

  @property
  def target_steps(self):
    datagram = self._spi.readRegister(TMC4361_X_TARGET_REGISTER)
    value = datagram_to_int(datagram[1:])
    return value

  @property
  def target_speed(self):
    datagram = self._spi.readRegister(VMaxRegister.REGISTER)
    value = datagram_to_int(datagram[1:])
    # 24 bits integer part, 8 bits decimal places
    value = value >> 8
    return value

  def get_status_flags(self):
    return self._spi.read(StatusFlagRegister()).get_values()

  def get_status_events(self):
    """
    Read and parse the Status Event Register (EVENTS 0x0E).
    """
    return self._spi.read(StatusEventRegister()).get_values()

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
      position_register = TMC4361_VIRTUAL_STOP_LEFT_REGISTER
      # we doe not latch since we know where they are??
    else:
      endstop_config |= _BV(7)
      position_register = TMC4361_VIRTUAL_STOP_RIGHT_REGISTER
      # we doe not latch since we know where they are??
    self.writeRegister(TMC4361_REFERENCE_CONFIG_REGISTER, endstop_config)
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
      clearing_pattern = TMC4361_LEFT_ENDSTOP_REGISTER_PATTERN
    else: # right
      clearing_pattern = TMC4361_RIGHT_ENDSTOP_REGISTER_PATTERN
    clearing_pattern = ~ clearing_pattern
    endstop_config &= clearing_pattern
    return endstop_config

  def reset(self, hard=False):
    if hard and self._reset_pin:
      GPIO.output(self._reset_pin, GPIO.LOW)
      sleep_microseconds(10)
      GPIO.output(self._reset_pin, GPIO.HIGH)
    else:
      # FIXME soft reset doesnt seem to work
      self._spi.write(ResetClockAndGatingRegister()
        .set(ResetClockAndGatingRegister.bits.RESET_REG, 0x525354) )

  def enable_tmc26x(self):
    """
    Enable special support for TCM26x drivers.
    See section 10.6 in the TMC4361 datasheet.
    """
    # Configure SPI Output Conf Register to talk to TMC26x
    # TMC_260_CONFIG 0x8440000a //SPI-Out: block/low/high_time=8/4/4 Takte; CoverLength=autom; TMC26x
    log.debug('Setting SPIOutConfRegister for TCM26x')
    self._spi.write(SPIOutConfRegister()
      .set(SPIOutConfRegister.bits.SPI_OUTPUT_FORMAT, 10)
      .set(SPIOutConfRegister.bits.COVER_DATA_LENGTH, 0)
      .set(SPIOutConfRegister.bits.AUTOREPEAT_COVER_EN)
      .set(SPIOutConfRegister.bits.SPI_OUT_LOW_TIME, 4)
      .set(SPIOutConfRegister.bits.SPI_OUT_HIGH_TIME, 4)
      .set(SPIOutConfRegister.bits.SPI_OUT_BLOCK_TIME, 8)
    )
    # Initialize TMC26x
    log.debug('Initializing TCM26x')
    cover_spi = TMC26xCoverSPI(self._spi)
    self.tmc26x = TMC26XActivator(spi=cover_spi, dir_pin=0, step_pin=0, current=300, resistor=150)
    self.tmc26x.start()

  def disable_tmc26x(self):
    if self.tmc26x:
      self.tmc26x.stop()

  def transfer_to_tmc2660(self, datagram):
    """
    Datagram should be 20 bits.
    """
    self._spi.write(CoverLowRegister(datagram))
    resp = self._spi.read(CoverDriverLowRegister())
    return resp

  def get_tmc2660_response(self):
    return self._spi.read(CoverDriverLowRegister())
