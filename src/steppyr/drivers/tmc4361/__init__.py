import logging
import RPi.GPIO as GPIO
import steppyr
from steppyr.drivers import Driver
from steppyr.drivers.tmc26x import TMC26XDriver
from steppyr.drivers.tmc4361.registers import *
from steppyr.drivers.tmc4361.spi import TMC26xCoverSPI
from steppyr.lib.bits import _BV
from steppyr.lib.functions import sleep_microseconds
from steppyr.lib.trinamic import MICROSTEP_RESOLUTION, parse_ini

log = logging.getLogger(__name__)

# Toggle calling of the report() function in should_step()
REPORT = False

# Defines ported to Python functions
# simple FP math see https://ucexperiment.wordpress.com/2012/10/28/fixed-point-math-on-the-arduino-platform/
FIXED_23_8_MAKE = lambda a: a * (1 << 8)
FIXED_22_2_MAKE = lambda a: a * (1 << 2)

class TMC4361Driver(Driver):

  def __init__(self, spi, reset_pin=None, pin_mode=GPIO.BCM, clock_frequency=4000000):
    self._spi = spi
    self._reset_pin = reset_pin
    self._pin_mode = pin_mode
    self._last_target_speed = None
    # Initialize TMC26x driver
    self.tmc26x = TMC26XDriver(spi=TMC26xCoverSPI(self._spi), dir_pin=0, step_pin=0, current=300, resistor=150)
    # Initialize Registers with defaults
    self._registers = {
      ReferenceConfRegister: ReferenceConfRegister(0x0),
      SpiStatusSelectionRegister: SpiStatusSelectionRegister(0x82029805),
      SPIOutConfRegister: SPIOutConfRegister(0x0),
      GeneralConfigurationRegister: GeneralConfigurationRegister(0x00006020)\
        .set(GeneralConfigurationRegister.bits.POL_DIR_OUT),
      RampModeRegister: RampModeRegister(0x0),
      ExternalClockFrequencyRegister: ExternalClockFrequencyRegister(clock_frequency),
      MotorDriverSettingsRegister: MotorDriverSettingsRegister(0x00FB0C80),
      VMaxRegister: VMaxRegister(0x00000000),
      VBreakRegister: VBreakRegister(0x00000000),
      AMaxRegister: AMaxRegister(0x00000000),
      DMaxRegister: DMaxRegister(0x00000000),
      VStartRegister: VStartRegister(0x00000000),
      VStopRegister: VStopRegister(0x000000),
      Bow1Register: Bow1Register(0x000000),
      Bow2Register: Bow2Register(0x000000),
      Bow3Register: Bow3Register(0x000000),
      Bow4Register: Bow4Register(0x000000),
      AStartRegister: AStartRegister(0x000000),
      DFinalRegister: DFinalRegister(0x000000),
      XActualRegister: XActualRegister(0x00000000),
      AActualRegister: AActualRegister(0x00000000),
      VActualRegister: VActualRegister(0x00000000),
      XTargetRegister: XTargetRegister(0x000000),
      StatusFlagRegister: StatusFlagRegister(),
      StatusEventRegister: StatusEventRegister()\
        .set(StatusEventRegister.bits.TARGET_REACHED)
        .set(StatusEventRegister.bits.POS_COMP_REACHED),
      CurrentScaleValuesRegister: CurrentScaleValuesRegister(0xFFFFFFFF)\
        .set(CurrentScaleValuesRegister.bits.HOLD_SCALE_VAL, 100),
      CurrentScalingConfRegister: CurrentScalingConfRegister(0x0)\
        .set(CurrentScalingConfRegister.bits.HOLD_CURRENT_SCALE_EN, 1),

      StandbyDelayRegister: StandbyDelayRegister(0x0),
      FreewheelDelayRegister: FreewheelDelayRegister(0x0),
      VDRVScaleLimitRegister: VDRVScaleLimitRegister(0x0),
      UpScaleDelayRegister: UpScaleDelayRegister(0x0),
      HoldScaleDelayRegister: HoldScaleDelayRegister(0x0),
      DriveScaleDelayRegister: DriveScaleDelayRegister(0x0),
      BoostTimeRegister: BoostTimeRegister(0x0),
      #BetaGammaRegister: BetaGammaRegister(),
      #SpiDacAddressRegister: SpiDacAddressRegister(),
      FullStepVelocityRegister: FullStepVelocityRegister(0x0),
      #MSLUT0Register: MSLUT0Register(),
      #MSLUT1Register: MSLUT1Register(),
      #MSLUT2Register: MSLUT2Register(),
      #MSLUT3Register: MSLUT3Register(),
      #MSLUT4Register: MSLUT4Register(),
      #MSLUT5Register: MSLUT5Register(),
      #MSLUT6Register: MSLUT6Register(),
      #MSLUT7Register: MSLUT7Register(),
      #MSLUTSelectRegister: MSLUTSelectRegister(),
      #MicrostepCountRegister: MicrostepCountRegister(),
      #StartSineRegister: StartSineRegister(),
      #GearRatioRegister: GearRatioRegister(0x01000000)
      VirtualStopLeftRegister: VirtualStopLeftRegister(),
      VirtualStopRightRegister: VirtualStopRightRegister()
    }

  def flush_registers(self):
    for register in self._registers.values():
      self._spi.write(register)

  def load_registers(self):
    for register_class, register in self._registers.items():
      self._registers[register_class] = self._spi.read(register)

  def load_register(self, register_key):
    register = self._spi.read(self._registers[register_key])
    self._registers[register_key] = register
    return register

  def load_register_value(self, register_key, value_key):
    return self.load_register(register_key).get(value_key)

  def load_registers_from_ini(self, path):
    for register_code, register_value in parse_ini(path):
      # Look up register
      for register_class, register in self._registers.items():
        if register_class.REGISTER == register_code:
          self._registers[register_class] = register_class(register_value)

  #
  # Driver API methods
  #

  def activate(self):
    """
    "Per default, the voltage level transition from high to low triggers a start
    signal (START is an input), or START output indicates an active START event
    by switching from high to low level." Datasheet 9.1.3

    Implements Driver.start() method.
    """
    GPIO.setmode(self._pin_mode)
    if self._reset_pin:
      GPIO.setup(self._reset_pin, GPIO.OUT)
    self.reset(hard=True)
    # We need to write our current register values to the TMC4361 since we just
    # wiped them all out on the chip due to reset.
    self.flush_registers()
    # Enable the slave TCM26x
    self.enable_tmc26x()
    # Reload the current state of the TCM4361
    self.load_registers()

  def shutdown(self):
    """
    Implements Driver.stop() method.
    """
    self.disable_tmc26x()
    self.reset(hard=True)

  def enable(self):
    """
    Implements Driver.enable() method.
    """
    # TODO implement
    pass

  def disable(self):
    """
    Implements Driver.disable() method.
    """
    # TODO implement
    pass

  def step(self, direction=None):
    """
    Implements Driver.step(direction) method.
    Implements Profile.step() method.
    """
    pass

  def set_microsteps(self, microsteps):
    """
    Implements Driver.set_microsteps(microsteps) method.
    Implements Profile.set_microsteps(microsteps) method.
    """
    value = MICROSTEP_RESOLUTION[microsteps]
    log.debug('Setting microsteps to: %s', microsteps)
    self._spi.write(self._registers[MotorDriverSettingsRegister]
      .set(MotorDriverSettingsRegister.bits.MSTEP_PER_FS, value) # full stepping
    )
    # Set microsteps on tmc26x
    self.tmc26x.set_microsteps(microsteps)

  @property
  def microsteps(self):
    value = self.load_register_value(MotorDriverSettingsRegister, MotorDriverSettingsRegister.bits.MSTEP_PER_FS)
    # TODO add a reverse lookup method to MICROSTEP_RESOLUTION
    # reverse lookup to get microsteps
    return [k for k,v in MICROSTEP_RESOLUTION.items() if v == value][0]

  @property
  def pulse_width(self):
    """
    Implements Driver.pulse_width method.
    """
    return 0

  def set_pulse_width(self, pulse_width_us):
    """
    Implements Driver.set_pulse_width(pulse_width_us) method.
    """
    pass

  #
  # Profile API methods
  #

  def set_target_speed(self, speed):
    """
    Implements Profile.set_target_speed(speed) method.
    """
    self._last_target_speed = speed
    self._spi.write(self._registers[VMaxRegister].set(VMaxRegister.bits.VMAX, speed))

  def set_target_steps(self, absolute_steps):
    """
    Implements Profile.set_target_steps(absolute_steps) method.
    Acts like StepperController.move_to(position) method
    """
    # HACK restore VMAX
    vmax = self.load_register_value(VMaxRegister, VMaxRegister.bits.VMAX)
    if not vmax:
      log.debug('VMax currently %s. Restoring to %s', vmax, self._last_target_speed)
      self._spi.write(self._registers[VMaxRegister].set(VMaxRegister.bits.VMAX, self._last_target_speed))
    log.debug('Setting absolute target steps to %s', absolute_steps)
    self._spi.write(self._registers[XTargetRegister].set(XTargetRegister.bits.XTARGET, absolute_steps))

  def set_target_acceleration(self, acceleration):
    """
    Implements Profile.set_target_acceleration(acceleration) method.

    acceleration: (int) steps / sec / sec
    """
    self._spi.write(self._registers[AMaxRegister].set(AMaxRegister.bits.AMAX, acceleration))
    self._spi.write(self._registers[DMaxRegister].set(DMaxRegister.bits.DMAX, acceleration))

  def set_full_steps_per_rev(self, full_steps_per_rev):
    """
    Implements Profile.set_motor_steps_per_rev(steps_per_rev) method.
    """
    self._spi.write(self._registers[MotorDriverSettingsRegister]
      .set(MotorDriverSettingsRegister.bits.FS_PER_REV, full_steps_per_rev) )

  @property
  def full_steps_per_rev(self):
    return self.load_register_value(MotorDriverSettingsRegister, MotorDriverSettingsRegister.bits.FS_PER_REV)

  def set_current_steps(self, position):
    """
    Implements Profile.set_current_position(position) method.
    """
    self._spi.write(self._registers[XActualRegister].set(XActualRegister.bits.XACTUAL, position))

  @property
  def current_acceleration(self):
    return self.load_register_value(AActualRegister, AActualRegister.bits.AACTUAL)

  @property
  def steps_to_go(self):
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
    current_speed = self.load_register_value(VActualRegister, VActualRegister.bits.VACTUAL)
    if current_speed < 0:
      return steppyr.DIRECTION_CCW
    elif current_speed > 0:
      return steppyr.DIRECTION_CW
    else:
      return steppyr.DIRECTION_NONE

  @property
  def current_steps(self):
    return self.load_register_value(XActualRegister, XActualRegister.bits.XACTUAL)

  def compute_new_speed(self):
    pass

  def should_step(self):
    if REPORT:
      self.report()
    return self.is_moving

  #
  # StepperController API methods
  #

  def stop(self):
    """
    Implements StepperController.stop() method.
    """

    current_position = self.load_register_value(XActualRegister, XActualRegister.bits.XACTUAL)

    # Enable virtual stops
    log.debug('Enabling virtual stop switches')
    self._spi.write(self._registers[ReferenceConfRegister].set(ReferenceConfRegister.bits.VIRTUAL_LEFT_LIMIT_EN, 1))
    self._spi.write(self._registers[ReferenceConfRegister].set(ReferenceConfRegister.bits.VIRTUAL_RIGHT_LIMIT_EN, 1))
    self._spi.write(self._registers[ReferenceConfRegister].set(ReferenceConfRegister.bits.VIRT_STOP_MODE, 1))
    # Trigger virtual stop
    log.debug('Stopping by triggering virtual stops at current_position %s', current_position)
    self._spi.write(self._registers[VirtualStopLeftRegister].set(VirtualStopLeftRegister.bits.VIRT_STOP_LEFT, current_position))
    self._spi.write(self._registers[VirtualStopRightRegister].set(VirtualStopRightRegister.bits.VIRT_STOP_RIGHT, current_position))
    # Reset virtual stops
    log.debug('Disabling virtual stop switches')
    self._spi.write(self._registers[ReferenceConfRegister].set(ReferenceConfRegister.bits.VIRTUAL_LEFT_LIMIT_EN, 0))
    self._spi.write(self._registers[ReferenceConfRegister].set(ReferenceConfRegister.bits.VIRTUAL_RIGHT_LIMIT_EN, 0))
    self._spi.write(self._registers[VirtualStopLeftRegister].set(VirtualStopLeftRegister.bits.VIRT_STOP_LEFT, 0))
    self._spi.write(self._registers[VirtualStopRightRegister].set(VirtualStopRightRegister.bits.VIRT_STOP_RIGHT, 0))
    # Read events register to reset stopped flag
    self.get_status_events()

    # Docs say to set VMAX=0, but then we would always have to reset speed when we want to move.
    # self._last_target_speed = self._registers[VMaxRegister].get(VMaxRegister.bits.VMAX)
    self._spi.write(self._registers[VMaxRegister].set(VMaxRegister.bits.VMAX, 0))
    # self._spi.write(self._registers[VActualRegister].set(VActualRegister.bits.VACTUAL, 0))
    # Immediately switching back to original VMAX causes motor to just keep running
    # self._spi.write(self._registers[VMaxRegister].set(VMaxRegister.bits.VMAX, self._last_target_speed))
    # self._spi.write(self._registers[VActualRegister].set(VActualRegister.bits.VACTUAL, 0))
    # Setting target position = current position.
    # Warning: that means our position attribute is not reliable!
    target_position = self._registers[XTargetRegister].get(XTargetRegister.bits.XTARGET)
    self._spi.write(self._registers[XActualRegister].set(XActualRegister.bits.XACTUAL, target_position))
    # self._spi.write(self._registers[XTargetRegister].set(XTargetRegister.bits.XTARGET, current_position))

    # Stop ramp immediately
    # current_speed = self.load_register_value(VActualRegister, VActualRegister.bits.VACTUAL)
    # log.debug('Setting VStopRegister to current_speed %s', current_speed)
    # self._spi.write(self._registers[VStopRegister].set(VStopRegister.bits.VSTOP, current_speed))

  # TODO const char homeMotorTMC4361

  #
  # Non-API methods
  #

  @property
  def target_acceleration(self):
    return self.load_register_value(AMaxRegister, AMaxRegister.bits.AMAX)

  @property
  def target_deceleration(self):
    return self.load_register_value(DMaxRegister, DMaxRegister.bits.DMAX)

  def set_ramp_scurve(self, target_speed, target_acceleration, target_deceleration,
      bow1, bow2, bow3, bow4,
      a_start=0, d_final=0,
      motion_profile=2, operation_mode=1, # fixed values
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
    self._last_target_speed = target_speed
    self._registers[RampModeRegister]\
      .set(RampModeRegister.bits.MOTION_PROFILE, motion_profile)\
      .set(RampModeRegister.bits.OPERATION_MODE, operation_mode)
    self._registers[Bow1Register].set(Bow1Register.bits.BOW1, bow1)
    self._registers[Bow2Register].set(Bow2Register.bits.BOW2, bow2)
    self._registers[Bow3Register].set(Bow3Register.bits.BOW3, bow3)
    self._registers[Bow4Register].set(Bow4Register.bits.BOW4, bow4)
    self._registers[VMaxRegister].set(VMaxRegister.bits.VMAX, target_speed)
    self._registers[AMaxRegister].set(AMaxRegister.bits.AMAX, target_acceleration)
    self._registers[DMaxRegister].set(DMaxRegister.bits.DMAX, target_deceleration)
    self._registers[VStartRegister].set(VStartRegister.bits.VSTART, v_start)
    self._registers[VStopRegister].set(VStopRegister.bits.VSTOP, v_stop)
    self._registers[AStartRegister].set(AStartRegister.bits.ASTART, a_start)
    self._registers[DFinalRegister].set(DFinalRegister.bits.DFINAL, d_final)
    self.flush_registers()

  def set_ramp_none(self, target_speed=None):
    """
    Motion profile No Ramp follows VMAX only (rectangular velocity shape).

    If you do not provide target_speed parameters, you need to set this later.

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
    self._last_target_speed = target_speed
    self._registers[RampModeRegister]\
      .set(RampModeRegister.bits.MOTION_PROFILE, 0)\
      .set(RampModeRegister.bits.OPERATION_MODE, 1)
    if target_speed:
      self._registers[VMaxRegister].set(VMaxRegister.bits.VMAX, target_speed)
    self.flush_registers()

  def set_ramp_trapezoid(self, target_speed=None, target_acceleration=None, target_deceleration=None,
      motion_profile=1, operation_mode=1, v_break=0, # fixed values
      v_start=0, v_stop=0):
    """
    Consideration of acceleration and deceleration values
    without adaptation of these acceleration values.

    If you do not provide target_speed, target_acceleration, target_deceleration
    parameters, you need to set these later.

    Action:
      - Set RAMPMODE(1:0) =b’01 (register 0x20).
      - Set VBREAK =0 (register 0x27).
      - Set proper AMAX register 0x28 and DMAX register 0x29.
      - Set proper VMAX register 0x24.
      - (Optional) Set proper VSTART > 0 (register 0x25).
      - (Optional) Set proper VSTOP > 0 (register 0x26).
    Result:
      The internal velocity VACTUAL is changed successively to VMAX with a linear ramp.
      Only AMAX and DMAX define the acceleration/deceleration slopes.

    AMAX determines the rising slope from absolute low to absolute high velocities,
    whereas DMAX determines the falling slope from absolute high to absolute low
    velocities.
    Acceleration slope and deceleration slopes have only one acceleration and
    deceleration value each.
    """
    self._last_target_speed = target_speed
    self._registers[RampModeRegister]\
      .set(RampModeRegister.bits.MOTION_PROFILE, motion_profile)\
      .set(RampModeRegister.bits.OPERATION_MODE, operation_mode)
    self._registers[VBreakRegister].set(VBreakRegister.bits.VBREAK, v_break)
    self._registers[VStartRegister].set(VStartRegister.bits.VSTART, v_start)
    self._registers[VStopRegister].set(VStopRegister.bits.VSTOP, v_stop)
    if target_speed:
      self._registers[VMaxRegister].set(VMaxRegister.bits.VMAX, target_speed)
    if target_acceleration:
      self._registers[AMaxRegister].set(AMaxRegister.bits.AMAX, target_acceleration)
    if target_deceleration:
      self._registers[DMaxRegister].set(DMaxRegister.bits.DMAX, target_deceleration)
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

  @property
  def current_speed(self):
    return self.load_register_value(VActualRegister, VActualRegister.bits.VACTUAL)

  @property
  def target_steps(self):
    return self.load_register_value(XTargetRegister, XTargetRegister.bits.XTARGET)

  @property
  def target_speed(self):
    return self.load_register_value(VMaxRegister, VMaxRegister.bits.VMAX)

  def get_status_flags(self):
    return self.load_register(StatusFlagRegister).get_values()

  def get_status_events(self):
    """
    Read and parse the Status Event Register (EVENTS 0x0E).
    """
    return self.load_register(StatusEventRegister).get_values()

  # TODO Update this unmainted code
  '''
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
  '''

  # TODO Update this unmainted code
  '''
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
  '''

  # TODO Update this unmainted code
  '''
  def getClearedEndStopConfig(self):
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
  '''

  def reset(self, hard=False):
    if hard and self._reset_pin:
      GPIO.output(self._reset_pin, GPIO.LOW)
      sleep_microseconds(10)
      GPIO.output(self._reset_pin, GPIO.HIGH)
    else:
      # FIXME soft reset doesnt seem to work
      self._spi.write(ResetClockAndGatingRegister()
        .set(ResetClockAndGatingRegister.bits.RESET_REG, 0x525354) )

  def report(self):
    """
    Log a status report at DEBUG level.
    """
    log.debug('TMC4361Driver Status Report')
    log.debug('    current_speed %s', self.current_speed)
    log.debug('    target_position %s', self.target_steps)
    log.debug('    current_position %s', self.current_steps)
    log.debug('    target_speed %s', self.target_speed)
    log.debug('    current_acceleration %s', self.current_acceleration)
    log.debug('    target_acceleration %s', self.target_acceleration)
    log.debug('    target_deceleration %s', self.target_deceleration)
    log.debug('    microsteps %s', self.microsteps)
    log.debug('    full_steps_per_rev %s', self.full_steps_per_rev)
    # Probably not a good idea to constantly read this since it is cleared on read.
    # log.debug('    get_status_events', self.get_status_events())
    log.debug('    get_status_flags %s', self.get_status_flags())
    for register_class, register in self._registers.items():
      log.debug('    %s', register_class.__name__)
      if not hasattr(register, 'bits'):
        continue
      for name, representation in register.bits.items():
        value = register.get(representation)
        if value:
          log.debug('        %s %s', name, value)
    log.debug('TMC26xDriver Status Report')
    for register_class, register in self.tmc26x._registers.items():
      log.debug('    %s', register_class.__name__)
      for name, representation in register.bits.items():
        value = register.get(representation)
        if value:
          log.debug('        %s %s', name, value)

  def enable_tmc26x(self):
    """
    Enable special support for TCM26x drivers.
    See section 10.6 in the TMC4361 datasheet.
    """
    # Configure SPI Output Conf Register to talk to TMC26x
    # TMC_260_CONFIG 0x8440000a //SPI-Out: block/low/high_time=8/4/4 Takte; CoverLength=autom; TMC26x
    self._spi.write(self._registers[SPIOutConfRegister]
      .set(SPIOutConfRegister.bits.SPI_OUTPUT_FORMAT, 10)
      .set(SPIOutConfRegister.bits.COVER_DATA_LENGTH, 0)
      .set(SPIOutConfRegister.bits.AUTOREPEAT_COVER_EN)
      .set(SPIOutConfRegister.bits.SCALE_VAL_TRANSFER_EN, 1)
      .set(SPIOutConfRegister.bits.SPI_OUT_LOW_TIME, 4)
      .set(SPIOutConfRegister.bits.SPI_OUT_HIGH_TIME, 4)
      .set(SPIOutConfRegister.bits.SPI_OUT_BLOCK_TIME, 8) )
    # Initialize TMC26x
    self.tmc26x.activate()
    self.tmc26x.set_stepdir_off(1)

  def disable_tmc26x(self):
    if self.tmc26x:
      self.tmc26x.shutdown()

  def transfer_to_tmc2660(self, datagram):
    """
    Datagram should be 20 bits.
    """
    self._spi.write(CoverLowRegister(datagram))
    resp = self._spi.read(CoverDriverLowRegister())
    return resp
