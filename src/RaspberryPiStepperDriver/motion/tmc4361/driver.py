import RPi.GPIO as GPIO
from RaspberryPiStepperDriver import sleep_microseconds, tobin, set_bit, unset_bit
from . import registers, _BV

# TODO different from standard values
DIRECTION_CW = 1
DIRECTION_CCW = -1

CLOCK_FREQUENCY = 16000000

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
    self._spi.writeRegister(registers.TMC4361_START_CONFIG_REGISTER, 0 | _BV(5))

    GPIO.output(self._start_signal_pin, GPIO.HIGH)
    # GPIO.setup(self._start_signal_pin, GPIO.IN)
    sleep_microseconds(1)
    GPIO.output(self._start_signal_pin, GPIO.LOW)

    self._is_homing = False
    # TODO digitalWriteFast(START_SIGNAL_PIN,HIGH);

    # will be released after setup is complete
    # TODO digitalWriteFast(motors[i].target_reached_interrupt_pin,LOW);
    # _BV(5) //external start is an start (to ensure start is an input)
    self._spi.writeRegister(registers.TMC4361_START_CONFIG_REGISTER, 0 | _BV(5))

    # preconfigure the TMC4361

    # see datasheet 19.13-19.14
    self._spi.writeRegister(registers.TMC4361_SPI_STATUS_SELECTION_REGISTER, 0 | _BV(0) | _BV(1))

    # long g_conf
    g_conf = 0 | _BV(5)
    # TODO if differential_encoder_motors ...
    #if differential_encoder_motors & _BV(i) {
    #  g_conf |= _BV(12) | 0x2000000ul; // we don't use direct values and we forward the clock & diff encoder
    # we don't use direct values
    self._spi.writeRegister(registers.TMC4361_GENERAL_CONFIG_REGISTER, g_conf)
    # trapezoidal positioning
    self._spi.writeRegister(registers.TMC4361_RAMP_MODE_REGISTER, _BV(2) | 1)
    # trapezoidal positioning
    self._spi.writeRegister(registers.TMC4361_SH_RAMP_MODE_REGISTER, _BV(2) | 1)
    # self._spi.writeRegister(registers.TMC4361_RAMP_MODE_REGISTER,_BV(2) | 2); //we want to go to positions in nice S-Ramps)
    # self._spi.writeRegister(registers.TMC4361_SH_RAMP_MODE_REGISTER,_BV(2) | 2); //we want to go to positions in nice S-Ramps)
    self._spi.writeRegister(registers.TMC4361_CLK_FREQ_REGISTER, CLOCK_FREQUENCY)
    # NEEDED so THAT THE SQUIRREL CAN RECOMPUTE EVERYTHING!
    self._spi.writeRegister(registers.TMC4361_START_DELAY_REGISTER, 512)
    # TODO shouldn't we add target_reached - just for good measure??
    self.setStepsPerRevolution(self._steps_per_revolution)
    self._spi.writeRegister(registers.TMC4361_START_CONFIG_REGISTER, default_4361_start_config);
    # filter start (unsigned long filter)
    # 0x20000 = 2 << 16, 0x400000 = 4 << 20
    filter = 0x20000 | 0x400000
    # filter ref
    filter |= (2<<8) | 0x4000
    self._spi.writeRegister(registers.TMC4361_INPUT_FILTER_REGISTER, filter)

    # debugging?
    self._spi.readRegister(registers.TMC4361_EVENTS_REGISTER)

    # last_target[i]=0;

  def setStepsPerRevolution(self, steps):
    """
    Arguments:
    (unsigned int) steps

    Returns: (const char)
    """
    # Configure the motor type
    # FIXME dont asume 256 microsteps
    # unsigned long motorconfig
    motorconfig = 0x00 # we want 256 microsteps
    motorconfig |= steps << 4
    self._spi.writeRegister(registers.TMC4361_STEP_CONF_REGISTER, motorconfig)
    return 0

  # TODO const char homeMotorTMC4361

  def getMotorPosition(self):
    """
    Returns: (long)
    """
    # TODO do we have to take into account that the motor may never have reached the x_target??
    # vactual!=0 -> x_target, x_pos sonst or similar
    return self._spi.readRegister(registers.TMC4361_X_TARGET_REGISTER)

  def moveMotor(self, target_pos, vMax, aMax, vStart, vStop, isWaypoint):
    """
    Arguments:
    long target_pos
    double vMax
    double aMax
    double vStart
    double vStop
    boolean isWaypoint
    """
    # calculate the value for x_target so taht we go over pos_comp
    # long last_pos
    # this was our last position
    last_pos = self.last_target
    # and for failsafe movement we need to write down the direction
    next_direction =  DIRECTION_CW if target_pos > last_pos else DIRECTION_CCW

    # long aim_target
    if isWaypoint:
      aim_target = target_pos + (target_pos - last_pos) # 2*(target_pos-last_pos)+last_pos
    else:
      aim_target = target_pos

    # long fixed_a_max
    fixed_a_max = FIXED_22_2_MAKE(aMax)

    # only change parameters if the target position is different than the last time
    if aim_target != next_targets:
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
      # pos comp is not shaddowed
      next_pos_comp = target_pos
      # self._spi.writeRegister(registers.TMC4361_X_TARGET_REGISTER,aim_target);
      next_targets = aim_target
      is_new_position = True
      last_target = target_pos
    else:
      is_new_position = False

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

  def setMotorPosition(self, position):
    """
    Arguments:
    long position
    """
    # unsigned long oldStartRegister
    oldStartRegister = self._spi.readRegister(registers.TMC4361_START_CONFIG_REGISTER)
    # we want an immediate start
    self._spi.writeRegister(registers.TMC4361_START_CONFIG_REGISTER, _BV(5)) # keep START as an input

    # we write x_actual, x_target and pos_comp to the same value to be safe
    self._spi.writeRegister(registers.TMC4361_V_MAX_REGISTER, 0)
    self._spi.writeRegister(registers.TMC4361_X_TARGET_REGISTER, position)
    self._spi.writeRegister(registers.TMC4361_X_ACTUAL_REGISTER, position)
    self._spi.writeRegister(registers.TMC4361_POSITION_COMPARE_REGISTER, position)
    last_target = position

    # back to normal
    self._spi.writeRegister(registers.TMC4361_START_CONFIG_REGISTER, oldStartRegister)

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
