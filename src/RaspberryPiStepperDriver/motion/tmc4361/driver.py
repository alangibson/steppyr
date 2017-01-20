import RPi.GPIO as GPIO
from RaspberryPiStepperDriver import sleep_microseconds
from . import registers

# TODO different from standard values
DIRECTION_CW = 1
DIRECTION_CCW = -1

# (const unsigned long) default_4361_start_config
# _BV(0) x_target requires start
# _BV(4) use shaddow motion profiles
#  _BV(5) external start is an start
default_4361_start_config = 0 | _BV(0) | _BV(4) | _BV(5)

class TMC4361:

  def __init__(self, spi, start_signal_pin, target_reached_interrupt_pin):
    self._spi = spi
    self._start_signal_pin = start_signal_pin
    self._target_reached_interrupt_pin = target_reached_interrupt_pin

  def prepareTMC4361(self):
    GPIO.setup(self._start_signal_pin, GPIO.IN)
    GPIO.output(self._start_signal_pin, GPIO.LOW)

    # will be released after setup is complete
    GPIO.setup(self._target_reached_interrupt_pin, GPIO.INPUT)
    GPIO.output(self._target_reached_interrupt_pin, GPIO.LOW)
    # _BV(5) external start is an start (to ensure start is an input)
    self._spi.writeRegister(registers.TMC4361_START_CONFIG_REGISTER, 0 | _BV(5))

    GPIO.output(self._start_signal_pin, GPIO.HIGH)
    GPIO.setup(self._start_signal_pin, GPIO.OUT)

    # TODO void initialzeTMC4361() {

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
    long last_pos = self.last_target # this was our last position
    next_direction =  DIRECTION_CW if target_pos > last_pos else DIRECTION_CCW # and for failsafe movement we need to write down the direction

    # long aim_target
    if isWaypoint:
      aim_target = target_pos + (target_pos - last_pos) # 2*(target_pos-last_pos)+last_pos
    else:
      aim_target = target_pos

    # long fixed_a_max
    fixed_a_max = FIXED_22_2_MAKE(aMax)

    # only change parameters if the target position is different than the last time
    if aim_target != next_targets:
      self._spi.writeRegister(registers.TMC4361_SH_RAMP_MODE_REGISTER, _BV(2) | 1) # trapezoidal positioning
      self._spi.writeRegister(registers.TMC4361_SH_V_MAX_REGISTER, FIXED_23_8_MAKE(vMax)) # set the velocity
      self._spi.writeRegister(registers.TMC4361_SH_A_MAX_REGISTER, fixed_a_max) # set maximum acceleration
      self._spi.writeRegister(registers.TMC4361_SH_D_MAX_REGISTER, fixed_a_max) # set maximum deceleration
      # self._spi.writeRegister(registers.TMC4361_SH_V_START_REGISTER,FIXED_23_8_MAKE(vMax/2)); //set start velocity
      # self._spi.writeRegister(registers.TMC4361_SH_V_STOP_REGISTER,FIXED_23_8_MAKE(vMax/2)); //set stop velocity
      self._spi.writeRegister(registers.TMC4361_SH_V_START_REGISTER,FIXED_23_8_MAKE(0)) # set start velocity
      self._spi.writeRegister(registers.TMC4361_SH_V_STOP_REGISTER,FIXED_23_8_MAKE(0)) # set stop velocity
      self._spi.writeRegister(registers.TMC4361_SH_V_START_REGISTER,FIXED_23_8_MAKE(vStart)) # set start velocity
      self._spi.writeRegister(registers.TMC4361_SH_V_STOP_REGISTER,FIXED_23_8_MAKE(vStop)); # set stop velocity
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
