"""
registers for TMC4361
"""
from RaspberryPiStepperDriver import set_bit, unset_bit, lsb
from RaspberryPiStepperDriver.motion.tmc4361 import _BV
from RaspberryPiStepperDriver.motion.tmc4361.io import Datagram

WRITE_MASK = 0x80 # register | WRITE_MASK
READ_MASK = 0x7F # register & READ_MASK

# General Configuration Register
TMC4361_GENERAL_CONFIG_REGISTER = 0x0

# Reference Switch Configuration Register
TMC4361_REFERENCE_CONFIG_REGISTER = 0x01

# RW. Start Switch Configuration Register
# Bit 4:0. start_en
#   xxxx1 Alteration of XTARGET value requires distinct start signal.
#   xxx1x Alteration of VMAX value requires distinct start signal.
#   xx1xx Alteration of RAMPMODE value requires distinct start signal.
#   x1xxx Alteration of GEAR_RATIO value requires distinct start signal.
#   1xxxx Shadow Register Feature Set is enabled.
# Bit 8:5. trigger_events
#   0000 Timing feature set is disabled because start signal generation is disabled.
#   xxx0 START pin is assigned as output.
#   xxx1 External start signal is enabled as timer trigger. START pin is assigned as input.
#   xx1x TARGET_REACHED event is assigned as start signal trigger.
#   x1xx VELOCITY_REACHED event is assigned as start signal trigger.
#   1xxx POSCOMP_REACHED event is assigned as start signal trigger
# Bit 9. pol_start_signal
#   0 START pin is low active (input resp. output).
#   1 START pin is high active (input resp. output).
# Bit 10. immediate_start_in
#   0 Active START input signal starts internal start timer.
#   1 Active START input signal is executed immediately.
# Bit 11. busy_state_en
#   0 START pin is only assigned as input or output.
#   1 Busy start state is enabled. START pin is assigned as input with a weakly driven active start polarity or as output with a strongly driven inactive start polarity.
# Bit 15:12. pipeline_en
#   0000 No pipelining is active.
#   xxx1 X_TARGET is considered for pipelining.
#   xx1x POS_COMP is considered for pipelining.
#   x1xx GEAR_RATIO is considered for pipelining.
#   1xxx GENERAL_CONF is considered for pipelining.
# Bit 17:16. shadow_option
#   0 Single-level shadow registers for 13 relevant ramp parameters.
#   1 Double-stage shadow registers for S-shaped ramps.
#   2 Double-stage shadow registers for trapezoidal ramps (excl. VSTOP).
#   3 Double-stage shadow registers for trapezoidal ramps (excl. VSTART).
# Bit 18. cyclic_shadow_regs
#   0 Current ramp parameters are not written back to the shadow register.
#   1 Current ramp parameters are written back to the appropriate shadow register.
# Bit 19: Reserved. Set to 0.
# Bit 23:20. SHADOW_MISS_CNT
#   U Number of unused start internal start signals between two consecutive shadow register transfers.
# Bit 31:24. XPIPE_REWRITE_REG
#   Current assigned pipeline registers – START_CONF(15:12) – are written back to
#     X_PIPEx in the case of an internal start signal generation and if assigned in this
#     register with a ‘1’:
TMC4361_START_CONFIG_REGISTER = 0x2

TMC4361_INPUT_FILTER_REGISTER = 0x3

# SPI Output Configuration Register
# RW. Bit 3:0. spi_output_format
#   0 SPI output interface is off.
#   1 SPI output interface is connected with a SPI-DAC. SPI output values are mapped to full amplitude:
#     Current=0  VCC/2
#     Current=-max  0
#     Current=max  VCC
#   2 SPI output interface is connected with a SPI-DAC. SPI output values are absolute
#     values. Phase of coilA is forwarded via STPOUT, whereas phase of coilB is
#     forwarded via DIROUT. Phase bit = 0:positive value.
#   3 SPI output interface is connected with a SPI-DAC. SPI output values are absolute
#     values. Phase of coilA is forwarded via STPOUT, whereas phase of coilB is
#     forwarded via DIROUT. Phase bit = 0: negative value.
#   4 The actual unsigned scaling factor is forwarded via SPI output interface.
#   5 Both actual signed current values CURRENTA and CURRENTB are forwarded in
#     one datagram via SPI output interface.
#   6 SPI output interface is connected with a SPI-DAC. The actual unsigned scaling
#     factor is merged with DAC_ADDR_A value to an output datagram.
#   8 SPI output interface is connected with a TMC23x stepper motor driver.
#   9 SPI output interface is connected with a TMC24x stepper motor driver.
#  10 SPI output interface is connected with a TMC26x/389 stepper motor driver.
#     Configuration and current data are transferred to the stepper motor driver.
#  11 SPI output interface is connected with a TMC26x stepper motor driver. Only
#     configuration data is transferred to the stepper motor driver. S/D output interface
#     provides steps.
#  12 SPI output interface is connected with a TMC2130 stepper motor driver. Only
#     configuration data is transferred to the stepper motor driver. S/D output interface
#     provides steps.
#  13 SPI output interface is connected with a TMC2130 stepper motor driver.
#     Configuration and current data are transferred to the stepper motor driver.
#  15 Only cover datagrams are transferred via SPI output interface.
# Bit 19:13. COVER_DATA_LENGTH
#   U Number of bits for the complete datagram length. Maximum value = 64
#     Set to 0 in case a TMC stepper motor driver is selected. The datagram length is then
#     selected automatically.
# Bits 23:20. SPI_OUT_LOW_TIME
#   U Number of clock cycles the SPI output clock remains at low level.
# Bits 27:24. SPI_OUT_HIGH_TIME
#   U Number of clock cycles the SPI output clock remains at high level
# Bits 31:28. SPI_OUT_BLOCK_TIME
#   U Number of clock cycles the NSCSDRV output remains high (inactive) after a SPI
#     output transmission
TMC4361_SPIOUT_CONF_REGISTER = 0x04

TMC4361_ENCODER_INPUT_CONFIG_REGISTER = 0x07
TMC4361_STEP_CONF_REGISTER = 0x0A

# Event Selection Registers
# RW. Bit 31:0. Events selection for SPI datagrams: Event bits of EVENTS
#   register 0x0E that are selected (=1) in this register are forwarded to the
#   eight status bits that are transferred with every SPI datagram (first eight
#   bits from LSB are significant!).
TMC4361_SPI_STATUS_SELECTION_REGISTER = 0x0B
TMC4361_EVENT_CLEAR_CONF_REGISTER = 0x0c
TMC4361_INTERRUPT_CONFIG_REGISTER = 0x0d

# Status Event Register
# 0 TARGET_REACHED has been triggered.
# 1 POS_COMP_REACHED has been triggered.
# 2 VEL_REACHED has been triggered.
# 3 VEL_STATE = b’00 has been triggered (VACTUAL = 0).
# 4 VEL_STATE = b’01 has been triggered (VACTUAL > 0).
# 5 VEL_STATE = b’10 has been triggered (VACTUAL < 0).
# 6 RAMP_STATE = b’00 has been triggered (AACTUAL = 0, VACTUAL is constant).
# 7 RAMP_STATE = b’01 has been triggered (|VACTUAL| increases).
# 8 RAMP_STATE = b’10 has been triggered (|VACTUAL| increases).
# 9 MAX_PHASE_TRAP: Trapezoidal ramp has reached its limit speed using maximum values for AMAX or DMAX (|VACTUAL| > VBREAK; VBREAK≠0).
# 10 FROZEN: NFREEZE has switched to low level. i Reset TMC4361A for further motion.
# 11 STOPL has been triggered. Motion in negative direction is not executed until this event is cleared and (STOPL is not active any more or stop_left_en is set to 0).
# 12 STOPR has been triggered. Motion in positive direction is not executed until this event is cleared and (STOPR is not active any more or stop_right_en is set to 0).
# 13 VSTOPL_ACTIVE: VSTOPL has been activated. No further motion in negative direction until this event is cleared and (a new value is chosen for VSTOPL or virtual_left_limit_en is set to 0).
# 14 VSTOPR_ACTIVE: VSTOPR has been activated. No further motion in positive direction until this event is cleared and (a new value is chosen for VSTOPR or virtual_right_limit_en is set to 0).
# 15 HOME_ERROR: Unmatched HOME_REF polarity and HOME is outside of safety margin.
# 16 XLATCH_DONE indicates if X_LATCH was rewritten or homing process has been completed.
# 17 FS_ACTIVE: Fullstep motion has been activated.
# 18 ENC_FAIL: Mismatch between XACTUAL and ENC_POS has exceeded specified limit.
# 19 N_ACTIVE: N event has been activated.
# 20 ENC_DONE indicates if ENC_LATCH was rewritten.
# 21 SER_ENC_DATA_FAIL: Failure during multi-cycle data evaluation or between two consecutive data requests has occured.
# 22 Reserved.
# 23 SER_DATA_DONE: Configuration data was received from serial SPI encoder.
# 24 One of the SERIAL_ENC_Flags was set.
# 25 COVER_DONE: SPI datagram was sent to the motor driver.
# 26 ENC_VEL0: Encoder velocity has reached 0.
# 27 CL_MAX: Closed-loop commutation angle has reached maximum value.
# 28 CL_FIT: Closed-loop deviation has reached inner limit.
# 29 STOP_ON_STALL: Motor stall detected. Motor ramp has stopped.
# 30 MOTOR_EV: One of the selected TMC motor driver flags was triggered.
# 31 RST_EV: Reset was triggered.
TMC4361_EVENTS_REGISTER = 0x0e

# Status Flag Register
# 0 TARGET_REACHED_F is set high if XACTUAL = XTARGET
# 1 POS_COMP_REACHED_F is set high if XACTUAL = POS_COMP
# 2 VEL_REACHED_F is set high if VACTUAL = |VMAX|
# 4:3 VEL_STATE_F: Current velocity state: 0  VACTUAL = 0;
#   1  VACTUAL > 0;
#   2  VACTUAL < 0
# 6:5 RAMP_STATE_F: Current ramp state: 0  AACTUAL = 0;
#   1  AACTUAL increases (acceleration);
#   2  AACTUAL decreases (deceleration)
# 7 STOPL_ACTIVE_F: Left stop switch is active.
# 8 STOPR_ACTIVE_F: Right stop switch is active.
# 9 VSTOPL_ACTIVE_F: Left virtual stop switch is active.
# 10 VSTOPR_ACTIVE_F: Right virtual stop switch is active.
# 11 ACTIVE_STALL_F: Motor stall is detected and VACTUAL > VSTALL_LIMIT.
# 12 HOME_ERROR_F: HOME_REF input signal level is not equal to expected home level.
# 13 FS_ACTIVE_F: Fullstep operation is active.
# 14 ENC_FAIL_F: Mismatch between XACTUAL and ENC_POS is out of tolerated range.
# 15 N_ACTIVE_F: N event is active.
# 16 ENC_LATCH_F: ENC_LATCH is rewritten.
# 17 Applies to absolute encoders only:
#    MULTI_CYCLE_FAIL_F indicates a failure during last multi cycle data evaluation.
#    Applies to absolute encoders only:
#    SER_ENC_VAR_F indicates a failure during last serial data evaluation due to a substantial
#    deviation between two consecutive serial data values.
# 18 Reserved.
# 19 CL_FIT_F: Active if ENC_POS_DEV < CL_TOLERANCE. The current mismatch between XACTUAL and ENC_POS is within tolerated range.
# 23:20 Applies to absolute encoders only: SERIAL_ENC_FLAGS received from encoder. These flags are reset with a new encoder transfer request.
# 24 TMC26x / TMC2130 only: SG: StallGuard2 status
#    Optional for TMC24x only: Calculated stallGuard status.
#    TMC23x / TMC24x only: UV_SF: Undervoltage flag.
# 25 All TMC motor drivers: OT: Overtemperature shutdown.
# 26 All TMC motor drivers: OTPW: Overtemperature warning.
# 27 TMC26x / TMC2130 only: S2GA: Short to ground detection bit for high side MOSFE of coil A.
#    TMC23x / TMC24x only: OCA: Overcurrent bridge A.
# 28 TMC26x / TMC2130 only: S2GB: Short to ground detection bit for high side MOSFET of coil B.
#    TMC23x / TMC24x only: OCB: Overcurrent bridge B.
# 29 All TMC motor drivers: OLA: Open load indicator of coil A.
# 30 All TMC motor drivers: OLB: Open load indicator of coil B.
# 31 TMC26x / TMC2130 only: STST: Standstill indicator.
#    TMC23x / TMC24x only: OCHS: Overcurrent high side.
TMC4361_STATUS_REGISTER = 0x0f

# Various Configuration Registers: S/D, Synchronization, etc.
TMC4361_STP_LENGTH_ADD_REGISTER = 0x10
TMC4361_START_OUT_ADD_REGISTER = 0x11
TMC4361_GEAR_RATIO_REGISTER = 0x12
# RW. Bits 31:0. START_DELAY.
#   Delay time [# clock cycles] between start trigger and internal start signal release.
TMC4361_START_DELAY_REGISTER = 0x13

# Ramp Generator Registers
TMC4361_RAMP_MODE_REGISTER = 0x20
# RW. Current internal microstep position; signed; 32 bits
# RW. Bit 0:31. Actual internal motor position [pulses]: –2^31 ≤ XACTUAL ≤ 2^31 – 1
TMC4361_X_ACTUAL_REGISTER = 0x21
# R. Current step velocity; 24 bits; signed; no decimals.
# Bit 31:0: Actual ramp generator velocity [pulses per second]: 1 pps ≤ |VACTUAL| ≤ CLK_FREQ · 1/2 pulses (fCLK = 16 MHz -> 8 Mpps)
TMC4361_V_ACTUAL_REGISTER = 0x22
# R. Current step acceleration; 24 bits; signed; no decimals.
TMC4361_A_ACTUAL_REGISTER = 0x23
# RW. Maximum permitted or target velocity; signed; 32 bits= 24+8 (24 bits integer part, 8 bits decimal places).
TMC4361_V_MAX_REGISTER = 0x24
# RW. Velocity at ramp start; unsigned; 31 bits=23+8.
TMC4361_V_START_REGISTER = 0x25
# RW. Velocity at ramp end; unsigned; 31 bits=23+8.
TMC4361_V_STOP_REGISTER = 0x26
# RW. At this velocity value, the aceleration/deceleration will change during trapezoidal ramps; unsigned; 31 bits=23+8.
TMC4361_V_BREAK_REGISTER = 0x27
# RW. Maximum permitted or target acceleration; unsigned; 24 bits=22+2 (22 bits integer part, 2 bits decimal places).
TMC4361_A_MAX_REGISTER = 0x28
# RW. Maximum permitted or target deceleration; unsigned; 24 bits=22+2.
TMC4361_D_MAX_REGISTER = 0x29
# RW. First bow value of a complete velocity ramp; unsigned; 24 bits=24+0 (24 bits integer part, no decimal places).
TMC4361_BOW_1_REGISTER = 0x2d
# RW. Second bow value of a complete velocity ramp; unsigned; 24bits=24+0.
TMC4361_BOW_2_REGISTER = 0x2e
# RW. Third bow value of a complete velocity ramp; unsigned; 24 bits=24+0.
TMC4361_BOW_3_REGISTER = 0x2f
# RW. Fourth bow value of a complete velocity ramp; unsigned; 24 bits=24+0.
TMC4361_BOW_4_REGISTER = 0x30

# External Clock Frequency Register
# RW. External clock frequency fCLK; unsigned; 25 bits.
TMC4361_CLK_FREQ_REGISTER = 0x31

# Target and Compare Registers
TMC4361_POSITION_COMPARE_REGISTER = 0x32
TMC4361_VIRTUAL_STOP_LEFT_REGISTER = 0x33
TMC4361_VIRTUAL_STOP_RIGHT_REGISTER = 0x34
TMC4361_X_LATCH_REGISTER = 0x36
# RW. Target position; signed; 32 bits.
TMC4361_X_TARGET_REGISTER = 0x37

TMC4361_X_TARGET_PIPE_0_REGSISTER = 0x38

# Shadow Register
#   Some applications require a complete new ramp parameter set for a specific ramp
#   situation / point in time. TMC4361A provides up to 14 shadow registers, which
#   are loaded into the corresponding ramp parameter registers after an internal
#   start signal is generated.
# RW. Bit 31:0. Val S. SH_REG0 (Default: 0x00000000) : 1st shadow register.
TMC4361_SH_V_MAX_REGISTER = 0x40
# RW. Bit 31:0. Val U. SH_REG1 (Default: 0x00000000) : 2nd shadow register
TMC4361_SH_A_MAX_REGISTER = 0x41
TMC4361_SH_D_MAX_REGISTER = 0x42
TMC4361_SH_VBREAK_REGISTER = 0x45
TMC4361_SH_V_START_REGISTER = 0x46
TMC4361_SH_V_STOP_REGISTER = 0x47
TMC4361_SH_BOW_1_REGISTER = 0x48
TMC4361_SH_BOW_2_REGISTER = 0x49
TMC4361_SH_BOW_3_REGISTER = 0x4A
TMC4361_SH_BOW_4_REGISTER = 0x4B
TMC4361_SH_RAMP_MODE_REGISTER = 0x4C

# Reset and Clock Gating Register
# RW. Bit 2:0. CLK_GATING_REG
#   0 Clock gating is not activated.
#   7 Clock gating is activated.
# Bits 31:8. RESET_REG
#  0 No reset is activated.
#  0x525354 Internal reset is activated.
TMC4361_RESET_CLK_GATING_REGISTER = 0x4F

TMC4361_ENCODER_POSITION_REGISTER = 0x50
TMC4361_ENCODER_INPUT_RESOLUTION_REGISTER = 0x54
TMC4361_COVER_LOW_REGISTER = 0x6C
TMC4361_COVER_HIGH_REGISTER = 0x6D
TMC4361_COVER_DRV_LOW_REGISTER = 0x6E
TMC4361_COVER_DRV_HIGH_REGISTER = 0x6F

# how to mask REFERENCE_CONFIG_REGISTER if you want to configure just one end
TMC4361_LEFT_ENDSTOP_REGISTER_PATTERN = (_BV(0) | _BV(2) | _BV(6) | _BV(10) | _BV(11) | _BV(14))
TMC4361_RIGHT_ENDSTOP_REGISTER_PATTERN = (_BV(1) | _BV(3) | _BV(7) | _BV(12) | _BV(13) | _BV(15))

def bits(n):
  """
  Returns position of all bits = 1.
  http://stackoverflow.com/questions/8898807/pythonic-way-to-iterate-over-bits-of-integer
  """
  while n:
    b = n & (~n+1)
    yield b
    n ^= b

class AttributeDict(dict):
  __getattr__ = dict.__getitem__
  __setattr__ = dict.__setitem__

class Register(Datagram):

  def __init__(self, register_value=0):
    self._header = self.REGISTER
    self._data = register_value

  def set(self, bitmask, value=None):
    if value == None:
      # Just apply the bitmask itself
      self._data = set_bit(self._data, bitmask)
    else:
      # Apply value at bitmask bits
      self._data = set_bit(unset_bit(self._data, bitmask), (value << lsb(bitmask)))
    return self

  def unset(self, bitmask):
    self._data = unset_bit(self._data, bitmask)
    return self

  def get_values(self):
    events = []
    for name, bitmask in self.bits.items():
      if self._data & bitmask:
        events.append((name, bitmask))
    return events

class StatusEventRegister(Register):
  """
  """
  REGISTER = TMC4361_EVENTS_REGISTER
  bits = AttributeDict({
    # TARGET_REACHED has been triggered
    'TARGET_REACHED': _BV(0),
    # POS_COMP_REACHED has been triggered.
    'POS_COMP_REACHED': _BV(1),
    # VEL_REACHED has been triggered.
    'VEL_REACHED': _BV(2),
    # VEL_STATE': b’00 has been triggered (VACTUAL = 0).
    'VACTUAL_EQ_0': _BV(3),
    # VEL_STATE = b’01 has been triggered (VACTUAL > 0).
    'VACTUAL_GT_0': _BV(4),
    # VEL_STATE = b’10 has been triggered (VACTUAL < 0).
    'VACTUAL_LT_0': _BV(5),
    # RAMP_STATE = b’00 has been triggered (AACTUAL = 0, VACTUAL is constant).
    'RAMP_STATE_00': _BV(6),
    # RAMP_STATE = b’01 has been triggered (|VACTUAL| increases).
    'RAMP_STATE_01': _BV(7),
    # RAMP_STATE = b’10 has been triggered (|VACTUAL| increases).
    'RAMP_STATE_10': _BV(8),
    # MAX_PHASE_TRAP: Trapezoidal ramp has reached its limit speed using maximum values for AMAX or DMAX (|VACTUAL| > VBREAK; VBREAK≠0).
    'MAX_PHASE_TRAP': _BV(9),
    # FROZEN: NFREEZE has switched to low level. Reset TMC4361A for further motion.
    'FROZEN': _BV(10),
    # STOPL has been triggered. Motion in negative direction is not executed until this event is cleared and (STOPL is not active any more or stop_left_en is set to 0).
    'STOPL_TRIGGERED': _BV(11),
    # STOPR has been triggered. Motion in positive direction is not executed until this event is cleared and (STOPR is not active any more or stop_right_en is set to 0).
    'STOPR_TRIGGERED': _BV(12),
    # VSTOPL_ACTIVE: VSTOPL has been activated. No further motion in negative direction until this event is cleared and (a new value is chosen for VSTOPL or virtual_left_limit_en is set to 0).
    'VSTOPL_ACTIVE': _BV(13),
    # VSTOPR_ACTIVE: VSTOPR has been activated. No further motion in positive direction until this event is cleared and (a new value is chosen for VSTOPR or virtual_right_limit_en is set to 0).
    'VSTOPR_ACTIVE': _BV(14),
    # HOME_ERROR: Unmatched HOME_REF polarity and HOME is outside of safety margin.
    'HOME_ERROR': _BV(15),
    # XLATCH_DONE indicates if X_LATCH was rewritten or homing process has been completed.
    'XLATCH_DONE': _BV(16),
    # FS_ACTIVE: Fullstep motion has been activated.
    'FS_ACTIVE': _BV(17),
    # ENC_FAIL: Mismatch between XACTUAL and ENC_POS has exceeded specified limit.
    'ENC_FAIL': _BV(18),
    # N_ACTIVE: N event has been activated.
    'N_ACTIVE': _BV(19),
    # ENC_DONE indicates if ENC_LATCH was rewritten.
    'ENC_DONE': _BV(20),
    # SER_ENC_DATA_FAIL: Failure during multi-cycle data evaluation or between two consecutive data requests has occured.
    'SER_ENC_DATA_FAIL': _BV(21),
    # 22: Reserved
    # SER_DATA_DONE: Configuration data was received from serial SPI encoder.
    'SER_DATA_DONE': _BV(23),
    # One of the SERIAL_ENC_Flags was set.
    'SERIAL_ENC_FLAG_SET': _BV(24),
    # COVER_DONE: SPI datagram was sent to the motor driver.
    'COVER_DONE': _BV(25),
    # ENC_VEL0: Encoder velocity has reached 0.
    'ENC_VEL0': _BV(26),
    # CL_MAX: Closed-loop commutation angle has reached maximum value.
    'CL_MAX': _BV(27),
    # CL_FIT: Closed-loop deviation has reached inner limit
    'CL_FIT': _BV(28),
    # STOP_ON_STALL: Motor stall detected. Motor ramp has stopped.
    'STOP_ON_STALL': _BV(29),
    # MOTOR_EV: One of the selected TMC motor driver flags was triggered.
    'MOTOR_EV': _BV(30),
    # RST_EV: Reset was triggered.
    'RST_EV': _BV(31)
  })

class SpiStatusSelectionRegister(Register):
  """
  Represents SPI_STATUS_SELECTION Register (0x0B).

  Events selection for SPI datagrams:
  Event bits of EVENTS register 0x0E that are selected (=1) in this register are
  forwarded to the eight status bits that are transferred with every SPI datagram (first
  eight bits from LSB are significant!).
  """
  REGISTER = TMC4361_SPI_STATUS_SELECTION_REGISTER
  bits = StatusEventRegister.bits

class GeneralConfigurationRegister(Register):
  REGISTER = TMC4361_GENERAL_CONFIG_REGISTER
  bits = AttributeDict({
    # use_astart_and_vstart (only valid for S-shaped ramps)
    # 0 Sets AACTUAL = AMAX or –AMAX at ramp start and in the case of VSTART ≠ 0.
    # 1 Sets AACTUAL = ASTART or –ASTART at ramp start and in the case of VSTART ≠ 0.
    'USE_ASTART_AND_VSTART': _BV(0),
    # 0 Acceleration values are divided by CLK_FREQ.
    # 1 Acceleration values are set directly as steps per clock cycle.
    'DIRECT_ACC_VAL_EN': _BV(1),
    # 0 Bow values are calculated due to division by CLK_FREQ.
    # 1 Bow values are set directly as steps per clock cycle.
    'DIRECT_BOW_VAL_EN': _BV(2),
    # 0 STPOUT = 1 indicates an active step.
    # 1 STPOUT = 0 indicates an active step.
    'STEP_INACTIVE_POL': _BV(3),
    # 0 Only STPOUT transitions from inactive to active polarity indicate steps.
    # 1 Every level change of STPOUT indicates a step.
    'TOGGLE_STEP': _BV(4),
    # 0 DIROUT = 0 indicates negative direction.
    # 1 DIROUT = 1 indicates negative direction.
    'POL_DIR_OUT': _BV(5),
    # 0 Internal step control (internal ramp generator will be used)
    # 1 External step control via STPIN / DIRIN interface with high active steps at STPIN
    # 2 External step control via STPIN / DIRIN interface with low active steps at STPIN
    # 3 External step control via STPIN / DIRIN interface with toggling steps at STPIN
    'SDIN_MODE': _BV(6) | _BV(7),
    # 0 DIRIN = 0 indicates negative direction.
    # 1 DIRIN = 1 indicates negative direction.
    'POL_DIR_IN': _BV(8),
    # 0 STPIN/DIRIN input signals will manipulate internal steps at XACTUAL directly.
    # 1 STPIN/DIRIN input signals will manipulate XTARGET register value, the internal ramp generator is used.
    'SD_INDIRECT_CONTROL': _BV(9),
    # 0 An incremental encoder is connected to encoder interface.
    # 1 An absolute SSI encoder is connected to encoder interface.
    # 2 Reserved
    # 3 An absolute SPI encoder is connected to encoder interface
    'SERIAL_ENC_IN_MODE': _BV(11) | _BV(10),
    # 0 Differential encoder interface inputs enabled.
    # 1 Differential encoder interface inputs is disabled (automatically set for SPI encoder).
    'DIFF_ENC_IN_DISABLE': _BV(12),
    # 0 Standby signal becomes forwarded with an active low level at STDBY_CLK output.
    # 1 Standby signal becomes forwarded with an active high level at STDBY_CLK output.
    # 2 STDBY_CLK passes ChopSync clock (TMC23x, TMC24x stepper motor drivers only).
    # 3 Internal clock is forwarded to STDBY_CLK output pin.
    'STDBY_CLK_PIN_ASSIGNMENT': _BV(14) | _BV(13),
    # 0 INTR=0 indicates an active interrupt.
    # 1 INTR=1 indicates an active interrupt.
    'INTR_POL': _BV(15),
    # 0 TARGET_REACHED signal is set to 1 to indicate a target reached event.
    # 1 TARGET_REACHED signal is set to 0 to indicate a target reached event.
    'INVERT_POL_TARGET_REACHED': _BV(16),
    # 0 Clock gating is disabled.
    # 1 Internal clock gating is enabled.
    'CLK_GATING_EN': _BV(17),
    # 0 No clock gating during standby phase.
    # 1 Intenal clock gating during standby phase is enabled.
    'CLK_GATING_STDBY_EN': _BV(18),
    # 0 Fullstep switchover is disabled.
    # 1 SPI output forwards fullsteps, if |VACTUAL| > FS_VEL
    'FS_EN': _BV(19),
    # 0 No fullstep switchover for Step/Dir output is enabled.
    # 1 Fullsteps are forwarded via Step/Dir output also if fullstep operation is active.
    'FS_SDOUT': _BV(20),
    # 0 dcStep is disabled.
    # 1 dcStep signal generation will be selected automatically
    # 2 dcStep with external STEP_READY signal generation (TMC2130).
    # 3 dcStep with internal STEP_READY signal generation (TMC26x).
    #   TMC26x config: use const_toff-Chopper (CHM = 1);
    #   slow decay only (HSTRRT = 0);
    #   TST = 1 and SGT0=SGT1=1 (on_state_xy).
    'DCSTEP_MODE': _BV(22) | _BV(21),
    # 0 PWM output is disabled. Step/Dir output is enabled at STPOUT/DIROUT.
    # 1 STPOUT/DIROUT output pins are used as PWM output (PWMA/PWMB).
    'PWM_OUT_EN': _BV(23),
    # 0 No encoder is connected to SPI output.
    # 1 SPI output is used as SSI encoder interface to forward absolute SSI encoder data.
    'SERIAL_ENC_OUT_ENABLE': _BV(24),
    # 0 Differential serial encoder output is enabled.
    # 1 Differential serial encoder output is disabled.
    'SERIAL_ENC_OUT_DIFF_DISABLE': _BV(25),
    # 0 VACTUAL=0 & AACTUAL=0 after switching off direct external step control.
    # 1 VACTUAL = VSTART and AACTUAL = ASTART after switching off direct external step control.
    'AUTOMATIC_DIRECT_SDIN_SWITCH_OFF': _BV(26),
    # 0 The register value of X_LATCH is forwarded at register 0x36.
    # 1 The register value of REV_CNT (#internal revolutions) is forwarded at register 0x36.
    'CIRCULAR_CNT_AS_XLATCH': _BV(27),
    # 0 The direction of the internal SinLUT is regularly used.
    # 1 The direction of internal SinLUT is reversed
    'REVERSE_MOTOR_DIR': _BV(28),
    # 0 INTR and TARGET_REACHED are outputs with strongly driven output values..
    # 1 INTR and TARGET_REACHED are used as outputs with gated pull-up and/or pull-down functionality.
    'INTR_TR_PU_PD_EN': _BV(29),
    # 0 INTR output function is used as Wired-Or in the case of intr_tr_pu_pd_en = 1.
    # 1 INTR output function is used as Wired-And. in the case of intr_tr_pu_pd_en = 1.
    'INTR_AS_WIRED_AND': _BV(30),
    # 0 TARGET_REACHED output function is used as Wired-Or in the case of intr_tr_pu_pd_en = 1.
    # 1 TARGET_REACHED output function is used as Wired-And in the case of intr_tr_pu_pd_en = 1.
    'TR_AS_WIRED_AND': _BV(31)
  })

class RampModeRegister(Register):
  """
  Ramp operation mode and motion profile.
  """
  REGISTER = TMC4361_RAMP_MODE_REGISTER
  bits = AttributeDict({
    # Bit 1:0: Motion Profile:
    #   0: No ramp: VACTUAL follows only VMAX (rectangle velocity shape).
    #   1: Trapezoidal ramp (incl. sixPoint ramp): Consideration of acceleration and deceleration values for generating VACTUAL without adapting the acceleration values.
    #   2: S-shaped ramp: Consideration of all ramp values (incl. bow values) for generating VACTUAL.
    'MOTION_PROFILE': _BV(0) | _BV(1),
    # Bit 2: Operation Mode:
    #   1 Positioning mode: XTARGET is superior target of velocity ramp.
    #   0 Velocitiy mode: VMAX is superior target of velocity ramp.
    'OPERATION_MODE': _BV(2)
  })

class ExternalClockFrequencyRegister(Register):
  REGISTER = TMC4361_CLK_FREQ_REGISTER
  bits = AttributeDict({
    # RW. External clock frequency fCLK; unsigned; 25 bits.
    'EXTERNAL_CLOCK_FREQUENCY': 0b1111111111111111111111111
  })

class MotorDriverSettingsRegister(Register):
  REGISTER = TMC4361_STEP_CONF_REGISTER
  bits = AttributeDict({
    # Highest microsteps resolution: 256 microsteps per fullstep.
    # Set to 256 for closed-loop operation.
    # When using a Step/Dir driver, it must be capable of a 256 resolution via
    # Step/Dir input for best performance (but lower resolution Step/Dir drivers
    # can be used as well).
    # 1 128 microsteps per fullstep.
    # 2 64 microsteps per fullstep.
    # 3 32 microsteps per fullstep.
    # 4 16 microsteps per fullstep.
    # 5 8 microsteps per fullstep.
    # 6 4 microsteps per fullstep.
    # 7 Halfsteps: 2 microsteps per fullstep.
    # 8 Full steps (maximum possible setting)
    'MSTEP_PER_FS': 0b1111,
    # Fullsteps per motor axis revolution
    'FS_PER_REV': 0b0000111111111111,
    # Selection of motor driver status bits for SPI response datagrams: ORed
    # with Motor Driver Status Register Set (7:0): if set here and a particular
    # flag is set from the motor stepper driver, an event will be generated at EVENTS(30)
    'MSTATUS_SELECTION': 0b000000000000000011111111
    # 31:24 Reserved. Set to 0x00.
  })
