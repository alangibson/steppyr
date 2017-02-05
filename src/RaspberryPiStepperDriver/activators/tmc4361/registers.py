"""
registers for TMC4361
"""
import functools
from RaspberryPiStepperDriver import set_bit, unset_bit, lsb, mask, _BV, get_bits
from RaspberryPiStepperDriver.activators.tmc4361.io import Datagram

WRITE_MASK = 0x80 # register | WRITE_MASK
READ_MASK = 0x7F # register & READ_MASK

# Defines ported to Python functions
# simple FP math see https://ucexperiment.wordpress.com/2012/10/28/fixed-point-math-on-the-arduino-platform/
FIXED_23_8_MAKE = lambda a: a * (1 << 8)
FIXED_22_2_MAKE = lambda a: a * (1 << 2)

def decode_twos_complement(input_value, num_bits):
	"""
  Calculates a two's complement integer from the given input value's bits
  https://en.wikipedia.org/wiki/Two's_complement
  """
	mask = 2**(num_bits - 1)
	return -(input_value & mask) + (input_value & ~mask)

# Convert floating point to/from fixed point
number_to_fixed = lambda float_value, fractional_bits: int(float_value * (1 << fractional_bits))
fixed_to_number = lambda fixed_value, fractional_bits: fixed_value * (2**-fractional_bits)

def unpack_bitmask_dict(bitmask_dict):
  if type(bitmask_dict) == dict:
    return (bitmask_dict['bitmask'], bitmask_dict['format'])
  else:
    return (bitmask_dict, None)

class AttributeDict(dict):
  __getattr__ = dict.__getitem__
  __setattr__ = dict.__setitem__

class Representation:
  """
  Value Representation

  Fixed Point Math on the Arduino Platform
    https://ucexperiment.wordpress.com/2012/10/28/fixed-point-math-on-the-arduino-platform/
  """
  def __init__(self, first_bit, last_bit=None, whole_bits=None, fractional_bits=0, signed=False):
    self._first_bit = first_bit
    if last_bit == None:
      last_bit = first_bit
    self._last_bit = last_bit
    self.bitmask = mask(first_bit, last_bit)
    if whole_bits == None:
      self._whole_bits = last_bit + 1 # first/last_bit is 0 indexed
    else:
      self._whole_bits = whole_bits
    self._fractional_bits = fractional_bits
    self._signed = signed

  def to_register_value(self, number):
    # TODO handle signed
    # Convert to fixed point
    value = number_to_fixed(number, self._fractional_bits)
    # Just apply the bitmask itself
    # self._data = set_bit(number, bitmask)
    # Apply value at bitmask bits
    value = set_bit(unset_bit(value, self.bitmask), (value << lsb(self.bitmask)))
    return value

  def from_register_value(self, register_value):
    # TODO handle signed
    # Apply bitmask to extract bits from encoded register value
    value = get_bits(register_value, self.bitmask)
    return fixed_to_number(value, self._fractional_bits)

class Register(Datagram):

  def __init__(self, data=0):
    self._header = self.REGISTER
    self._data = data

  def set(self, bitmask, value=None):
    """
    value: (int or float) If float, a Representation is needed to encode to
           valid register value.
    bitmask can be
    1. integer bitmask (or really any integer)
    # Disabled: 2. dict like: {bitmask: mask(0, 30), representation: Representation(23, 8, False)}
    """

    # Get representation and bitmask
    representation = None
    if type(bitmask) == Representation:
      representation = bitmask
      bitmask = representation.bitmask

    # Apply Representation if requested
    if representation:
      # Encode value to register value using Representation
      value = representation.to_register_value(value)
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

  def get(self, representation):
    return representation.from_register_value(self._data)

  def get_values(self):
    events = []
    for name, bitmask in self.bits.items():

      # Get representation and bitmask
      representation = None
      if type(bitmask) == Representation:
        representation = bitmask
        bitmask = representation.bitmask

      if self._data & bitmask:
        events.append((name, bitmask))
    return events

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
TMC4361_ENCODER_INPUT_CONFIG_REGISTER = 0x07
TMC4361_EVENT_CLEAR_CONF_REGISTER = 0x0c
TMC4361_INTERRUPT_CONFIG_REGISTER = 0x0d
# Various Configuration Registers: S/D, Synchronization, etc.
TMC4361_STP_LENGTH_ADD_REGISTER = 0x10
TMC4361_START_OUT_ADD_REGISTER = 0x11
TMC4361_GEAR_RATIO_REGISTER = 0x12
# RW. Bits 31:0. START_DELAY.
#   Delay time [# clock cycles] between start trigger and internal start signal release.
TMC4361_START_DELAY_REGISTER = 0x13
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
TMC4361_ENCODER_POSITION_REGISTER = 0x50
TMC4361_ENCODER_INPUT_RESOLUTION_REGISTER = 0x54
TMC4361_COVER_HIGH_REGISTER = 0x6D
TMC4361_COVER_DRV_HIGH_REGISTER = 0x6F
# how to mask REFERENCE_CONFIG_REGISTER if you want to configure just one end
TMC4361_LEFT_ENDSTOP_REGISTER_PATTERN = (_BV(0) | _BV(2) | _BV(6) | _BV(10) | _BV(11) | _BV(14))
TMC4361_RIGHT_ENDSTOP_REGISTER_PATTERN = (_BV(1) | _BV(3) | _BV(7) | _BV(12) | _BV(13) | _BV(15))

class XActualRegister(Register):
  REGISTER = 0x21
  bits = AttributeDict({
    # RW. 31:0 XACTUAL (Default: 0x00000000)
    #   Actual internal motor position [pulses]: –2^31 ≤ XACTUAL ≤ 2^31 – 1
    #   signed; 32 bits
    'XACTUAL': Representation(0, 31, 32, 0, True)
  })
class VActualRegister(Register):
  REGISTER = 0x22
  bits = AttributeDict({
    # R. Current step velocity; 24 bits; signed; no decimals.
    # Bit 31:0: Actual ramp generator velocity [pulses per second]:
    #   1 pps ≤ |VACTUAL| ≤ CLK_FREQ · 1/2 pulses (fCLK = 16 MHz -> 8 Mpps)
    'VACTUAL': Representation(0, 31, 24, 0, True)
  })
class AActualRegister(Register):
  REGISTER = 0x23
  bits = AttributeDict({
    # R. 31:0 AACTUAL (Default: 0x00000000)
    # Current step acceleration; 24 bits; signed; no decimals.
    'AACTUAL': Representation(0, 31, 24, 0, True)
  })
class VMaxRegister(Register):
  REGISTER = 0x24
  bits = AttributeDict({
    # RW 31:0 VMAX (Default: 0x00000000)
    # Maximum ramp generator velocity in positioning mode or
    # Target ramp generator velocity in velocity mode and no ramp motion profile.
    # Value representation: signed; 32 bits= 24+8 (24 bits integer part, 8 bits decimal places).
    'VMAX': Representation(0, 31, 24, 8, True)
  })
class VStartRegister(Register):
  REGISTER = 0x25
  bits = AttributeDict({
    # RW 30:0 VSTART (Default: 0x00000000)
    # Absolute start velocity in positioning mode and velocity mode
    # In case VSTART is used: no first bow phase B1 for S-shaped ramps
    # VSTART in positioning mode:
    #   In case VACTUAL = 0 and XTARGET ≠ XACTUAL:
    #   no acceleration phase for VACTUAL = 0  VSTART.
    # VSTART in velocity mode:
    #   In case VACTUAL = 0 and VACTUAL ≠ VMAX:
    #   no acceleration phase for VACTUAL = 0  VSTART.
    # Value representation: 23 digits and 8 decimal places. unsigned; 31 bits=23+8.
    'VSTART': Representation(0, 30, 23, 8, False)
  })
class VStopRegister(Register):
  REGISTER = 0x26
  bits = AttributeDict({
    # RW 30:0 VSTOP (Default:0x00000000)
    # Absolute stop velocity in positioning mode and velocity mode.
    # In case VSTOP is used: no last bow phase B4 for S-shaped ramps.
    # In case VSTOP is very small and positioning mode is used, it is possible that the
    # ramp is finished with a constant VACTUAL = VSTOP until XTARGET is reached.
    # VSTOP in positioning mode:
    #   In case VACTUAL≤VSTOP and XTARGET=XACTUAL: VACTUAL is immediately set to 0.
    # VSTOP in velocity mode:
    #   In case VACTUAL ≤ VSTOP and VMAX = 0: VACTUAL is immediately set to 0.
    # Value representation: 23 digits and 8 decimal places. unsigned; 31 bits=23+8.
    'VSTOP': Representation(0, 30, 23, 8, False)
  })
class VBreakRegister(Register):
  REGISTER = 0x27
  bits = AttributeDict({
    # RW 30:0 VBREAK (Default:0x00000000)
    # Absolute break velocity in positioning mode and in velocity mode,
    # This only applies for trapezoidal ramp motion profiles.
    # In case VBREAK = 0: pure linear ramps are generated with AMAX / DMAX only.
    # In case |VACTUAL| < VBREAK: |AACTUAL| = ASTART or DFINAL
    # In case |VACTUAL| ≥ VBREAK: |AACTUAL| = AMAX or DMAX
    # Always set VBREAK > VSTOP! If VBREAK != 0.
    # Value representation: 23 digits and 8 decimal places. unsigned; 31 bits=23+8.
    'VBREAK': Representation(0, 30, 23, 8, False)
  })
class AMaxRegister(Register):
  REGISTER = 0x28
  bits = AttributeDict({
    # RW 23:0 AMAX (Default: 0x000000)
    # S-shaped ramp motion profile: Maximum acceleration value.
    # Trapezoidal ramp motion profile:
    #   Acceleration value in case |VACTUAL| ≥ VBREAK or in case VBREAK = 0.
    # Value representation:
    #   Frequency mode: [pulses per sec2]
    #     22 digits and 2 decimal places: 250 mpps2 ≤ AMAX ≤ 4 Mpps2
    #     unsigned; 24 bits=22+2 (22 bits integer part, 2 bits decimal places).
    #   Direct mode: [∆v per clk cycle]
    #     a[∆v per clk_cycle]= AMAX / 237
    #     AMAX [pps2] = AMAX / 237 • fCLK2
    'AMAX': Representation(0, 23, 22, 2, False)
  })
class DMaxRegister(Register):
  REGISTER = 0x29
  bits = AttributeDict({
    # RW 23:0 DMAX (Default: 0x000000)
    # S-shaped ramp motion profile: Maximum deceleration value.
    # Trapezoidal ramp motion profile:
    #   Deceleration value if |VACTUAL| ≥ VBREAK or if VBREAK = 0.
    # Value representation:
    #   Frequency mode: [pulses per sec2]
    #     22 digits and 2 decimal places: 250 mpps2 ≤ DMAX ≤ 4 Mpps2
    #     unsigned; 24 bits=22+2.
    #   Direct mode: [∆v per clk cycle]
    #     d[∆v per clk_cycle]= DMAX / 237
    #     DMAX [pps2] = DMAX / 237 • fCLK2
    'DMAX': Representation(0, 23, 22, 2, False)
  })
# RW. First bow value of a complete velocity ramp; unsigned; 24 bits=24+0 (24 bits integer part, no decimal places).
class Bow1Register(Register):
  REGISTER = 0x2d
  bits = AttributeDict({
    # RW 23:0 BOW1 (Default: 0x000000)
    # Bow value 1 (first bow B1 of the acceleration ramp).
    # Value representation:
    #   Frequency mode: [pulses per sec3]
    #     24 digits and 0 decimal places: 1 pps3 ≤ BOW1 ≤ 16 Mpps3
    #      unsigned; 24 bits=24+0 (24 bits integer part, no decimal places).
    #   Direct mode: [∆a per clk cycle]
    #     bow[av per clk_cycle]= BOW1 / 253
    #     BOW1 [pps3] = BOW1 / 253 • fCLK3
    'BOW1': Representation(0, 23, 24, 0, False)
  })
class Bow2Register(Register):
  REGISTER = 0x2e
  bits = AttributeDict({
    # RW 23:0 BOW2 (Default: 0x000000)
    # Bow value 2 (second bow B2 of the acceleration ramp).
    # Value representation:
    #   Frequency mode: [pulses per sec3]
    #     24 digits and 0 decimal places: 1 pps3 ≤ BOW2 ≤ 16 Mpps3
    #     unsigned; 24bits=24+0.
    #   Direct mode: [∆a per clk cycle]
    #     bow[av per clk_cycle]= BOW2 / 253
    #     BOW2 [pps3] = BOW2 / 253 • fCLK3
    'BOW2': Representation(0, 23, 24, 0, False)
  })
class Bow3Register(Register):
  REGISTER = 0x2f
  bits = AttributeDict({
    # RW 23:0 BOW3 (Default: 0x000000)
    # Bow value 3 (first bow B3 of the deceleration ramp).
    # Value representation:
    #   Frequency mode: [pulses per sec3]
    #     24 digits and 0 decimal places: 1 pps3 ≤ BOW3 ≤ 16 Mpps3
    #       unsigned; 24 bits=24+0.
    #   Direct mode: [∆a per clk cycle]
    #     bow[av per clk_cycle]= BOW3 / 253
    #     BOW3 [pps3] = BOW3 / 253 • fCLK3
    'BOW3': Representation(0, 23, 24, 0, False)
  })
class Bow4Register(Register):
  REGISTER = 0x30
  bits = AttributeDict({
    # RW 23:0 BOW 4 (Default: 0x000000)
    # Bow value 4 (second bow B4 of the deceleration ramp).
    # Value representation:
    #   Frequency mode: [pulses per sec3]
    #     24 digits and 0 decimal places: 1 pps3 ≤ BOW4 ≤ 16 Mpps3
    #     unsigned; 24 bits=24+0.
    #   Direct mode: [∆a per clk cycle]
    #     bow[av per clk_cycle]= BOW4 / 253
    #     BOW4 [pps3] = BOW4 / 253 • fCLK3
    'BOW4': Representation(0, 23, 24, 0, False)
  })
class AStartRegister(Register):
  REGISTER = 0x2A
  bits = AttributeDict({
    # RW 23:0 ASTART (Default: 0x000000)
    #   S-shaped ramp motion profile: start acceleration value.
    #   Trapezoidal ramp motion profile:
    #     Acceleration value in case |VACTUAL| < VBREAK.
    #     Acceleration value after switching from external to internal step control.
    #   Value representation:
    #     Frequency mode: [pulses per sec2]
    #       22 digits and 2 decimal places: 250 mpps2 ≤ ASTART ≤ 4 Mpps2
    #       unsigned; 24 bits=22+2
    #     Direct mode: [∆v per clk cycle]
    #       a[∆v per clk_cycle]= ASTART / 237
    #       ASTART [pps2] = ASTART / 237 • fCLK2
    #       Consider maximum values, represented in section 6.7.5, page 50
    'ASTART': Representation(0, 23, 22, 2, False),
    # RW 31 Sign of AACTUAL after switching from external to internal step control.
    'AACTUAL_SIGN': _BV(31)
  })
class DFinalRegister(Register):
  REGISTER = 0x2B
  bits = AttributeDict({
    # RW 23:0 DFINAL (Default: 0x000000)
    #   S-shaped ramp motion profile: Stop deceleration value, which is not used during positioning mode.
    #   Trapezoidal ramp motion profile:
    #     Deceleration value in case |VACTUAL| < VBREAK.
    #   Value representation:
    #     Frequency mode: [pulses per sec2]
    #       22 digits and 2 decimal places: 250 mpps2 ≤ DFINAL ≤ 4 Mpps2
    #       unsigned; 24 bits=22+2
    #     Direct mode: [∆v per clk cycle]
    #       d[∆v per clk_cycle]= DFINAL / 237
    #       DFINAL [pps2] = DFINAL / 237 • fCLK2
    'DFINAL': Representation(0, 23, 22, 2, False)
  })
class StatusEventRegister(Register):
  """
  """
  REGISTER = 0x0E
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
  REGISTER = 0x0B
  bits = StatusEventRegister.bits

class GeneralConfigurationRegister(Register):
  REGISTER = 0x0
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
    'SERIAL_ENC_IN_MODE': Representation(10, 11),
    # 0 Differential encoder interface inputs enabled.
    # 1 Differential encoder interface inputs is disabled (automatically set for SPI encoder).
    'DIFF_ENC_IN_DISABLE': _BV(12),
    # 0 Standby signal becomes forwarded with an active low level at STDBY_CLK output.
    # 1 Standby signal becomes forwarded with an active high level at STDBY_CLK output.
    # 2 STDBY_CLK passes ChopSync clock (TMC23x, TMC24x stepper motor drivers only).
    # 3 Internal clock is forwarded to STDBY_CLK output pin.
    'STDBY_CLK_PIN_ASSIGNMENT': Representation(13, 14),
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
    'DCSTEP_MODE': Representation(21, 22),
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
  REGISTER = 0x20
  bits = AttributeDict({
    # Bit 1:0: Motion Profile:
    #   0: No ramp: VACTUAL follows only VMAX (rectangle velocity shape).
    #   1: Trapezoidal ramp (incl. sixPoint ramp): Consideration of acceleration and deceleration values for generating VACTUAL without adapting the acceleration values.
    #   2: S-shaped ramp: Consideration of all ramp values (incl. bow values) for generating VACTUAL.
    'MOTION_PROFILE': Representation(0, 1),
    # Bit 2: Operation Mode:
    #   1 Positioning mode: XTARGET is superior target of velocity ramp.
    #   0 Velocitiy mode: VMAX is superior target of velocity ramp.
    'OPERATION_MODE': _BV(2)
  })

class ExternalClockFrequencyRegister(Register):
  REGISTER = 0x31
  bits = AttributeDict({
    # RW. External clock frequency fCLK; unsigned; 25 bits.
    'EXTERNAL_CLOCK_FREQUENCY': Representation(0, 24)
  })

class MotorDriverSettingsRegister(Register):
  REGISTER = 0x0A
  bits = AttributeDict({
    # 3:0 Highest microsteps resolution: 256 microsteps per fullstep.
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
    'MSTEP_PER_FS': Representation(0, 3),
    # 15:4 Fullsteps per motor axis revolution
    'FS_PER_REV': Representation(4, 15),
    # 23:16 Selection of motor driver status bits for SPI response datagrams: ORed
    # with Motor Driver Status Register Set (7:0): if set here and a particular
    # flag is set from the motor stepper driver, an event will be generated at EVENTS(30)
    'MSTATUS_SELECTION': Representation(16, 23)
    # 31:24 Reserved. Set to 0x00.
  })

class StatusFlagRegister(Register):
  REGISTER = 0x0F
  bits = AttributeDict({
    # 0 TARGET_REACHED_F is set high if XACTUAL = XTARGET
    'TARGET_REACHED_F': _BV(0),
    # 1 POS_COMP_REACHED_F is set high if XACTUAL = POS_COMP
    'POS_COMP_REACHED_F': _BV(1),
    # 2 VEL_REACHED_F is set high if VACTUAL = |VMAX|
    'VEL_REACHED_F': _BV(2),
    # 4:3 VEL_STATE_F: Current velocity state: 0  VACTUAL = 0;
    #   1  VACTUAL > 0;
    #   2  VACTUAL < 0
    'VEL_STATE_F': Representation(3, 4),
    # 6:5 RAMP_STATE_F: Current ramp state: 0  AACTUAL = 0;
    #   1  AACTUAL increases (acceleration);
    #   2  AACTUAL decreases (deceleration)
    'RAMP_STATE_F': Representation(5, 6),
    # 7 STOPL_ACTIVE_F: Left stop switch is active.
    'STOPL_ACTIVE_F': _BV(7),
    # 8 STOPR_ACTIVE_F: Right stop switch is active.
    'STOPR_ACTIVE_F': _BV(8),
    # 9 VSTOPL_ACTIVE_F: Left virtual stop switch is active.
    'VSTOPL_ACTIVE_F': _BV(9),
    # 10 VSTOPR_ACTIVE_F: Right virtual stop switch is active.
    'VSTOPR_ACTIVE_F': _BV(10),
    # 11 ACTIVE_STALL_F: Motor stall is detected and VACTUAL > VSTALL_LIMIT.
    'ACTIVE_STALL_F': _BV(11),
    # 12 HOME_ERROR_F: HOME_REF input signal level is not equal to expected home level.
    'HOME_ERROR_F': _BV(12),
    # 13 FS_ACTIVE_F: Fullstep operation is active.
    'FS_ACTIVE_F': _BV(13),
    # 14 ENC_FAIL_F: Mismatch between XACTUAL and ENC_POS is out of tolerated range.
    'ENC_FAIL_F': _BV(14),
    # 15 N_ACTIVE_F: N event is active.
    'N_ACTIVE_F': _BV(15),
    # 16 ENC_LATCH_F: ENC_LATCH is rewritten.
    'ENC_LATCH_F': _BV(16),
    # 17 Applies to absolute encoders only:
    #    MULTI_CYCLE_FAIL_F indicates a failure during last multi cycle data evaluation.
    #    Applies to absolute encoders only:
    #    SER_ENC_VAR_F indicates a failure during last serial data evaluation due to a substantial
    #    deviation between two consecutive serial data values.
    # TODO '': _BV(),
    # 18 Reserved.
    # 19 CL_FIT_F: Active if ENC_POS_DEV < CL_TOLERANCE. The current mismatch
    #    between XACTUAL and ENC_POS is within tolerated range.
    'CL_FIT_F': _BV(19),
    # 23:20 Applies to absolute encoders only: SERIAL_ENC_FLAGS received from
    #   encoder. These flags are reset with a new encoder transfer request.
    'SERIAL_ENC_FLAGS': Representation(20, 23),
    # 24 TMC26x / TMC2130 only: SG: StallGuard2 status
    #    Optional for TMC24x only: Calculated stallGuard status.
    #    TMC23x / TMC24x only: UV_SF: Undervoltage flag.
    'SG_OR_UV_SF': _BV(24),
    # 25 All TMC motor drivers: OT: Overtemperature shutdown.
    'OT_SHUTDOWN_F': _BV(25),
    # 26 All TMC motor drivers: OTPW: Overtemperature warning.
    'OT_WARNING_F': _BV(26),
    # 27 TMC26x / TMC2130 only: S2GA: Short to ground detection bit for high side MOSFE of coil A.
    #    TMC23x / TMC24x only: OCA: Overcurrent bridge A.
    'S2G_A_OR_OC_A_F': _BV(27),
    # 28 TMC26x / TMC2130 only: S2GB: Short to ground detection bit for high side MOSFET of coil B.
    #    TMC23x / TMC24x only: OCB: Overcurrent bridge B.
    'S2G_B_OR_OC_B_F': _BV(28),
    # 29 All TMC motor drivers: OLA: Open load indicator of coil A.
    'OL_A_F': _BV(29),
    # 30 All TMC motor drivers: OLB: Open load indicator of coil B.
    'OL_B_F': _BV(30),
    # 31 TMC26x / TMC2130 only: STST: Standstill indicator.
    #    TMC23x / TMC24x only: OCHS: Overcurrent high side.
    'STST_OR_OCHS_F': _BV(31)
  })

class SPIOutConfRegister(Register):
  REGISTER = 0x04
  bits = AttributeDict({
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
    'SPI_OUTPUT_FORMAT': Representation(0, 3),
    # 4 (TMC389 only)
    #   0 A 2-phase stepper motor driver is connected to the SPI output (TMC26x).
    #   1 A 3-phase stepper motor driver is connected to the SPI output (TMC389)
    'THREE_PHASE_STEPPER_EN': _BV(4),
    # 4 (No TMC driver)
    #   0 NSCSDRV_SDO is tied low before SCKDRV_NSDO to initiate a new data transfer.
    #   1 SCKDRV_NSDO is tied low before NSCSDRV_SDO to initiate a new data transfer.
    'SCK_LOW_BEFORE_CSN': _BV(4),
    # 5:4 (TMC23x/24x only)
    #   0 Both mixed decay bits are always off.
    #   1 Mixed decay bits are on during falling ramps until reaching a current value of 0.
    #   2 Mixed decay bits are always on, except during standstill.
    #   3 Mixed decay bits are always on.
    'MIXED_DECAY': Representation(4, 5),
    # 23:4 (Serial encoder output only)
    #   U Monoflop time for SSI output interface: Delay time [clock cycles] during which the absolute encoder data remain stable after the last master request.
    'SSI_OUT_MTIME': Representation(4, 23),
    # 5 (TMC26x/2130 in SD mode only)
    #   0 No transfer of scale values.
    #   1 Transmission of current scale values to the appropriate driver registers.
    'SCALE_VAL_TRANSFER_EN': _BV(5),
    # 5 (No TMC driver)
    #   0 New value bit at SDODRV_SCLK is assigned at falling edge of SCKDRV_NSDO.
    #   1 New value bit at SDODRV_SCLK is assigned at rising edge of SCKDRV_NSDO.
    'NEW_OUT_BIT_AT_RISE': _BV(5),
    # 6 (TMC24x only)
    #   0 No standby datagram is sent.
    #   1 In case of a Stop-on-Stall event, a standby datagram is sent to the TMC24x.
    'STDBY_ON_STALL_FOR_24X': _BV(6),
    # 6 (TMC26x/2130 in SD mode only)
    #   0 Permanent transfer of polling datagrams to check driver status.
    #   1 No transfer of polling datagrams.
    'DISABLE_POLLING': _BV(6),
    # 7 (TMC24x only)
    #   0 Undervoltage flag of TMC24x is mapped at STATUS(24).
    #   1 Calculated stall status of TMC24x is forwarded at STATUS(24).
    'STALL_FLAG_INSTEAD_OF_UV_EN': _BV(7),
    # 7 (TMC26x/2130 only)
    #   0 No automatic continuous streaming of cover datagrams.
    #   1 Enabling of automatic continuous streaming of cover datagrams.
    'AUTOREPEAT_COVER_EN': _BV(7),
    # 11:7  (SPI-DAC only)
    # U Number of bits for command address.
    # 12 Reserved. Set to 0.
    'DAC_CMD_LENGTH': Representation(7, 11),
    # 10:8 (TMC24x only)
    #   U A stall is detected if the stall limit value STALL_LOAD_LIMIT is higher than the combination of the load bits (LD2&LD1&LD0).
    'STALL_LOAD_LIMIT': Representation(8, 10),
    # 11:8 (TMC26x in SD mode only, TMC2130 only)
    #   U Multiplier for calculating the time interval between two consecutive polling datagrams: tPOLL = 2^POLL_BLOCK_EXP ∙ SPI_OUT_BLOCK_TIME / fCLK
    'POLL_BLOCK_EXP': Representation(8, 11),
    # 11 (TMC24x only)
    #   0 No phase shift during PWM mode.
    #   1 During PWM mode, the internal SinLUT microstep position MSCNT is shifted to MS_OFFSET microsteps. Consequently, the sine/cosine values have a phase shift of (MS_OFFSET / 1024 ∙ 360°)
    'PWM_PHASE_SHFT_EN': _BV(11),
    # 12 (TMC23x/24x only)
    #   0 ChopSync frequency remains stable during standby.
    #   1 CHOP_SYNC_DIV is halfed during standby.
    'DOUBLE_FREQ_AT_STDBY': _BV(12),
    # 12 (TMC26x/2130 only)
    #   0 COVER_DONE event is set for every datagram that is sent to the motor driver.
    #   1 COVER_DONE event is only set for cover datagrams sent to the motor driver.
    'COVER_DONE_ONLY_FOR_COVER': _BV(12),
    # Bit 19:13. COVER_DATA_LENGTH
    #   U Number of bits for the complete datagram length. Maximum value = 64
    #     Set to 0 in case a TMC stepper motor driver is selected. The datagram length is then
    #     selected automatically.
    'COVER_DATA_LENGTH': Representation(13, 19),
    # Bits 23:20. SPI_OUT_LOW_TIME
    #   U Number of clock cycles the SPI output clock remains at low level.
    'SPI_OUT_LOW_TIME': Representation(20, 23),
    # Bits 27:24. SPI_OUT_HIGH_TIME
    #   U Number of clock cycles the SPI output clock remains at high level
    'SPI_OUT_HIGH_TIME': Representation(24, 27),
    # Bits 31:28. SPI_OUT_BLOCK_TIME
    #   U Number of clock cycles the NSCSDRV output remains high (inactive) after a SPI
    #     output transmission
    'SPI_OUT_BLOCK_TIME': Representation(28, 31)
  })

class CoverLowRegister(Register):
  """
  This register is write-only COVER_LOW and read-only POLLING_STATUS.
  """
  REGISTER = 0x6C
  bits = AttributeDict({
    'COVER_LOW_OR_POLLING_STATUS': Representation(0, 31)
  })

class CoverDriverLowRegister(Register):
  """
  Lower configuration bits of SPI response received from the motor driver connected to
  the SPI output.
  """
  REGISTER = 0x6E
  bits = AttributeDict({
    # 31:0 (Default:0x00000000)
    # Lower configuration bits of SPI response received from the motor driver
    # connected to the SPI output.
    'COVER_DRV_LOW': Representation(0, 31)
  })

class ResetClockAndGatingRegister(Register):
  REGISTER = 0x4F
  bits = AttributeDict({
    # RW. Bit 2:0.
    #   0 Clock gating is not activated.
    #   7 Clock gating is activated.
    'CLK_GATING_REG': Representation(0, 2),
    # Bits 31:8.
    #  0 No reset is activated.
    #  0x525354 Internal reset is activated.
    'RESET_REG': Representation(8, 31)
  })

# TODO finish this
class InputFilterRegister(Register):
  REGISTER = 0x3
