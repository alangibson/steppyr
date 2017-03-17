"""
registers for TMC4361
"""
from steppyr.lib import AttributeDict
from steppyr.lib.trinamic import Representation, Register as TrinamicRegister
from .io import Datagram

WRITE_MASK = 0x80 # register | WRITE_MASK
READ_MASK = 0x7F # register & READ_MASK

# Defines ported to Python functions
# simple FP math see https://ucexperiment.wordpress.com/2012/10/28/fixed-point-math-on-the-arduino-platform/
FIXED_23_8_MAKE = lambda a: a * (1 << 8)
FIXED_22_2_MAKE = lambda a: a * (1 << 2)

class Register(TrinamicRegister, Datagram):
  def __init__(self, data=0, header=None, header_len=8, datagram_len=40):
    super().__init__(data=data, header=header, header_len=header_len, datagram_len=datagram_len)

class ReferenceConfRegister(Register):
  REGISTER = 0x01
  bits = AttributeDict({
    # 0 stop_left_en
    #   0 STOPL signal processing disabled.
    #   1 STOPL signal processing enabled.
    'STOP_LEFT_EN': Representation(0),
    # 1 stop_right_en
    #   0 STOPR signal processing disabled.
    #   1 STOPR signal processing enabled.
    'STOP_RIGHT_EN': Representation(1),
    # 2 pol_stop_left
    #   0 STOPL input signal is low active.
    #   1 STOPL input signal is high active.
    'POL_STOP_LEFT': Representation(2),
    # 3 pol_stop_right
    #   0 STOPR input signal is low active.
    #   1 STOPR input signal is high active.
    'POL_STOP_RIGHT': Representation(3),
    # 4 invert_stop_direction
    #   0 STOPL / STOPR stops motor in negative / positive direction.
    #   1 STOPL / STOPR stops motor in positive / negative direction.
    'INVERT_STOP_DIRECTION': Representation(4),
    # 5 soft_stop_en
    #   0 Hard stop enabled. VACTUAL is immediately set to 0 on any external stop event.
    #   1 Soft stop enabled.A linear velocity ramp is used for decreasing VACTUAL to v = 0.
    'SOFT_STOP_EN': Representation(5),
    # 6 virtual_left_limit_en
    #   0 Position limit VIRT_STOP_LEFT disabled.
    #   1 Position limit VIRT_STOP_LEFT enabled.
    'VIRTUAL_LEFT_LIMIT_EN': Representation(6),
    # 7 virtual_right_limit_en
    #   0 Position limit VIRT_STOP_RIGHT disabled.
    #   1 Position limit VIRT_STOP_RIGHT enabled.
    'VIRTUAL_RIGHT_LIMIT_EN': Representation(7),
    # 9:8 virt_stop_mode
    #   0 Reserved.
    #   1 Hard stop: VACTUAL is set to 0 on a virtual stop event.
    #   2 Soft stop is enabled with linear velocity ramp (from VACTUAL to v = 0).
    #   3 Reserved.
    'VIRT_STOP_MODE': Representation(8, 9),
    # 10 latch_x_on_inactive_l
    #   0 No latch of XACTUAL if STOPL becomes inactive.
    #   1 X_LATCH = XACTUAL is stored in the case STOPL becomes inactive.
    'LATCH_X_ON_INACTIVE_L': Representation(10),
    # 11 latch_x_on_active_l
    #   0 No latch of XACTUAL if STOPL becomes active.
    #   1 X_LATCH = XACTUAL is stored in the case STOPL becomes active.
    'LATCH_X_ON_ACTIVE_L': Representation(11),
    # 12 latch_x_on_inactive_r
    #   0 No latch of XACTUAL if STOPR becomes inactive.
    #   1 X_LATCH = XACTUAL is stored in the case STOPL becomes inactive.
    'LATCH_X_ON_INACTIVE_R': Representation(12),
    # 13 latch_x_on_active_r
    #   0 No latch of XACTUAL if STOPR becomes active.
    #   1 X_LATCH = XACTUAL is stored in the case STOPL becomes active.
    'LATCH_X_ON_ACTIVE_R': Representation(13),
    # 14 stop_left_is_home
    #   0 STOPL input signal is not also the HOME position.
    #   1 STOPL input signal is also the HOME position.
    'STOP_LEFT_IS_HOME': Representation(14),
    # 15 stop_right_is_home
    #   0 STOPR input signal is not lso the HOME position.
    #   1 STOPR input signal is also the HOME position.
    'STOP_RIGHT_IS_HOME': Representation(15),
    # 19:16 home_event
    #   0 Next active N event of connected ABN encoder signal indicates HOME position.
    #   2 HOME_REF = 1 indicates an active home event X_HOME is located at the rising edge of the active range.
    #   3 HOME_REF = 0 indicates negative region / position from the home position.
    #   4 HOME_REF = 1 indicates an active home event X_HOME is located at the falling edge of the active range.
    #   6 HOME_REF = 1 indicates an active home event X_HOME is located in the middle of the active range.
    #   9 HOME_REF = 0 indicates an active home event X_HOME is located in the middle of the active range.
    #   11 HOME_REF = 0 indicates an active home event X_HOME is located at the rising edge of the active range.
    #   12 HOME_REF = 1 indicates negative region / position from the home position.
    #   13 HOME_REF = 0 indicates an active home event X_HOME is located at the falling edge of the active range.
    'HOME_EVENT': Representation(16, 19),
    # 20 start_home_tracking
    #   0 No storage to X_HOME by passing home position.
    #   1 Storage of XACTUAL as X_HOME at next regular home event.
    #     An XLATCH_DONE event is released.
    #     In case the event is cleared, start_home_tracking is reset automatically
    'START_HOME_TRACKING': Representation(20),
    # 21 clr_pos_at_target
    #   0 Ramp stops at XTARGET if positioning mode is active.
    #   1 Set XACTUAL = 0 after XTARGET has been reached.
    #   The next ramp starts immediately.
    'CLR_POS_AT_TARGET': Representation(21),
    # 22 circular_movement_en
    #   0 Range of  XACTUAL is not limited: -2 31 ≤ XACTUAL ≤ 2 31 - 1
    #   1 Range of XACTUAL is limited by X_RANGE: -X_RANGE ≤ XACTUAL ≤ X_RANGE - 1
    'CIRCULAR_MOVEMENT_EN': Representation(22),
    # 24:23 pos_comp_output
    #   0 TARGET_REACHED is set active on TARGET_REACHED_Flag.
    #   1 TARGET_REACHED is set active on VELOCITY_REACHED_Flag.
    #   2 TARGET_REACHED is set active on ENC_FAIL flag.
    #   3 TARGET_REACHED triggers on POSCOMP_REACHED_Flag.
    'POS_COMP_OUTPUT': Representation(23, 24),
    # 25 pos_comp_source
    #   0 POS_COMP is compared to internal position XACTUAL.
    #   1 POS_COMP is compared with external position ENC_POS.
    'POS_COMP_SOURCE': Representation(25),
    # 26 stop_on_stall
    #   0 SPI and S / D output interface remain active in case of an stall event.
    #   1 SPI and S / D output interface stops motion in case of an stall event (hard stop).
    'STOP_ON_STALL': Representation(26),
    # 27 drv_after_stall
    #   0 No further motion in case of an active stop-on-stall event.
    #   1 Motion is possible in case of an active stop-on-stall event and after the stop-on-stall event is reset.
    'DRV_AFTER_STALL': Representation(27),
    # 29:28 modified_pos_compare:
    #       POS_COMP_REACHED_F / event is based on comparison between XACTUAL resp.ENC_POS and
    #   0 POS_COMP
    #   1 X_HOME
    #   2 X_LATCH resp.ENC_LATCH
    #   3 REV_CNT
    'MODIFIED_POS_COMPARE': Representation(28, 29),
    # 30 automatic_cover
    #   0 SPI output interface will not transfer automatically any cover datagram.
    #   1 SPI output interface sends automatically cover datagrams when VACTUAL crosses SPI_SWITCH_VEL.
    'AUTOMATIC_COVER': Representation(30),
    # 31 circular_enc_en
    #   0 Range of ENC_POS is not limited: -2 31 ≤ ENC_POS ≤ 2 31 - 1
    #   1 Range of ENC_POS is limited by X_RANGE: -X_RANGE ≤ ENC_POS ≤ X_RANGE –1
    'CIRCULAR_ENC_EN': Representation(31)
  })

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
    # R. Bit 31:0: Actual ramp generator velocity [pulses per second]:
    #   1 pps ≤ |VACTUAL| ≤ CLK_FREQ · 1/2 pulses (fCLK = 16 MHz -> 8 Mpps)
    # TODO actually Representation(0, 31, 24, 0, True). Representation needs bug fix
    'VACTUAL': Representation(0, 23, 24, 0, True)
  })
class AActualRegister(Register):
  REGISTER = 0x23
  bits = AttributeDict({
    # R. 31:0 AACTUAL (Default: 0x00000000)
    # Current step acceleration; 24 bits; signed; no decimals.
    # TODO actually Representation(0, 31, 24, 0, True). Representation needs bug fix
    'AACTUAL': Representation(0, 23, 24, 0, True)
  })
class VMaxRegister(Register):
  REGISTER = 0x24
  bits = AttributeDict({
    # RW 31:0 VMAX (Default: 0x00000000)
    # Defined as pulses per second [pps].
    # Maximum ramp generator velocity in positioning mode or
    # Target ramp generator velocity in velocity mode and no ramp motion profile.
    # Value representation: signed; 32 bits = 24+8 (24 bits integer part, 8 bits decimal places).
    # The maximum velocity VMAX is restricted as follows:
    #   Velocity mode: |VMAX| ≤ ½ pulse · fCLK
    #   Positioning mode: |VMAX| ≤ ¼ pulse · fCLK
    'VMAX': Representation(0, 31, 24, 8, True)
  })
class VStartRegister(Register):
  REGISTER = 0x25
  bits = AttributeDict({
    # RW 30:0 VSTART (Default: 0x00000000)
    # Defined as pulses per second [pps].
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
    # Defined as pulses per second [pps].
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
    # Defined as pulses per second [pps].
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
class XTargetRegister(Register):
  REGISTER = 0x37
  bits = AttributeDict({
    # RW 31:0 X_TARGET (Default: 0x00000000)
    # Target motor position in positioning mode
    #  signed; 32 bits.
    'XTARGET': Representation(0, 31, 32, 0, True)
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
    'AACTUAL_SIGN': Representation(31)
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
    'TARGET_REACHED': Representation(0),
    # POS_COMP_REACHED has been triggered.
    'POS_COMP_REACHED': Representation(1),
    # VEL_REACHED has been triggered.
    'VEL_REACHED': Representation(2),
    # VEL_STATE': b’00 has been triggered (VACTUAL = 0).
    'VACTUAL_EQ_0': Representation(3),
    # VEL_STATE = b’01 has been triggered (VACTUAL > 0).
    'VACTUAL_GT_0': Representation(4),
    # VEL_STATE = b’10 has been triggered (VACTUAL < 0).
    'VACTUAL_LT_0': Representation(5),
    # RAMP_STATE = b’00 has been triggered (AACTUAL = 0, VACTUAL is constant).
    'RAMP_STATE_00': Representation(6),
    # RAMP_STATE = b’01 has been triggered (|VACTUAL| increases).
    'RAMP_STATE_01': Representation(7),
    # RAMP_STATE = b’10 has been triggered (|VACTUAL| increases).
    'RAMP_STATE_10': Representation(8),
    # MAX_PHASE_TRAP: Trapezoidal ramp has reached its limit speed using maximum values for AMAX or DMAX (|VACTUAL| > VBREAK; VBREAK≠0).
    'MAX_PHASE_TRAP': Representation(9),
    # FROZEN: NFREEZE has switched to low level. Reset TMC4361A for further motion.
    'FROZEN': Representation(10),
    # STOPL has been triggered. Motion in negative direction is not executed until this event is cleared and (STOPL is not active any more or stop_left_en is set to 0).
    'STOPL_TRIGGERED': Representation(11),
    # STOPR has been triggered. Motion in positive direction is not executed until this event is cleared and (STOPR is not active any more or stop_right_en is set to 0).
    'STOPR_TRIGGERED': Representation(12),
    # VSTOPL_ACTIVE: VSTOPL has been activated. No further motion in negative direction until this event is cleared and (a new value is chosen for VSTOPL or virtual_left_limit_en is set to 0).
    'VSTOPL_ACTIVE': Representation(13),
    # VSTOPR_ACTIVE: VSTOPR has been activated. No further motion in positive direction until this event is cleared and (a new value is chosen for VSTOPR or virtual_right_limit_en is set to 0).
    'VSTOPR_ACTIVE': Representation(14),
    # HOME_ERROR: Unmatched HOME_REF polarity and HOME is outside of safety margin.
    'HOME_ERROR': Representation(15),
    # XLATCH_DONE indicates if X_LATCH was rewritten or homing process has been completed.
    'XLATCH_DONE': Representation(16),
    # FS_ACTIVE: Fullstep motion has been activated.
    'FS_ACTIVE': Representation(17),
    # ENC_FAIL: Mismatch between XACTUAL and ENC_POS has exceeded specified limit.
    'ENC_FAIL': Representation(18),
    # N_ACTIVE: N event has been activated.
    'N_ACTIVE': Representation(19),
    # ENC_DONE indicates if ENC_LATCH was rewritten.
    'ENC_DONE': Representation(20),
    # SER_ENC_DATA_FAIL: Failure during multi-cycle data evaluation or between two consecutive data requests has occured.
    'SER_ENC_DATA_FAIL': Representation(21),
    # 22: Reserved
    # SER_DATA_DONE: Configuration data was received from serial SPI encoder.
    'SER_DATA_DONE': Representation(23),
    # One of the SERIAL_ENC_Flags was set.
    'SERIAL_ENC_FLAG_SET': Representation(24),
    # COVER_DONE: SPI datagram was sent to the motor driver.
    'COVER_DONE': Representation(25),
    # ENC_VEL0: Encoder velocity has reached 0.
    'ENC_VEL0': Representation(26),
    # CL_MAX: Closed-loop commutation angle has reached maximum value.
    'CL_MAX': Representation(27),
    # CL_FIT: Closed-loop deviation has reached inner limit
    'CL_FIT': Representation(28),
    # STOP_ON_STALL: Motor stall detected. Motor ramp has stopped.
    'STOP_ON_STALL': Representation(29),
    # MOTOR_EV: One of the selected TMC motor driver flags was triggered.
    'MOTOR_EV': Representation(30),
    # RST_EV: Reset was triggered.
    'RST_EV': Representation(31)
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
    'USE_ASTART_AND_VSTART': Representation(0),
    # 0 Acceleration values are divided by CLK_FREQ.
    # 1 Acceleration values are set directly as steps per clock cycle.
    'DIRECT_ACC_VAL_EN': Representation(1),
    # 0 Bow values are calculated due to division by CLK_FREQ.
    # 1 Bow values are set directly as steps per clock cycle.
    'DIRECT_BOW_VAL_EN': Representation(2),
    # 0 STPOUT = 1 indicates an active step.
    # 1 STPOUT = 0 indicates an active step.
    'STEP_INACTIVE_POL': Representation(3),
    # 0 Only STPOUT transitions from inactive to active polarity indicate steps.
    # 1 Every level change of STPOUT indicates a step.
    'TOGGLE_STEP': Representation(4),
    # 0 DIROUT = 0 indicates negative direction.
    # 1 DIROUT = 1 indicates negative direction.
    'POL_DIR_OUT': Representation(5),
    # 0 Internal step control (internal ramp generator will be used)
    # 1 External step control via STPIN / DIRIN interface with high active steps at STPIN
    # 2 External step control via STPIN / DIRIN interface with low active steps at STPIN
    # 3 External step control via STPIN / DIRIN interface with toggling steps at STPIN
    'SDIN_MODE': Representation(6, 7),
    # 0 DIRIN = 0 indicates negative direction.
    # 1 DIRIN = 1 indicates negative direction.
    'POL_DIR_IN': Representation(8),
    # 0 STPIN/DIRIN input signals will manipulate internal steps at XACTUAL directly.
    # 1 STPIN/DIRIN input signals will manipulate XTARGET register value, the internal ramp generator is used.
    'SD_INDIRECT_CONTROL': Representation(9),
    # 0 An incremental encoder is connected to encoder interface.
    # 1 An absolute SSI encoder is connected to encoder interface.
    # 2 Reserved
    # 3 An absolute SPI encoder is connected to encoder interface
    'SERIAL_ENC_IN_MODE': Representation(10, 11),
    # 0 Differential encoder interface inputs enabled.
    # 1 Differential encoder interface inputs is disabled (automatically set for SPI encoder).
    'DIFF_ENC_IN_DISABLE': Representation(12),
    # 0 Standby signal becomes forwarded with an active low level at STDBY_CLK output.
    # 1 Standby signal becomes forwarded with an active high level at STDBY_CLK output.
    # 2 STDBY_CLK passes ChopSync clock (TMC23x, TMC24x stepper motor drivers only).
    # 3 Internal clock is forwarded to STDBY_CLK output pin.
    'STDBY_CLK_PIN_ASSIGNMENT': Representation(13, 14),
    # 0 INTR=0 indicates an active interrupt.
    # 1 INTR=1 indicates an active interrupt.
    'INTR_POL': Representation(15),
    # 0 TARGET_REACHED signal is set to 1 to indicate a target reached event.
    # 1 TARGET_REACHED signal is set to 0 to indicate a target reached event.
    'INVERT_POL_TARGET_REACHED': Representation(16),
    # 0 Clock gating is disabled.
    # 1 Internal clock gating is enabled.
    'CLK_GATING_EN': Representation(17),
    # 0 No clock gating during standby phase.
    # 1 Intenal clock gating during standby phase is enabled.
    'CLK_GATING_STDBY_EN': Representation(18),
    # 0 Fullstep switchover is disabled.
    # 1 SPI output forwards fullsteps, if |VACTUAL| > FS_VEL
    'FS_EN': Representation(19),
    # 0 No fullstep switchover for Step/Dir output is enabled.
    # 1 Fullsteps are forwarded via Step/Dir output also if fullstep operation is active.
    'FS_SDOUT': Representation(20),
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
    'PWM_OUT_EN': Representation(23),
    # 0 No encoder is connected to SPI output.
    # 1 SPI output is used as SSI encoder interface to forward absolute SSI encoder data.
    'SERIAL_ENC_OUT_ENABLE': Representation(24),
    # 0 Differential serial encoder output is enabled.
    # 1 Differential serial encoder output is disabled.
    'SERIAL_ENC_OUT_DIFF_DISABLE': Representation(25),
    # 0 VACTUAL=0 & AACTUAL=0 after switching off direct external step control.
    # 1 VACTUAL = VSTART and AACTUAL = ASTART after switching off direct external step control.
    'AUTOMATIC_DIRECT_SDIN_SWITCH_OFF': Representation(26),
    # 0 The register value of X_LATCH is forwarded at register 0x36.
    # 1 The register value of REV_CNT (#internal revolutions) is forwarded at register 0x36.
    'CIRCULAR_CNT_AS_XLATCH': Representation(27),
    # 0 The direction of the internal SinLUT is regularly used.
    # 1 The direction of internal SinLUT is reversed
    'REVERSE_MOTOR_DIR': Representation(28),
    # 0 INTR and TARGET_REACHED are outputs with strongly driven output values..
    # 1 INTR and TARGET_REACHED are used as outputs with gated pull-up and/or pull-down functionality.
    'INTR_TR_PU_PD_EN': Representation(29),
    # 0 INTR output function is used as Wired-Or in the case of intr_tr_pu_pd_en = 1.
    # 1 INTR output function is used as Wired-And. in the case of intr_tr_pu_pd_en = 1.
    'INTR_AS_WIRED_AND': Representation(30),
    # 0 TARGET_REACHED output function is used as Wired-Or in the case of intr_tr_pu_pd_en = 1.
    # 1 TARGET_REACHED output function is used as Wired-And in the case of intr_tr_pu_pd_en = 1.
    'TR_AS_WIRED_AND': Representation(31)
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
    'OPERATION_MODE': Representation(2)
  })

class ExternalClockFrequencyRegister(Register):
  REGISTER = 0x31
  bits = AttributeDict({
    # RW. External clock frequency fCLK; unsigned; 25 bits.
    # Set to proper Hz value which is defined by the external clock frequency
    # fCLK. Any value between fCLK = 4.2 MHz and 32 MHz can be selected.
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
    'TARGET_REACHED_F': Representation(0),
    # 1 POS_COMP_REACHED_F is set high if XACTUAL = POS_COMP
    'POS_COMP_REACHED_F': Representation(1),
    # 2 VEL_REACHED_F is set high if VACTUAL = |VMAX|
    'VEL_REACHED_F': Representation(2),
    # 4:3 VEL_STATE_F: Current velocity state: 0  VACTUAL = 0;
    #   1  VACTUAL > 0;
    #   2  VACTUAL < 0
    'VEL_STATE_F': Representation(3, 4),
    # 6:5 RAMP_STATE_F: Current ramp state: 0  AACTUAL = 0;
    #   1  AACTUAL increases (acceleration);
    #   2  AACTUAL decreases (deceleration)
    'RAMP_STATE_F': Representation(5, 6),
    # 7 STOPL_ACTIVE_F: Left stop switch is active.
    'STOPL_ACTIVE_F': Representation(7),
    # 8 STOPR_ACTIVE_F: Right stop switch is active.
    'STOPR_ACTIVE_F': Representation(8),
    # 9 VSTOPL_ACTIVE_F: Left virtual stop switch is active.
    'VSTOPL_ACTIVE_F': Representation(9),
    # 10 VSTOPR_ACTIVE_F: Right virtual stop switch is active.
    'VSTOPR_ACTIVE_F': Representation(10),
    # 11 ACTIVE_STALL_F: Motor stall is detected and VACTUAL > VSTALL_LIMIT.
    'ACTIVE_STALL_F': Representation(11),
    # 12 HOME_ERROR_F: HOME_REF input signal level is not equal to expected home level.
    'HOME_ERROR_F': Representation(12),
    # 13 FS_ACTIVE_F: Fullstep operation is active.
    'FS_ACTIVE_F': Representation(13),
    # 14 ENC_FAIL_F: Mismatch between XACTUAL and ENC_POS is out of tolerated range.
    'ENC_FAIL_F': Representation(14),
    # 15 N_ACTIVE_F: N event is active.
    'N_ACTIVE_F': Representation(15),
    # 16 ENC_LATCH_F: ENC_LATCH is rewritten.
    'ENC_LATCH_F': Representation(16),
    # 17 Applies to absolute encoders only:
    #    MULTI_CYCLE_FAIL_F indicates a failure during last multi cycle data evaluation.
    #    Applies to absolute encoders only:
    #    SER_ENC_VAR_F indicates a failure during last serial data evaluation due to a substantial
    #    deviation between two consecutive serial data values.
    # TODO '': Representation(),
    # 18 Reserved.
    # 19 CL_FIT_F: Active if ENC_POS_DEV < CL_TOLERANCE. The current mismatch
    #    between XACTUAL and ENC_POS is within tolerated range.
    'CL_FIT_F': Representation(19),
    # 23:20 Applies to absolute encoders only: SERIAL_ENC_FLAGS received from
    #   encoder. These flags are reset with a new encoder transfer request.
    'SERIAL_ENC_FLAGS': Representation(20, 23),
    # 24 TMC26x / TMC2130 only: SG: StallGuard2 status
    #    Optional for TMC24x only: Calculated stallGuard status.
    #    TMC23x / TMC24x only: UV_SF: Undervoltage flag.
    'SG_OR_UV_SF': Representation(24),
    # 25 All TMC motor drivers: OT: Overtemperature shutdown.
    'OT_SHUTDOWN_F': Representation(25),
    # 26 All TMC motor drivers: OTPW: Overtemperature warning.
    'OT_WARNING_F': Representation(26),
    # 27 TMC26x / TMC2130 only: S2GA: Short to ground detection bit for high side MOSFE of coil A.
    #    TMC23x / TMC24x only: OCA: Overcurrent bridge A.
    'S2G_A_OR_OC_A_F': Representation(27),
    # 28 TMC26x / TMC2130 only: S2GB: Short to ground detection bit for high side MOSFET of coil B.
    #    TMC23x / TMC24x only: OCB: Overcurrent bridge B.
    'S2G_B_OR_OC_B_F': Representation(28),
    # 29 All TMC motor drivers: OLA: Open load indicator of coil A.
    'OL_A_F': Representation(29),
    # 30 All TMC motor drivers: OLB: Open load indicator of coil B.
    'OL_B_F': Representation(30),
    # 31 TMC26x / TMC2130 only: STST: Standstill indicator.
    #    TMC23x / TMC24x only: OCHS: Overcurrent high side.
    'STST_OR_OCHS_F': Representation(31)
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
    'THREE_PHASE_STEPPER_EN': Representation(4),
    # 4 (No TMC driver)
    #   0 NSCSDRV_SDO is tied low before SCKDRV_NSDO to initiate a new data transfer.
    #   1 SCKDRV_NSDO is tied low before NSCSDRV_SDO to initiate a new data transfer.
    'SCK_LOW_BEFORE_CSN': Representation(4),
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
    'SCALE_VAL_TRANSFER_EN': Representation(5),
    # 5 (No TMC driver)
    #   0 New value bit at SDODRV_SCLK is assigned at falling edge of SCKDRV_NSDO.
    #   1 New value bit at SDODRV_SCLK is assigned at rising edge of SCKDRV_NSDO.
    'NEW_OUT_BIT_AT_RISE': Representation(5),
    # 6 (TMC24x only)
    #   0 No standby datagram is sent.
    #   1 In case of a Stop-on-Stall event, a standby datagram is sent to the TMC24x.
    'STDBY_ON_STALL_FOR_24X': Representation(6),
    # 6 (TMC26x/2130 in SD mode only)
    #   0 Permanent transfer of polling datagrams to check driver status.
    #   1 No transfer of polling datagrams.
    'DISABLE_POLLING': Representation(6),
    # 7 (TMC24x only)
    #   0 Undervoltage flag of TMC24x is mapped at STATUS(24).
    #   1 Calculated stall status of TMC24x is forwarded at STATUS(24).
    'STALL_FLAG_INSTEAD_OF_UV_EN': Representation(7),
    # 7 (TMC26x/2130 only)
    #   0 No automatic continuous streaming of cover datagrams.
    #   1 Enabling of automatic continuous streaming of cover datagrams.
    'AUTOREPEAT_COVER_EN': Representation(7),
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
    'PWM_PHASE_SHFT_EN': Representation(11),
    # 12 (TMC23x/24x only)
    #   0 ChopSync frequency remains stable during standby.
    #   1 CHOP_SYNC_DIV is halfed during standby.
    'DOUBLE_FREQ_AT_STDBY': Representation(12),
    # 12 (TMC26x/2130 only)
    #   0 COVER_DONE event is set for every datagram that is sent to the motor driver.
    #   1 COVER_DONE event is only set for cover datagrams sent to the motor driver.
    'COVER_DONE_ONLY_FOR_COVER': Representation(12),
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

class CurrentScalingConfRegister(Register):
  REGISTER = 0x05
  bits = AttributeDict({
    # 0 hold_current_scale_en
    #   0 No hold current scaling during standstill phase.
    #   1 Hold current scaling during standstill phase.
    'HOLD_CURRENT_SCALE_EN': Representation(0),
    # 1 drive_current_scale_en
    #   0 No drive current scaling during motion.
    #   1 Drive current scaling during motion.
    'DRIVE_CURRENT_SCALE_EN': Representation(1),
    # 2 boost_current_on_acc_en
    #   0 No boost current scaling for deceleration ramps.
    #   1 Boost current scaling if RAMP_STATE = b’01 (acceleration slopes).
    'BOOST_CURRENT_ON_ACC_EN': Representation(2),
    # 3 boost_current_on_dec_en
    #   0 No boost current scaling for deceleration ramps.
    #   1 Boost current scaling if RAMP_STATE = b’10 (deceleration slopes).
    'BOOST_CURRENT_ON_DEC_EN': Representation(3),
    # 4 boost_current_after_start_en
    #   0 No boost current at ramp start.
    #   1 Temporary boost current if VACTUAL = 0 and new ramp starts.
    'BOOST_CURRENT_AFTER_START_EN': Representation(4),
    # 5 sec_drive_current_scale_en
    #   0 One drive current value for the whole motion ramp.
    #   1 Second drive current scaling for VACTUAL > VDRV_SCALE_LIMIT.
    'SEC_DRIVE_CURRENT_SCALE_EN': Representation(5),
    # 6 freewheeling_en
    #   0 No freewheeling.
    #   1 Freewheeling after standby phase.
    'FREEWHEELING_EN': Representation(6),
    # 7 closed_loop_scale_en
    #   0 No closed-loop current scaling.
    #   1 Closed-loop current scaling – CURRENT_CONF(6:0) = 0 is set automatically
    #     Turn off for closed-loop calibration with maximum current!
    'CLOSED_LOOP_SCALE_EN': Representation(7),
    # 8 pwm_scale_en
    #   0 PWM scaling is disabled.
    #   1 PWM scaling is enabled.
    'PWM_SCALE_EN': Representation(8),
    # 15:9 Reserved. Set to 0x00.
    # 31:16 PWM_AMPL
    # PWM amplitude during Voltage PWM mode at VACTUAL = 0.
    # i Maximum duty cycle = (0.5 + (PWM_AMPL + 1) / 217)
    # Minimum duty cycle = (0.5 – (PWM_AMPL + 1) / 2
    # PWM_AMPL = 216 – 1 at VACTUAL = PWM_VMAX.
    'PWM_AMPL': Representation(16, 31)
  })

class InputFilterRegister(Register):
  REGISTER = 0x3
  # TODO finish this

class CurrentScaleValuesRegister(Register):
  REGISTER = 0x06
  DEFAULT = 0xFFFFFFFF
  # TODO finish this class
  bits = AttributeDict({
    # RW  Open-loop standby scaling value.
    'HOLD_SCALE_VAL': Representation(24, 31)
  })

class StandbyDelayRegister(Register):
  REGISTER = 0x15
  bits = AttributeDict({
    # RW. 31:0 STDBY_DELAY(Default: 0x00000000)
    # Delay time [# clock cycles] between ramp stop and activating standby phase.
    'STDBY_DELAY': Representation(0, 31)
  })

class FreewheelDelayRegister(Register):
  REGISTER = 0x16
  bits = AttributeDict({
    # RW. 31:0 FREEWHEEL_DELAY (Default:0x00000000)
    # Delay time [# clock cycles] between initialization of active standby phase and
    # freewheeling initialization.
    'FREEWHEEL_DELAY': Representation(0, 31)
  })

class VDRVScaleLimitRegister(Register):
  REGISTER = 0x17
  bits = AttributeDict({
    # 23: 0 VDRV_SCALE_LIMIT(Default: 0x000000)
    # (Voltage PWM mode is not active)
    # Drive scaling separator:
    # DRV2_SCALE_VAL is active in case VACTUAL > VDRV_SCALE_LIMIT
    # DRV1_SCALE_VAL is active in case VACTUAL ≤ VDRV_SCALE_LIMIT
    # 2nd assignment: Also used as PWM_VMAX if Voltage PWM is enabled(see 19.17. )
    'VDRV_SCALE_LIMIT': Representation(0, 23)
  })

class UpScaleDelayRegister(Register):
  REGISTER = 0x18
  bits = AttributeDict({
    # RW 23:0
    # UP_SCALE_DELAY(Default: 0x000000) (Open - loop operation)
    #   Increment delay[  # clock cycles]. The value defines the clock cycles, which are used to increase the current
    #   scale value for one step towards higher values.
    # CL_UPSCALE_DELAY (Default:0x000000) (Closed - loop operation)
    #   Increment delay[  # clock cycles]. The value defines the clock cycles, which are used to increase the current
    #   scale value for one step towards higher current values during closed-loop operation.
    'UP_SCALE_DELAY': Representation(0, 23),
    'CL_UPSCALE_DELAY': Representation(0, 23)
  })

class HoldScaleDelayRegister(Register):
  REGISTER = 0x19
  bits = AttributeDict({
    # 23: 0
    # HOLD_SCALE_DELAY(Default: 0x000000) (Open - loop operation)
    #   Decrement delay[  # clock cycles] to decrease the actual scale value by one step towards hold current.
    # CL_DNSCALE_DELAY(Default: 0x000000) (Closed - loop operation)
    #   Decrement delay[  # clock cycles] to decrease the current scale value by one step towards lower current values during closed - loop operation.
    'HOLD_SCALE_DELAY': Representation(0, 23),
    'CL_DNSCALE_DELAY': Representation(0, 23)
  })

class DriveScaleDelayRegister(Register):
  REGISTER = 0x1A
  bits = AttributeDict({
    # 23:0 DRV_SCALE_DELAY(Default: 0x000000)
    # Decrement delay[  # clock cycles], which signifies current scale value decrease by one step towards lower value.
    'DRV_SCALE_DELAY': Representation(0, 23)
  })

class BoostTimeRegister(Register):
  REGISTER = 0x1B
  bits = AttributeDict({
    # RW 31:0
    # BOOST_TIME(Default: 0x00000000)
    # Time[  # clk cycles] after a ramp start when boost scaling is active.
    'BOOST_TIME': Representation(0, 31)
  })

class BetaGammaRegister(Register):
  REGISTER = 0x1C
  bits = AttributeDict({
    # 8:0 CL_BETA(0x0FF)
    # Maximum commutation angle for closed - loop regulation.
    # - Set CL_BETA > 255 carefully(esp. if cl_vlimit_en = 1).
    # - Exactly 255 is recommended for best performance.
    'CL_BETA': Representation(0, 8),
    # 23:16
    # CL_GAMMA(Default: 0xFF)
    # Maximum balancing angle to compensate back - EMF at higher velocities during closed - loop regulation.
    'CL_GAMMA': Representation(16, 23)
  })

class SpiDacAddressRegister(Register):
  REGISTER = 0x1D
  bits = AttributeDict({
    # 15:0 DAC_ADDR_A(Default: 0x0000)
    # Fixed command / address, which is sent via SPI output before sending CURRENTA_SPI values.
    'DAC_ADDR_A': Representation(0, 15),
    # 31:16 DAC_ADDR_B(Default: 0x0000)
    # Fixed command / address, which is sent via SPI output before sending current CURRENTB_SPI values.
    'DAC_ADDR_B': Representation(16, 31),
    # 23:0 2nd assignment: Also used as SPI_SWITCH_VEL if SPI - DAC mode is disabled(19.16.)
    'SPI_SWITCH_VEL': Representation(0, 23)
  })

class FullStepVelocityRegister(Register):
  REGISTER = 0x60
  bits = AttributeDict({
    # 31:0 FS_VEL(Default:0x000000) (Closed-loop and dcStep operation are disabled)
    # Minimum fullstep velocity [pps].
    # In case |VACTUAL| > FS_VEL fullstep operation is active, if enabled.
    # 2nd assignment: Also used as DC_VEL if dcStep is enabled (see section 19.27. )
    # 3rd assignment: Also used as CL_VMIN_EMF if closed-loop is enabled (see 19.26. )
    # "Set DC_VEL register 0x60 as threshold velocity value[pps] at which dcStep is activated."
    'FS_VEL': Representation(0, 31)
  })

class MSLUT0Register(Register):
  REGISTER = 0x70
  bits = AttributeDict({
    # W. 31:0 MSLUT[0](Default: 0xAAAAB554)
    # Each bit defines the difference between consecutive values in the microstep look - up table MSLUT( in combination with MSLUTSEL).
    'MSLUT0': Representation(0, 31)
  })

class MSLUT1Register(Register):
  REGISTER = 0x71
  bits = AttributeDict({
    # W. 31:0 MSLUT[1](Default: 0x4A9554AA)
    # Each bit defines the difference between consecutive values in the microstep look - up table MSLUT( in combination with MSLUTSEL).
    'MSLUT1': Representation(0, 31)
  })

class MSLUT2Register(Register):
  REGISTER = 0x72
  bits = AttributeDict({
    # W. 31:0 MSLUT[2](Default: 0x24492929)
    # Each bit defines the difference between consecutive values in the microstep look - up table MSLUT( in combination with MSLUTSEL).
    'MSLUT2': Representation(0, 31)
  })

class MSLUT3Register(Register):
  REGISTER = 0x73
  bits = AttributeDict({
    # W. 31:0 MSLUT[3](Default: 0x10104222)
    # Each bit defines the difference between consecutive values in the microstep look - up table MSLUT( in combination with MSLUTSEL).
    'MSLUT3': Representation(0, 31)
  })

class MSLUT4Register(Register):
  REGISTER = 0x74
  bits = AttributeDict({
    # W. 31:0 MSLUT[4](Default: 0xFBFFFFFF)
    # Each bit defines the difference between consecutive values in the microstep look - up table MSLUT( in combination with MSLUTSEL).
    'MSLUT4': Representation(0, 31)
  })

class MSLUT5Register(Register):
  REGISTER = 0x75
  bits = AttributeDict({
    # W. 31:0 MSLUT[5](Default: 0xB5BB777D)
    # Each bit defines the difference between consecutive values in the microstep look - up table MSLUT( in combination with MSLUTSEL).
    'MSLUT5': Representation(0, 31)
  })

class MSLUT6Register(Register):
  REGISTER = 0x76
  bits = AttributeDict({
    # W. 31:0 MSLUT[6](Default: 0x49295556)
    # Each bit defines the difference between consecutive values in the microstep look - up table MSLUT( in combination with MSLUTSEL).
    'MSLUT6': Representation(0, 31)
  })

class MSLUT7Register(Register):
  REGISTER = 0x77
  bits = AttributeDict({
    # W. 31:0 MSLUT[7](Default: 0x00404222)
    # Each bit defines the difference between consecutive values in the microstep look - up table MSLUT( in combination with MSLUTSEL).
    'MSLUT7': Representation(0, 31)
  })

class MSLUTSelectRegister(Register):
  REGISTER = 0x78
  bits = AttributeDict({
    # W. 31:0 MSLUTSEL(Default: 0xFFFF8056)
    # Definition of the four segments within each quarter MSLUT wave.
    'MSLUTSEL': Representation(0, 31)
  })

class MicrostepCountRegister(Register):
  REGISTER = 0x79
  bits = AttributeDict({
    # R 9:0 MSCNT(Default: 0x000)
    # Actual µStep position of the sine value.
    # W 2nd assignment: Also used as MS_OFFSET if Voltage PWM is enabled(see 19.17. )
    'MSCNT': Representation(0, 9),
    'MS_OFFSET': Representation(0, 9)
  })

class StartSineRegister(Register):
  REGISTER = 0x7E
  bits = AttributeDict({
    # W 7:0 START_SIN(Default: 0x00)
    # Start value for sine waveform.
    'START_SIN': Representation(0, 7),
    # W 23:16 START_SIN90_120(Default: 0xF7)
    # Start value for cosine waveform.
    'START_SIN90_120': Representation(16, 23),
    # W 31:24GEAR_RATIO
    # 2nd assignment: Also used as DAC_OFFSET for write access(see section 19.30.)
    'DAC_OFFSET': Representation(24, 31)
  })

class GearRatioRegister(Register):
  REGISTER = 0x12
  bits = AttributeDict({
    # 31:0 GEAR_RATIO(Default: 0x01000000)
    # Constant value that is added to the internal position counter by an active step at STPIN.
    # Value representation: 8 digits and 24 decimal places.
    'GEAR_RATIO': Representation(0, 31, 8, 24)
  })

# TODO add these registers
# 0x6c COVER_LOW POLLING_STATUS
# 0x6d COVER_HIGH POLLING_REG

class VirtualStopLeftRegister(Register):
  REGISTER = 0x33
  DEFAULT = 0x00000000
  bits = AttributeDict({
    # RW 0x33 31:0 (Default: 0x00000000)
    # Virtual left stop position.
    'VIRT_STOP_LEFT': Representation(0, 31)
  })

class VirtualStopRightRegister(Register):
  REGISTER = 0x34
  DEFAULT = 0x00000000
  bits = AttributeDict({
    # RW 0x34 31:0 (Default: 0x00000000)
    # Virtual left right position.
    'VIRT_STOP_RIGHT': Representation(0, 31)
  })
