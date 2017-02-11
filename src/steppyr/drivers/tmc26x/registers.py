from steppyr.lib import AttributeDict
from steppyr.lib.trinamic import Representation, Register as TrinamicRegister
from .io import Datagram

# definitions for the driver control (DRVCTRL) register
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

class Register(TrinamicRegister, Datagram):

  def __init__(self, data=0, header=None, header_len=3, datagram_len=20):
    super().__init__(data=data, header=header, header_len=header_len, datagram_len=datagram_len)

class DriverControlRegister(Register):
  """
  DRVCTRL Register
  """
  REGISTER = 0b00
  bits = AttributeDict({
    # 17 PHA (SPI Mode SDOFF=1)
    # Polarity A Sign of current flow through coil A:
    #   0: Current flows from OA1 pins to OA2 pins.
    #   1: Current flows from OA2 pins to OA1 pins.
    # 17:10 0 (STEP/DIR Mode SDOFF=0) Reserved
    'PHA': Representation(17),
    # 16:9 CA7-CA0 (SPI Mode SDOFF=1)
    # Current A MSB Magnitude of current flow through coil A.
    #   The range is 0 to 248, if hysteresis or offset are used up to their full
    #   extent. The resulting value after applying hysteresis or
    #   offset must not exceed 255.
    'CA': Representation(9, 16),
    # 9 INTPOL (STEP/DIR Mode SDOFF=0)
    #   Enable STEP interpolation
    #   0: Disable STEP pulse interpolation.
    #   1: Enable STEP pulse multiplication by 16.
    'INTPOL': Representation(9),
    # 8 PHB (SPI Mode SDOFF=1)
    # Polarity B Sign of current flow through coil B:
    #   0: Current flows from OB1 pins to OB2 pins.
    #   1: Current flows from OB2 pins to OB1 pins.
    'PHB': Representation(8),
    # 8 DEDGE (STEP/DIR Mode SDOFF=0)
    #   Enable double edge STEP pulses
    #   0: Rising STEP pulse edge is active, falling edge is inactive.
    #   1: Both rising and falling STEP pulse edges are active.
    'DEDGE': Representation(8),
    # 7:0 CB7 (SPI Mode SDOFF=1)
    # Current B MSB Magnitude of current flow through coil B. The range is
    #   0 to 248, if hysteresis or offset are used up to their full
    #   extent. The resulting value after applying hysteresis or
    #   offset must not exceed 255.
    # 7:4 0 (STEP/DIR Mode SDOFF=0) Reserved
    'CB': Representation(0, 7),
    # 3:0 MRES (STEP/DIR Mode SDOFF=0)
    # Microstep resolution for STEP/DIR mode
    #   Microsteps per 90°:
    #   %0000: 256
    #   %0001: 128
    #   %0010: 64
    #   %0011: 32
    #   %0100: 16
    #   %0101: 8
    #   %0110: 4
    #   %0111: 2 (halfstep)
    #   %1000: 1 (fullstep)
    'MRES': Representation(0, 3)
  })
  def __init__(self, data=0, header=None,
               header_len=2, datagram_len=20):
    """
    Overriding just to set header_len=2
    """
    super().__init__(data=data, header=header,
                     header_len=header_len, datagram_len=datagram_len)

class ChopperControllRegister(Register):
  """
  Chopper Control Register (CHOPCONF)
  """
  REGISTER = 0b100
  bits = AttributeDict({
    # 16:15 TBL Blanking time Blanking time interval, in system clock periods:
    #   %00: 16
    #   %01: 24
    #   %10: 36
    #   %11: 54
    'TB': Representation(15, 16),
    # 14 CHM Chopper mode This mode bit affects the interpretation of the HDEC,
    #   HEND, and HSTRT parameters shown below.
    #   0 Standard mode (spreadCycle)
    #   1 Constant tOFF with fast decay time.
    #     Fast decay time is also terminated when the
    #     negative nominal current is reached. Fast
    #     decay is after on time.
    'CHM': Representation(14),
    # 13 RNDTF Random TOFF time Enable randomizing the slow decay phase duration:
    #   0: Chopper off time is fixed as set by bits tOFF
    #   1: Random mode, tOFF is random modulated by [...]
    'RNDTF': Representation(13),
    # 12:11 HDEC Hysteresis decrement interval or Fast decay mode
    #   CHM=0 Hysteresis decrement period setting, in system clock periods:
    #     %00: 16
    #     %01: 32
    #     %10: 48
    #     %11: 64
    #   CHM=1 HDEC1=0: current comparator can terminate
    #     the fast decay phase before timer expires.
    #     HDEC1=1: only the timer terminates the fast
    #     decay phase.
    #     HDEC0: MSB of fast decay time setting.
    'HDEC': Representation(11, 12),
    # 10:7 HEND Hysteresis end (low) value or
    #   Sine wave offset
    #   CHM=0 %0000 … %1111:
    #     Hysteresis is -3, -2, -1, 0, 1, …, 12
    #     (1/512 of this setting adds to current setting)
    #     This is the hysteresis value which becomes
    #     used for the hysteresis chopper.
    #   CHM=1 %0000 … %1111:
    #     Offset is -3, -2, -1, 0, 1, …, 12
    #     This is the sine wave offset and 1/512 of the
    #     value becomes added to the absolute value
    #     of each sine wave entry.
    'HEND': Representation(7, 10),
    # 6:4 HSTRT Hysteresis start value or
    #   Fast decay time
    #   setting
    #   CHM=0 Hysteresis start offset from HEND:
    #     %000: 1 %100: 5
    #     %001: 2 %101: 6
    #     %010: 3 %110: 7
    #     %011: 4 %111: 8
    #   Effective: HEND+HSTRT must be ≤ 15
    #   CHM=1 Three least-significant bits of the duration of
    #   the fast decay phase. The MSB is HDEC0.
    #   Fast decay time is a multiple of system clock
    #   periods: NCLK= 32 x (HDEC0+HSTRT)
    'HSTRT': Representation(4, 6),
    # 3:0 TOFF Off time/MOSFET disable
    #   Duration of slow decay phase. If TOFF is 0, the MOSFETs
    #   are shut off. If TOFF is nonzero, slow decay time is a
    #   multiple of system clock periods:
    #   NCLK= 12 + (32 x TOFF) (Minimum time is 64clocks.)
    #     %0000: Driver disable, all bridges off
    #     %0001: 1 (use with TBL of minimum 24 clocks)
    #     %0010 … %1111: 2 … 15
    'TOFF': Representation(0, 3)
  })

class CoolStepControlRegister(Register):
  """
  coolStep Control Register (SMARTEN)
  """
  REGISTER = 0b101
  bits = AttributeDict({
    # 15 SEIMIN Minimum coolStep current
    #   0: 1/2 CS current setting
    #   1: 1/4 CS current setting
    'SEIMIN': Representation(15),
    # 14:13 SEDN Current decrement speed
    #   Number of times that the stallGuard2 value must be
    #   sampled equal to or above the upper threshold for each
    #   decrement of the coil current:
    #     %00: 32
    #     %01: 8
    #     %10: 2
    #     %11: 1
    'SEDN': Representation(13, 14),
    # 12 0 Reserved
    # 11:8 SEMAX Upper coolStep threshold as an offset from the lower threshold
    #   If the stallGuard2 measurement value SG is sampled
    #   equal to or above (SEMIN+SEMAX+1) x 32 enough times,
    #   then the coil current scaling factor is decremented.
    'SEMAX': Representation(8, 11),
    # 7 0 Reserved
    # 6:5 SEUP Current increment size
    #   Number of current increment steps for each time that
    #   the stallGuard2 value SG is sampled below the lower
    #   threshold:
    #     %00: 1
    #     %01: 2
    #     %10: 4
    #     %11: 8
    'SEUP': Representation(5, 6),
    # 4 0 Reserved
    # 3:0 SEMIN Lower coolStep threshold/coolStep disable
    #   If SEMIN is 0, coolStep is disabled. If SEMIN is nonzero
    #   and the stallGuard2 value SG falls below SEMIN x 32,
    #   the coolStep current scaling factor is increased.
    'SEMIN': Representation(0, 3)
  })

class StallGuard2ControlRegister(Register):
  """
  stallGuard2 Control Register (SGCSCONF)
  """
  REGISTER = 0b110
  bits = AttributeDict({
    # 16 SFILT stallGuard2 filter enable
    #   0: Standard mode, fastest response time.
    #   1: Filtered mode, updated once for each four fullsteps to compensate for
    #      variation in motor construction, highest accuracy.
    'SFILT': Representation(16),
    # 15 0 Reserved
    # 14:8 SGT stallGuard2 threshold value
    #   The stallGuard2 threshold value controls the optimum
    #   measurement range for readout. A lower value results in
    #   a higher sensitivity and requires less torque to indicate
    #   a stall. The value is a two’s complement signed integer.
    #   Values below -10 are not recommended.
    #   Range: -64 to +63
    'SGT': Representation(8, 14),
    # 7:5 0 Reserved
    # 4:0 CS Current scale (scales digital currents A and B)
    #   Current scaling for SPI and step/direction operation.
    #   %00000 … %11111: 1/32, 2/32, 3/32, … 32/32
    #   This value is biased by 1 and divided by 32, so the
    #   range is 1/32 to 32/32.
    #   Example: CS=0 is 1/32 current
    'CS': Representation(0, 4)
  })

class DriverConfigRegister(Register):
  """
  Driver Config Register (DRVCONF)
  """
  REGISTER = 0b111
  bits = AttributeDict({
    # 16 TST Reserved TEST mode
    #   Must be cleared for normal operation. When set, the
    #   SG_TST output exposes digital test values, and the
    #   TEST_ANA output exposes analog test values. Test value
    #   selection is controlled by SGT1 and SGT0:
    #   TEST_ANA:
    #     %00: anatest_2vth,
    #     %01: anatest_dac_out,
    #     %10: anatest_vdd_half.
    #   SG_TST: %00: comp_A,
    #     %01: comp_B,
    #     %10: CLK,
    #     %11: on_state_xy
    'SG_TST': Representation(16),
    # 15:14 SLPH Slope control, high side
    #   %00: Minimum
    #   %01: Minimum temperature compensation mode.
    #   %10: Medium temperature compensation mode.
    #   %11: Maximum
    #   In temperature compensated mode (tc), the MOSFET gate
    #   driver strength is increased if the overtemperature
    #   warning temperature is reached. This compensates for
    #   temperature dependency of high-side slope control.
    'SLPH': Representation(14, 15),
    # 13:12 SLPL Slope control, low side
    #   %00: Minimum.
    #   %01: Minimum.
    #   %10: Medium.
    #   %11: Maximum.
    'SLPL': Representation(12, 13),
    # 11 0 Reserved
    # 10 DISS2G Short to GND protection disable
    #   0: Short to GND protection is enabled.
    #   1: Short to GND protection is disabled.
    'DISS2G': Representation(10),
    # 9:8 TS2G Short to GND detection timer
    #   %00: 3.2µs.
    #   %01: 1.6µs.
    #   %10: 1.2µs.
    #   %11: 0.8µs.
    'TS2G': Representation(8, 9),
    # 7 SDOFF STEP/DIR interface disable
    #   0: Enable STEP and DIR interface.
    #   1: Disable STEP and DIR interface. SPI interface is used to move motor.
    'SDOFF': Representation(7),
    # 6 VSENSE Sense resistor voltage-based current scaling
    #   0: Full-scale sense resistor voltage is 305mV.
    #   1: Full-scale sense resistor voltage is 165mV.
    #   (Full-scale refers to a current setting of 31 and a DAC value of 255.)
    'VSENSE': Representation(6),
    # 5:4 RDSEL Select value for read out (RD bits)
    #   %00 Microstep position read back
    #   %01 stallGuard2 level read back
    #   %10 stallGuard2 and coolStep current level read back
    #   %11 Reserved, do not use
    'RDSEL': Representation(4, 5)
    # 3:0 Reserved
  })
