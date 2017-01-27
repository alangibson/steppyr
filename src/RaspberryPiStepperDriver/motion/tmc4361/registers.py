"""
registers for TMC4361
"""
from . import _BV

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
TMC4361_SPIOUT_CONF_REGISTER = 0x04
TMC4361_ENCODER_INPUT_CONFIG_REGISTER = 0x07
TMC4361_STEP_CONF_REGISTER = 0x0A
TMC4361_EVENT_CLEAR_CONF_REGISTER = 0x0c
TMC4361_INTERRUPT_CONFIG_REGISTER = 0x0d
TMC4361_EVENTS_REGISTER = 0x0e
TMC4361_STATUS_REGISTER = 0x0f
TMC4361_START_OUT_ADD_REGISTER = 0x11
TMC4361_GEAR_RATIO_REGISTER = 0x12
TMC4361_START_DELAY_REGISTER = 0x13

# Ramp Generator Registers
# RW. Ramp operation mode and motion profile.
# Bit 2: Operation Mode:
#   1 Positioning mode: XTARGET is superior target of velocity ramp.
#   0 Velocitiy mode: VMAX is superior target of velocity ramp.
# Bit 1:0: Motion Profile:
#   0: No ramp: VACTUAL follows only VMAX (rectangle velocity shape).
#   1: Trapezoidal ramp (incl. sixPoint ramp): Consideration of acceleration and deceleration values for generating VACTUAL without adapting the acceleration values.
#   2: S-shaped ramp: Consideration of all ramp values (incl. bow values) for generating VACTUAL.
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

TMC4361_ENCODER_POSITION_REGISTER = 0x50
TMC4361_ENCODER_INPUT_RESOLUTION_REGISTER = 0x54
TMC4361_COVER_LOW_REGISTER = 0x6c
TMC4361_COVER_HIGH_REGISTER = 0x6d
TMC4361_SPI_STATUS_SELECTION_REGISTER = 0x0B

# how to mask REFERENCE_CONFIG_REGISTER if you want to configure just one end
TMC4361_LEFT_ENDSTOP_REGISTER_PATTERN = (_BV(0) | _BV(2) | _BV(6) | _BV(10) | _BV(11) | _BV(14))
TMC4361_RIGHT_ENDSTOP_REGISTER_PATTERN = (_BV(1) | _BV(3) | _BV(7) | _BV(12) | _BV(13) | _BV(15))
