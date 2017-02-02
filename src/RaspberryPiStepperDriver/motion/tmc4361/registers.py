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
