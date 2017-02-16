# Microstep resolution for STEP/DIR mode.
# Microsteps per 90°
MICROSTEP_RESOLUTION = {
  256:  0b0000,
  128:  0b0001,
  64:   0b0010,
  32:   0b0011,
  16:   0b0100,
  4:    0b0110,
  8:    0b0101,
  2:    0b0111,  # halfstep
  1:    0b1000   # fullstep
}
