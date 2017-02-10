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

def parse_ini(path):
  """
  Parses ini files exported from TMCL-IDE.
  """
  with open(path) as f:
    for line in f.readlines():
      # remove tabs and comments
      split = line.replace('\t', '').replace(' ', '').split(';')
      if not len(split[0]) or split[0][0] != '0':
        continue
      key, value = split[0].split('=')
      register_code, register_value = int(key, 16), int(value, 16)
      yield (register_code, register_value)