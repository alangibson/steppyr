from steppyr.lib.bits import mask, number_to_fixed, get_bits, decode_twos_complement, fixed_to_number, set_bit, \
  unset_bit, datagram_to_int

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

class Representation:
  """
  Value Representation

  Valid use:
  Representation(5) = _BV(5)
  Representation(0, 10) = mask(0, 10)
  Representation(0, 15, 16) = unsigned 16bit int in register bits 0:15
  Representation(0, 15, 8, 8, True) = signed (twos-compliment) 8bit.8bit float
    in register bits 0:15

  Fixed Point Math on the Arduino Platform
    https://ucexperiment.wordpress.com/2012/10/28/fixed-point-math-on-the-arduino-platform/
  """
  def __init__(self, first_bit, last_bit=None, whole_bits=None, fractional_bits=0, signed=False):
    """
    first_bit: lsb
    last_bit: msb
    """
    self._first_bit = first_bit
    if last_bit == None:
      # When last_bit == None, we are acting like _BV
      last_bit = first_bit
    self._last_bit = last_bit
    self.bitmask = mask(first_bit, last_bit)
    if whole_bits == None:
      # When whole_bits == None, we are acting like mask() or _BV()
      self._whole_bits = last_bit + 1 # first/last_bit is 0 indexed
    else:
      self._whole_bits = whole_bits
    self._fractional_bits = fractional_bits
    self._signed = signed

  def to_register_value(self, number):
    """
    Returns number encoded so that it can be directly ORed onto register data.
    1. Convert floating point to fixed point. Int remains unchanged.
    2. TODO Convert fixed point to twos-compliment. Positive remains unchanged.
    3. Left shift until lsb is same index as first_bit.
    """
    # Convert to fixed point
    value = number_to_fixed(number, self._fractional_bits)
    # Left shift until lsb is same index as first_bit.
    value = value << self._first_bit
    return value

  def from_register_value(self, register_value):
    # Apply bitmask to extract bits from encoded register value
    value = get_bits(register_value, self.bitmask)
    # FIXME handle case where self._last_bit > whole_bits + fractional_bits
    # test case: test_mismatch
    # Handle signed
    if self._signed:
      value = decode_twos_complement(value, self._whole_bits + self._fractional_bits)
    # Convert fixed point to floating point or int
    return fixed_to_number(value, self._fractional_bits)

class Datagram:

  def __init__(self, header, data, header_len, datagram_len):
    """
    header: status or register int
    data: message data
    """
    self._header = header
    if hasattr(data, '__iter__'):
      self._data = datagram_to_int(data)
    else:
      self._data = data
    self._header_len = header_len
    self._datagram_len = datagram_len

  @property
  def datagram(self):
    return (self._header << (self._datagram_len - self._header_len)) | self._data

  @property
  def header(self):
    return self._header

  @property
  def data(self):
    return self._data

  @property
  def register(self):
    return self._header

  def to_list(self):
    """
    Returns entire datagram (header+data) as a list of 8-bit ints.
    """
    raise NotImplementedError()

  def as_response(self, datagram_list):
    """
    Clone ourselves as a response.
    """
    return type(self)(datagram_to_int(datagram_list))

class Register:
  """Represents a memory register in an Trinamic chip"""

  def __init__(self, data, header, header_len, datagram_len):
    if header == None:
      self._header = self.REGISTER
    else:
      self._header = header
    self._data = data
    self._header_len = header_len
    self._datagram_len = datagram_len

  def set(self, representation, value=None):
    """
    value: (int or float) If float, a Representation is needed to encode to
           valid register value.
    bitmask can be
    1. integer bitmask (or really any integer)
    # Disabled: 2. dict like: {bitmask: mask(0, 30), representation: Representation(23, 8, False)}
    """
    if value == None:
      # If we have no value, just apply the bitmask. Usefull for setting bit flags.
      self._data = set_bit(self._data, representation.bitmask)
    else:
      # Encode value to register value using Representation
      value = representation.to_register_value(value)
      # Apply value at bitmask bits to register data
      self._data = set_bit(unset_bit(self._data, representation.bitmask), value)
    return self

  def unset(self, representation):
    self._data = unset_bit(self._data, representation.bitmask)
    return self

  def get(self, representation):
    return representation.from_register_value(self._data)

  def get_values(self):
    events = []
    for name, representation in self.bits.items():
      value = self.get(representation)
      if value:
        events.append((name, value))
    return events

