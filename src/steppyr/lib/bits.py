import functools

def unset_bit(value, mask):
  """ Convenience function to unset bits based on a mask """
  new_value = value & ~ mask
  #log.debug('unsetting bit(s) %s %s -> %s', bin(value), bin(mask), bin(new_value))
  return new_value

def set_bit(value, mask):
  """ Convenience function to set bits based on a mask """
  new_value = value | mask
  #log.debug('setting bit(s) %s %s -> %s', bin(value), bin(mask), bin(new_value))
  return new_value

def get_bits(value, mask):
  """
  Returns the bits from value that are masked by mask.
  """
  return (value & mask) >> lsb(mask)

def clear_bit(value, bit):
  """
  Clear a bit, aka set it to 0
  """
  return value & ~(1<<bit)

def mask(begin, end):
  return functools.reduce(lambda a,b: a+b, [2**i for i in range(begin, end+1)])

def _BV(bit):
  """
  An implementation of the Arduino _BV macro.
  http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_use_bv.html
  """
  return 1 << bit

def lsb(x):
  """
  Returns the index, counting from 0, of the least significant set bit in `x`.
  http://stackoverflow.com/questions/5520655/return-index-of-least-significant-bit-in-python
  """
  return ( x & -x).bit_length() - 1

def bits(n):
  """
  Returns position of all bits = 1.
  http://stackoverflow.com/questions/8898807/pythonic-way-to-iterate-over-bits-of-integer
  """
  while n:
    b = n & (~n+1)
    yield b
    n ^= b

def tobin(x, n=8):
  """
  Returns a space separated string of bin representing byte input.
  Accepts numbers and iterables.
  """
  if not hasattr(x, '__iter__'):
    x = [x]
  out = ''
  for i in x:
    out += (format(i, 'b').ljust(n, '0')) + ' '
  return out

def datagram_to_int(datagram):
  """
  Reduce a list of bytes or ints to a single int
  """
  value = 0
  for b in datagram:
    value = value << 8
    value |= b
  # Support twos-compliment negative integers
  bits = len(datagram) * 8
  # if sign bit is set e.g., 8bit: 128-255
  if (value & (1 << (bits - 1))) != 0:
    # compute negative value
    value = value - (1 << bits)
  return value