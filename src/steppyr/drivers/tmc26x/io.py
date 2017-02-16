from steppyr.lib.bits import clear_bit, _BV, datagram_to_int

class Datagram:

  def __init__(self, header, data, header_len=3, datagram_len=20):
    """
    header: 8 bit status or register int
    data: 32 bit message data
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
    datagram_list = [
      self._header,
      (self._data >> 16) & 0xff,
      (self._data >>  8) & 0xff,
      (self._data) & 0xff
    ]
    return datagram_list

  def as_response(self, datagram_list):
    """
    Clone ourselves as a response.
    """
    return type(self)(datagram_to_int(datagram_list))
