from steppyr.lib.trinamic import Datagram as TrinamicDatagram

class Datagram(TrinamicDatagram):

  def __init__(self, header, data, header_len=3, datagram_len=20):
    """
    header: status or register int
    data: message data
    """
    super().__init__(header=header, data=data, header_len=header_len, datagram_len=datagram_len)

  def to_list(self):
    """
    Returns entire datagram (header+data) as a list of 8-bit ints.
    """
    # TMC26x only: datagram is 4 separate bytes
    datagram_list = [
      self._header,
      (self._data >> 16) & 0xff,
      (self._data >>  8) & 0xff,
      (self._data) & 0xff
    ]
    return datagram_list
