from steppyr import datagram_to_int, clear_bit, _BV

WRITE_MASK = 0x80 # register | WRITE_MASK
READ_MASK = 0x7F # register & READ_MASK

class Datagram:

  def __init__(self, header, data):
    """
    header: 8 bit status or register int
    data: 32 bit message data
    """
    self._header = header
    if hasattr(data, '__iter__'):
      self._data = datagram_to_int(data)
    else:
      self._data = data
    self._is_read = True

  @property
  def datagram(self):
    return (self._header << 32) | self._data

  @property
  def header(self):
    return self._header

  @property
  def data(self):
    return self._data

  def to_list(self):
    """
    Returns entire datagram (header+data) as a list of 5 8-bit ints.
    """
    datagram_list = [
      self._header,
      (self._data >> 24) & 0xff,
      (self._data >> 16) & 0xff,
      (self._data >>  8) & 0xff,
      (self._data) & 0xff
    ]
    return datagram_list

  #
  # Properties and methods for requests
  #

  @property
  def register(self):
    # clear MSB (#7) because this is the read/write bit
    return clear_bit(self._header, 7)

  def set_write(self):
    """
    Set flag to make this a write operation
    """
    self._header = self._header | WRITE_MASK
    self._is_read = False
    return self

  def set_read(self):
    """
    Set flag to make this a read operation
    """
    self._header = self._header & READ_MASK
    self._is_read = True
    return self

  @property
  def is_read(self):
    return self._is_read

  @property
  def is_write(self):
    return not self._is_read

  #
  # Properties and methods for responses
  #

  @property
  def status(self):
    """
    Note: the meaning of the status depends on the settings of the
    SPI_STATUS_SELECTION register.
    """
    return self._header

  def as_response(self, datagram_list):
    """
    Clone ourselves as a response.
    """
    return type(self)(datagram_to_int(datagram_list))

class Status:
  """
  Parses status header returned by TMC2631.
  Transferred with every SPI datagram response as SPI_STATUS.

  In order to select an event for the SPI status bits, assign the
  SPI_STATUS_SELECTION register 0x0B according to the particular event in
  the EVENTS register.
  The bit positions are sorted according to the event bit positions in the EVENTS
  register 0x0E. In case more than eight events are selected, the first eight bits
  (starting from index 0 = LSB) are forwarded as SPI_STATUS.
  """

  def __init__(self, status, spi_status_selection_register):
    """
    status: 8-bit int status code from TCM4361.
    spi_status_selection_register: A configured instance of SpiStatusSelectionRegister
    """
    self._status = status
    self._spi_status_selection_register = spi_status_selection_register

  def get_values(self):
    selection_values = self._spi_status_selection_register.get_values()
    # Order selection_values from lsb to msb and take first 8
    selection_values = sorted(selection_values, key=lambda x: x[1])[0:7]
    values = []
    # ordinal position of bit in self._status == oridinal position in selection_values
    for i in range(0, len(selection_values)):
      if self._status & _BV(i):
        values.append(selection_values[i])
    return values
