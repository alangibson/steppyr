
class SpiDev:

  messages_sent = []

  def __init__(self):
    pass

  def open(self, bus=0, device=0):
    pass

  def xfer2(self, payload):
    self.messages_sent.append(payload)
    return [0 for x in payload]
