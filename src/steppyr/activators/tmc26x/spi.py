from steppyr.activators import spi as activator_spi

class SPI(activator_spi.SPI):

  def write(self, datagram):
    self.transfer(datagram.to_list())
