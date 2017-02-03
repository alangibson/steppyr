import logging, unittest
from RaspberryPiStepperDriver.activators.tmc26x.spi import SPI
from RaspberryPiStepperDriver.activators.tmc26x import TMC26XActivator

logging.basicConfig(level=logging.DEBUG)
log = logging.getLogger(__name__)

class TestSuite(unittest.TestCase):
  def test_1(self):
    # Given
    spi_dev = SPI(None, None)
    tmc26x = TMC26XActivator(spi=spi_dev, dir_pin=0, step_pin=0, current=300, resistor=150)
    # When
    tmc26x.start()
    # Then
    print('driver_control_register', tmc26x.driver_control_register.get_all_values())
    print('chopper_config_register', tmc26x.chopper_config_register.get_all_values())
    print('cool_step_register', tmc26x.cool_step_register.get_all_values())
    print('stall_guard2_register', tmc26x.stall_guard2_register.get_all_values())
    print('driver_config_register', tmc26x.driver_config_register.get_all_values())

if __name__ == '__main__':
  unittest.main()
