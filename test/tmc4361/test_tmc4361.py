import unittest
from steppyr.drivers.tmc4361 import TMC4361Driver, MotorDriverSettingsRegister

def dump_registers(tmc4361, exclude_zero=False):
  for register_class, register in tmc4361._registers.items():
    print(register_class.__name__, bin(register.register))
    for name, representation in register.bits.items():
      value = register.get(representation)
      if exclude_zero and not value:
        continue
      print('    ', name, value)

class TestRegisters(unittest.TestCase):
  def test_load_registers_from_ini(self):
    tmc4361 = TMC4361Driver(spi=None)
    tmc4361.load_registers_from_ini('test/tmc4361/data/20170210_02.55.36_TMC4361_Settings.ini')
    # dump_registers(tmc4361, exclude_zero=True)
    self.assertEqual(tmc4361._registers[MotorDriverSettingsRegister].data, 0b10110000110010001000)

if __name__ == '__main__':
  unittest.main()
