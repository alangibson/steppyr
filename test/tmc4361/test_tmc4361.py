import unittest
from steppyr.drivers.tmc4361 import TMC4361Driver

class TestRegisters(unittest.TestCase):
  def test_load_registers_from_ini(self):
    tmc4361 = TMC4361Driver(spi=None)
    tmc4361.load_registers_from_ini('test/tmc4361/data/20170210_02.55.36_TMC4361_Settings.ini')
    self.assertEqual(46, len(tmc4361._registers))

if __name__ == '__main__':
  unittest.main()
