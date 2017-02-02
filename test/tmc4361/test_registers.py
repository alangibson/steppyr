import unittest
from RaspberryPiStepperDriver.motion.tmc4361.registers import \
  StatusEventRegister, SpiStatusSelectionRegister, ExternalClockFrequencyRegister

class TestRegisters(unittest.TestCase):
  def test_1(self):
    register = StatusEventRegister(0b10)
    events = register.get_values()
    self.assertEqual(events[0][1], StatusEventRegister.bits.POS_COMP_REACHED)

  def test_SpiStatusSelectionRegister_defaults(self):
    register = SpiStatusSelectionRegister()

  def test_SpiStatusSelectionRegister(self):
    register = SpiStatusSelectionRegister(0)
    register.set(StatusEventRegister.bits.VACTUAL_EQ_0)
    register.set(StatusEventRegister.bits.POS_COMP_REACHED)
    self.assertEqual(register.data, 0b1010)
    # TODO assert: register.get_values()

  def test_register_set_1(self):
    register = SpiStatusSelectionRegister(0)
    register.set(0b111000, 1)
    self.assertEqual(register.data, 0b001000)

  def test_register_set_2(self):
    register = SpiStatusSelectionRegister(0)
    register.set(0b111000, 0)
    self.assertEqual(register.data, 0b000000)

  def test_ExternalClockFrequencyRegister_init(self):
    register = ExternalClockFrequencyRegister(16000000)
    self.assertEqual(register.data, 16000000)

  def test_ExternalClockFrequencyRegister_set(self):
    register = ExternalClockFrequencyRegister()
    register.set(ExternalClockFrequencyRegister.bits.EXTERNAL_CLOCK_FREQUENCY, 16000000)
    self.assertEqual(register.data, 16000000)

if __name__ == '__main__':
  unittest.main()
