import unittest
from RaspberryPiStepperDriver.activators.tmc4361.registers import mask, Representation, \
  StatusFlagRegister, StatusEventRegister, SpiStatusSelectionRegister, ExternalClockFrequencyRegister, VMaxRegister

class TestRegisters(unittest.TestCase):
  def test_1(self):
    register = StatusEventRegister(0b10)
    events = register.get_values()
    self.assertEqual(events[0][1], StatusEventRegister.bits.POS_COMP_REACHED)

  def test_mask(self):
    self.assertEqual(mask(7,7), 0b10000000)

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

  def test_mask(self):
    self.assertEqual(mask(5, 6), 0b1100000)

  def test_representation_set(self):
    num = 44.443355
    # rep = Representation(0, 0, 24, 8)
    register = VMaxRegister().set(VMaxRegister.bits.VMAX, num)
    self.assertEqual(register.data, 11377)

  def test_representation_roundtrip(self):
    num = 44.443355
    # rep = Representation(0, 0, 24, 8)
    register = VMaxRegister().set(VMaxRegister.bits.VMAX, num)
    self.assertEqual(register.data, 11377)
    value = register.get(VMaxRegister.bits.VMAX)
    # Note: fixed to floating point conversion is inexact
    self.assertEqual(value, 44.44140625)

  def test_get_status_flags(self):
    values = StatusFlagRegister(0b1).get_values()
    self.assertTrue('TARGET_REACHED_F' in [t[0] for t in values])

if __name__ == '__main__':
  unittest.main()
