from RaspberryPiStepperDriver import stepdir

class TB6560(stepdir.StepperDriver):
  """
  Set the constructors microsteps argument to match the number of microsteps
  set on the drivers hardware switches.

  EN (Enable input): Grounding EN+ input will enable the motor drive outputs.
  Providing a logic high will disable outputs to the motor.

  CW (Direction input): Grounding CW+ input will cause the motor to rotate
  anti-clockwise (counter-clockwise). Providing a logic high will cause the
  motor to rotate clockwise. The direction of rotation also depends upon the
  motor coil polarity.

  CLK (Step input): Continually pulsing the CLK+ input will cause the motor to
  step in one direction. Depending on the excitation mode setting (via switches
  SW3 & 4) the motor will step once per 1 to 16 pulses of the CLK+ pin.

  See http://forum.hobbycomponents.com/viewtopic.php?f=76&t=1371
  """
  pass
