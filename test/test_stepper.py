import unittest
from steppyr import calc_degrees_to_steps

class TestSuite(unittest.TestCase):

  def test_calc_degrees_to_steps(self):
    # given
    degrees = 180
    motor_steps_per_rev = 100
    microsteps = 2
    # when
    steps = calc_degrees_to_steps(degrees, motor_steps_per_rev, microsteps)
    # then
    assert(steps == 100)

if __name__ == '__main__':
  unittest.main()
