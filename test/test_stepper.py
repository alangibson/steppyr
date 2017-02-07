from steppyr.drivers import calc_degrees_to_steps

def test_calc_degrees_to_steps():
  # given
  degrees = 180
  motor_steps_per_rev = 100
  microsteps = 2
  # when
  steps = calc_degrees_to_steps(degrees, motor_steps_per_rev, microsteps)
  # then
  assert(steps == 100)

if __name__ == '__main__':
  test_calc_degrees_to_steps()
