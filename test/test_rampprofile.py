from RaspberryPiStepperDriver.profiles import calc_speed_from_rpm

def test_calc_speed_from_rpm():
  # Given
  rpm = 100
  steps_per_rev = 200
  microsteps = 1
  # When
  speed_steps_per_sec = calc_speed_from_rpm(rpm, steps_per_rev, microsteps)
  # Then
  assert(speed_steps_per_sec == (rpm / 60) * steps_per_rev * microsteps)

if __name__ == '__main__':
  test_calc_speed_from_rpm()
