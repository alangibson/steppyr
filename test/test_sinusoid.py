import asyncio, logging, math
from RaspberryPiStepperDriver.profiles.sinusoid import *
from RaspberryPiStepperDriver.drivers import StepperDriver
from RaspberryPiStepperDriver.activators.spi import SPI
from RaspberryPiStepperDriver.activators.tmc26x import TMC26XActivator

logging.basicConfig(level=logging.DEBUG)

def demo():
  # Diameter of unit circle aka length of 2 radians
  length_full_steps = 300
  # Target speed in steps per second
  target_speed = 100

  amplitude = length_full_steps / 2
  # Set frequency based on target_speed
  # frequency = target_speed = events/time
  # Since 1 period/full oscillation traverses the y axis 2 times:
  #   f = 1 hz = length_full_steps*2 steps/1 sec
  # We can do this with full steps since the ratio is the same with microstepping
  # frequency = target_speed / ( length_full_steps * 2 )
  frequency = target_speed / ( amplitude * 4 )
  # Phase shift by 45 so we always start at the lowest point of negative amplitude
  phase_shift = math.radians(270)

  waves = [
    (amplitude, frequency, phase_shift),
    (amplitude, frequency*2, phase_shift),
    (amplitude, frequency*3, phase_shift+1)
  ]

  profile = SinusoidProfile(waves=waves)

  while True:
    if profile.should_step():
      profile.step()
      print(profile._current_steps)

def demo2():
  # Diameter of unit circle aka length of 2 radians
  length_full_steps = 300
  # Target speed in steps per second
  target_speed = 100

  amplitude = length_full_steps / 2
  # Set frequency based on target_speed
  # frequency = target_speed = events/time
  # Since 1 period/full oscillation traverses the y axis 2 times:
  #   f = 1 hz = length_full_steps*2 steps/1 sec
  # We can do this with full steps since the ratio is the same with microstepping
  # frequency = target_speed / ( length_full_steps * 2 )
  frequency = target_speed / ( amplitude * 4 )
  # Phase shift by 45 so we always start at the lowest point of negative amplitude
  phase_shift = math.radians(270)

  waves = [
    (amplitude, frequency, phase_shift),
    (amplitude, frequency*2, phase_shift),
    (amplitude, frequency*3, phase_shift+1)
  ]

  # Create the stepper driver
  driver = StepperDriver(
    profile=SinusoidProfile(waves=waves),
    activator=TMC26XActivator(
      spi=SPI(bus=0, device=0),
      dir_pin=999,
      step_pin=999,
      current=300
    )
  )
  # stepper.set_target_speed(1000) # steps per second
  # stepper.set_acceleration(40000) # steps per second per second
  # stepper.set_pulse_width(2) # microseconds

  loop = asyncio.get_event_loop()
  asyncio.ensure_future(driver.run_forever())
  loop.run_forever()

if __name__ == '__main__':
  demo2()
