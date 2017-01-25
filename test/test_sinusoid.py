import math
from RaspberryPiStepperDriver.profiles.sinusoid import *

amplitude = 1
full_steps_per_sec = 100
length_full_steps = 300
# Set frequency based on full_steps_per_sec
# frequency = full_steps_per_sec # frequency=events/time
# Since 1 period/full oscillation traverses the y axis 2 times:
#   f = 1 hz = length_full_steps*2 steps/1 sec
# We can do this with full steps since the ratio is the same with microstepping
frequency = full_steps_per_sec / ( length_full_steps * 2 )
# Phase shift by 45 so we always start at the lowest point of negative amplitude
phase_shift = math.radians(270)

curves = [
  (amplitude, frequency, phase_shift),
  (amplitude, frequency*2, phase_shift),
  (amplitude*2, frequency*2, phase_shift)
]

profile = SinusoidProfile(curves=curves, full_steps_per_sec=full_steps_per_sec,
  length_full_steps=length_full_steps, microsteps=1)

while True:
  if profile.should_step():
    profile.step()
    print(profile._current_steps, profile.direction)
  # print(should_step)
