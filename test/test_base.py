from RaspberryPiStepperDriver.base import *

motor_steps = 200
dir_pin = 8
step_pin = 5
enable_pin = 7

GPIO.cleanup()

stepper = StepperDriver(motor_steps, dir_pin, step_pin, enable_pin,
                        microsteps=1, rpm=60)
stepper.enable()

try:
  while True:
    stepper.rotate(360)
    stepper.move(-200)
except KeyboardInterrupt:
  stepper.disable()

# GPIO.cleanup()
