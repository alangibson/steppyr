from RaspberryPiStepperDriver.stepdir import *

motor_steps = 200
dir_pin = 8
step_pin = 5
enable_pin = 7

GPIO.cleanup()

stepper = StepperDriver(motor_steps, dir_pin, step_pin, enable_pin,
                        microsteps=1, rpm=60)
stepper.enable()

async def run():
  await stepper.rotate(360)
  await stepper.move(-200)

loop = asyncio.get_event_loop()
# asyncio.ensure_future(stepper.run_forever())
# asyncio.ensure_future(quickChange())
try:
    loop.run_until_complete(run())
finally:
    # stepper.stop()
    loop.close()
