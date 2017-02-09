import asyncio, unittest
from steppyr.drivers.stepdir import StepDirDriver
import RPi.GPIO as GPIO

class TestSuite(unittest.TestCase):
  def test_1(self):
    motor_steps = 200
    dir_pin = 8
    step_pin = 5
    enable_pin = 7

    GPIO.cleanup()

    stepper = StepDirDriver(dir_pin, step_pin, enable_pin)
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

if __name__ == '__main__':
  unittest.main()
