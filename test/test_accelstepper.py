import asyncio, logging, time, unittest
import steppyr
from steppyr.profiles import accel
from steppyr.drivers.stepdir import StepDirDriver

logging.basicConfig(level=logging.DEBUG)

class TestSuite(unittest.TestCase):
  def test_1(self):
    # def __init__(self, profile, dir_pin, step_pin, enable_pin=None, pin_mode=GPIO.BCM):
    stepper = steppyr.StepperController(
      driver=StepDirDriver(999, 999, 999),
      profile=accel.AccelProfile()
    )
    stepper.activate()

    motor_steps_per_rev = 200
    motor_max_rpm = 200

    # steps per second
    stepper.set_target_speed(motor_steps_per_rev * ( motor_max_rpm / 60 ) )
    stepper.set_target_acceleration(1000) # steps per second per second
    # Pulse width is defined by the stepper driver. 1.9us for the TB6560
    stepper.set_pulse_width(2)

    async def doit():
        stepper.move_to(100)
        while True:
            await asyncio.sleep(0)
            # If at the end of travel go to the other end
            if stepper.steps_to_go == 0:
                stepper.move_to(-stepper.current_position)
            # stepper.run()
            await asyncio.sleep(0)

    async def quickChange():
      stepper.move_to(10)
      stepper.move_to(15)
      stepper.move_to(20)
      stepper.move_to(100)
      await asyncio.sleep(0.2)
      stepper.move_to(300)
      await asyncio.sleep(1)
      stepper.move_to(30)
      # FIXME we don't get to 30 because move_to doesnt block

    loop = asyncio.get_event_loop()
    # asyncio.ensure_future(stepper.run_forever())
    # asyncio.ensure_future(quickChange())
    try:
        loop.run_until_complete(quickChange())
    finally:
        stepper.shutdown()
        loop.close()

if __name__ == '__main__':
  unittest.main()
