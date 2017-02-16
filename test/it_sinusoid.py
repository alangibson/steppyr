import asyncio, logging, math
from steppyr.planners.sinusoid import SinusoidPlan, StepperWave

logging.basicConfig(level=logging.DEBUG)
log = logging.getLogger(__name__)

class MockController:
  def move_to(self, position):
    log.debug('MockController.move_to(%s)', position)

def demo():
  # Length of slider in full steps aka diameter of unit circle aka length of 2 radians
  length_full_steps = 300
  # Target speed in full steps per second
  target_speed = 100

  waves = [
    StepperWave(length_full_steps, target_speed),
    StepperWave(length_full_steps, target_speed*1.5),
    StepperWave(length_full_steps, target_speed*2, phase_shift=math.radians(270)+1)
  ]

  planner = SinusoidPlan(waves=waves, controller=MockController())

  loop = asyncio.get_event_loop()
  asyncio.ensure_future(planner.run_forever())
  try:
    loop.run_forever()
  except:
    loop.close()

if __name__ == '__main__':
  demo()
