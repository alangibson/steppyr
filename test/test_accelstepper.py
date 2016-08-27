import asyncio, time
from RaspberryPiStepperDriver import accelstepper

stepper = accelstepper.AccelStepper(999,999)

motor_steps_per_rev = 200
motor_max_rpm = 200
max_speed = ( motor_steps_per_rev * motor_max_rpm ) / 60

print('max_speed', max_speed)

# steps per second
stepper.max_speed = max_speed
stepper.acceleration = 1000 # steps per second per second
# Pulse width is defined by the stepper driver. 1.9us for the TB6560
stepper.pulse_width = 2

async def doit():
    stepper.moveTo(100)
    while True:
        await asyncio.sleep(0)
        # If at the end of travel go to the other end
        if stepper.distanceToGo() == 0:
            stepper.moveTo(-stepper.currentPosition)
        # stepper.run()
        await asyncio.sleep(0)

async def quickChange():
  stepper.moveTo(10)
  stepper.moveTo(15)
  stepper.moveTo(20)
  stepper.moveTo(100)
  await asyncio.sleep(0.2)
  stepper.moveTo(300)
  await asyncio.sleep(1)
  stepper.moveTo(30)

loop = asyncio.get_event_loop()
asyncio.ensure_future(stepper.runForever())
asyncio.ensure_future(quickChange())
try:
    loop.run_forever()
finally:
    loop.close()
