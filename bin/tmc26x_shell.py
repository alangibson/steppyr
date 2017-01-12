import asyncio, logging
from RaspberryPiStepperDriver import drivers
from RaspberryPiStepperDriver.activators import spi, tmc26x
from RaspberryPiStepperDriver.profiles import accel as accel_profile, max as max_profile, rectangle as rectangle_profile

logging.basicConfig(level=logging.DEBUG)
log = logging.getLogger(__name__)

driver = drivers.StepperDriver(
  activator=tmc26x.TMC26XActivator(
    spi=spi.SPI(bus=0, device=0),
    dir_pin=23,
    step_pin=18,
    current=300
  ),
  profile=rectangle_profile.RectangleProfile()
)
# stepper_driver.set_target_speed(stepper.max_speed)
# stepper_driver.set_microsteps(1)
# stepper_driver.set_current(1000)
# stepper_driver.set_acceleration(40000) # steps per second per second
# stepper_driver.set_pulse_width(2)

driver.start()

loop = asyncio.get_event_loop()
