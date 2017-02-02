import asyncio, logging
from RaspberryPiStepperDriver import drivers
from RaspberryPiStepperDriver.activators import spi, tmc26x
from RaspberryPiStepperDriver.profiles import accel as accel_profile, max as max_profile, rectangle as rectangle_profile
from RaspberryPiStepperDriver.motion.tmc4361.driver import TMC4361
from RaspberryPiStepperDriver.motion.tmc4361.spi import SPI as TMC4361SPI

logging.basicConfig(level=logging.DEBUG)
log = logging.getLogger(__name__)

spi0 = spi.SPI(bus=0, device=0)
spi1 = TMC4361SPI(bus=0, device=1)

driver = drivers.StepperDriver(
  activator=tmc26x.TMC26XActivator(
    spi=spi0,
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

tmc4361 = TMC4361(
  spi=spi1,
  start_signal_pin=0,
  target_reached_interrupt_pin=0
)

driver.start()
tmc4361.start()

loop = asyncio.get_event_loop()
