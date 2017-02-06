from RaspberryPiStepperDriver.activators.a4988 import A4988Activator
import RPi.GPIO as GPIO

GPIO.DEBUG = True

driver = A4988Activator(dir_pin=1, step_pin=2,
    ms1_pin=3, ms2_pin=4, ms3_pin=5)

driver.set_microsteps(0)
driver.set_microsteps(1)
driver.set_microsteps(2)
driver.set_microsteps(4)
driver.set_microsteps(8)
driver.set_microsteps(16)
