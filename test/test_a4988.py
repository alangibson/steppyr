from RaspberryPiStepperDriver import a4988
import RPi.GPIO as GPIO

GPIO.DEBUG = True

driver = a4988.A4988(motor_steps=200, dir_pin=1, step_pin=2,
    ms1_pin=3, ms2_pin=4, ms3_pin=5)

driver.set_microstep(0)
driver.set_microstep(1)
driver.set_microstep(2)
driver.set_microstep(4)
driver.set_microstep(8)
driver.set_microstep(16)
