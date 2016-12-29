# RaspberryPiStepperDriver

Python Raspberry Pi library for the STEP/DIR stepper drivers.

## Trinamic Support

TMC 260/261/262/2660 Stepper library for Python

Based on the Arduino TMC26X Stepper Motor Controller Library
  https://github.com/trinamic/TMC26XStepper

### Prerequisites

  pip install spidev

### Usage

  from RaspberryPiStepperDriver import spi, tmc26x

  # TMC26X drivers must be configured via SPI
  # Create an SPI object for /dev/spidev0.0
  spi_dev = spi.SPI(bus=0, device=0)

  # 200 steps per motor revolution
  # dir_pin = 23, step_pin = 18
  # driver current = 300 milliamps
  driver = tmc26x.TMC26XStepper(spi_dev, 200, 23, 18, 300)
  driver.start()
