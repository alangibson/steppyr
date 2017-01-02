# RaspberryPiStepperDriver

Python Raspberry Pi library for the STEP/DIR stepper drivers.

## Usage

There are currently 3 distinct driver implementations:

stepdir.StepperDriver
  Basic step/dir driver
  No ramping support
  All drivers not listed below inherit from this one.

tmc26x.TMC26XStepper
  No ramping support

accelstepper.AccelStepper
  A port of the Arduino AccelStepper library.
  Supports configurable ramp profiles.
    profiles.accel.AccelProfile
      The original AccelStepper profile
    profiles.max.MaxProfile
      A simple profile that accelerates and decelerates as fast as possible.

## Testing

  ./test/test.sh

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

## References

Arduino library for A4988, DRV8825, DRV8834 and generic two-pin (DIR/STEP) stepper motor drivers
  https://github.com/laurb9/StepperDriver

TMC260 / TMC260A & TMC261 DATASHEET
  http://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC261_datasheet.pdf

TMC262 DATASHEET
  http://www.microsemi.com/document-portal/doc_view/130727-trinamic-tmc262-stepper-motor-driver-datasheet

TMC4210+TMC2660 EVALUATION BOARD MANUAL
  http://www.trinamic.com/fileadmin/assets/Products/Eval_Documents/TMC4210_TMC2660_Eval_Manual.pdf

Arduino TMC26X Stepper Motor Controller Library
  https://github.com/trinamic/TMC26XStepper
