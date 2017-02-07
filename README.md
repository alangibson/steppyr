# steppyr

A stepper motor library in Python

## Installation

### Installing WiringPi/GPIO for TMC4361

If you are using the TMC4361 and want to use the Raspberry Pi to provide the
external clock signal, you will need to use the gpio program included with
wiringPi in order to activate the clock pin. This is mainly due to the fact
that the RPi.GPIO library does not support setting the clock pin.

    git clone git://git.drogon.net/wiringPi
    cd wiringPi
    ./build
    gpio readall

## Usage

### TMC4361 motion controller

If you are using the TMC4361 and want to use the Raspberry Pi to provide the
external clock signal, you will need to manually set the clock pin (BCM 4).

    gpio mode 7 clock
    gpio clock 7 1600000

### TMC26x driver

For a TMC26X driver, first make sure you have spidev installed:

    pip install spidev

TMC26X example code:

    from RaspberryPiStepperDriver.accelstepper import AccelStepper
    from RaspberryPiStepperDriver.activators.spi import SPI
    from RaspberryPiStepperDriver.activators.tmc26x import TMC26XActivator
    from RaspberryPiStepperDriver.profiles.rectangle import RectangleProfile

    # Create the stepper driver
    stepper = AccelStepper(
      profile=RectangleProfile(),
      activator=TMC26XActivator(
        spi=SPI(bus=0, device=0),
        dir_pin=pinout.dir_pin,
        step_pin=pinout.step_pin,
        current=300
      )
    )
    stepper.set_target_speed(1000) # steps per second
    # stepper.set_acceleration(40000) # steps per second per second
    # stepper.set_pulse_width(2) # microseconds

TMC4361 example code:

    from RaspberryPiStepperDriver.drivers import StepperDriver
    from RaspberryPiStepperDriver.activators.tmc4361.driver import TMC4361
    from RaspberryPiStepperDriver.activators.tmc4361.spi import SPI

    spi = SPI(bus=0, device=1)
    tmc4361 = TMC4361(
      spi=spi,
      reset_pin=26
    )
    driver = StepperDriver(
      activator=tmc4361,
      profile=tmc4361
    )

For all other STEP/DIR drivers.

    TODO

## Testing

  source test/env
  python3 -m unittest discover

## Wiring

### TMC4361/TMC4361-Eval to TMC2660

TMC4361       TMC4361-Eval      TMC2660   
-------       ------------      -------
STPOUT_PWMA   DIO6 (#17)        STEP      
DIRPOUT_PWMB  DIO7 (#18)        DIR       
MP1           DIO14 (#36)       SG_TST    
NSCSDRV_SDO   SPI2_CSN0 (#24)   CSN       
SDODRV_SCLK   SPI2_SDO (#28)    SDI       
SCKDRV_NSDO   SPI_SCK (#27)     SCK       
SDIDRV_NSCLK  SPI2_SDI (#29)    SDO       

## Trinamic Support

TMC 260/261/262/2660 Stepper library for Python

Based on the Arduino TMC26X Stepper Motor Controller Library
  https://github.com/trinamic/TMC26XStepper

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
