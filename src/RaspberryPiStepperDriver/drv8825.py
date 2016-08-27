from RaspberryPiStepperDriver import a4988

class DRV8825(a4988.A4988):
    """
    TODO enforce minimal pulse duration of 1.9us. pg7 https://www.pololu.com/file/download/drv8825.pdf
    """

    microstep_table = {
        0: int('000', 2),
        1: int('000', 2),
        2: int('001', 2),
        4: int('010', 2),
        8: int('011', 2),
        16: int('100', 2),
        32: int('111', 2) }

    # tA STEP minimum, HIGH pulse width (1.9us)
    step_high_min_us = 2;
    # tB STEP minimum, LOW pulse width (1.9us)
    step_low_min_us = 2;
    # wakeup time, nSLEEP inactive to STEP (1000us)
    wakeup_time_us = 1700;
    #also 650ns between ENBL/DIR/MSx changes and STEP HIGH
