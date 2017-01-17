import math
from RaspberryPiStepperDriver import constrain, micros, project, sign, micros, sleep_microseconds
from RaspberryPiStepperDriver.profiles import calc_step_interval_us

MIN_ACCEL = 60
MAX_ACCEL = 300 # 360 //300//320      // Maximun motor acceleration in (steps/seg2)/1000
MAX_SPEED = 25000 # max 25000 for 12V   // Maximun speed in steps/seg
# This is for the Accel ramp implementation (to smooth the intial acceleration), simplified S-profile
ACCEL_RAMP_MIN = 2500 # The S profile is generated up to this speed
ACCEL_RAMP_MAX = 10000
ZERO_SPEED = 65535

class SCurveProfile:
  """
  https://forum.arduino.cc/index.php?topic=22677.0
  """
  def __init__(self):
    # acceleration time in microsec (ta)
    self.acceleration_time_us = 3e6
    # deceleration time in microsec (td)
    self.deceleration_time_us = 3e6
    # current time in microsec (t)
    # You need to seed the initial time with something > 0
    # so you don't calculate to long of a delay
    # self.current_time = self.deceleration_time_us / 9
    self.start_time = micros()
    self.current_time = micros()+1
    # stepper pulse delay in microsec (dly)
    self.step_interval_us = None
    # count the number of pulses (count)
    self.current_steps = 0
    # TODO int Perr = 0;       // error in position
    self.last_time = micros()
    self.current_speed = 0
    self.current_step_offset = 0
    self.current_acceleration = 0
    self.min_acceleration = MIN_ACCEL
    self.current_step_offset = 0

    self.target_steps = 400
    # Lowest speed to apply to motor aka the speed to start at
    self.start_speed = 100
    self.target_speed = 500

  def compute_new_speed_orig(self):

    # acceleration time in microsec (ta)
    # acceleration_time_us = 3e6
    acceleration_time_us = self.acceleration_time_us
    # deceleration time in microsec (td)
    # deceleration_time_us = 3e6
    deceleration_time_us = self.deceleration_time_us
    # steady state velocity (pulse/sec) (Vm)
    target_speed_step_per_sec = 3200
    # total number of pulses for move (1600 steps per rev) (Pt)
    target_steps = 12800

    # stepper pulse delay in microsec (dly)
    # step_interval_us = None
    step_interval_us = self.step_interval_us
    # current time in microsec (t)
    # You need to seed the initial time with something > 0
    # so you don't calculate to long of a delay
    # current_time = deceleration_time_us / 9
    current_time = self.current_time
    # time during constant velocity in microsec (t12)
    time_at_target_speed_us = 0

    # Calculate the time at constant velocity
    step_interval_sec = target_speed_step_per_sec / 1e6 # Vm / 1e6
    total_time_us = target_steps / step_interval_sec
    time_at_target_speed_us = total_time_us - 0.5 * ( acceleration_time_us + deceleration_time_us )

    # print('current_time=%s' % (current_time))

    # Decide which part of the velocity curve your at
    if current_time < acceleration_time_us:
      # Acceleration
      # dly = (ta)/(2*(Vm/1e6)*t)
      step_interval_us = acceleration_time_us / ( 2 * step_interval_sec * current_time )
    elif current_time >= acceleration_time_us \
        and current_time < ( acceleration_time_us + time_at_target_speed_us ):
      # Constant velocity
      # dly = 1/(2*(Vm/1e6))
      step_interval_us = 1 / ( 2 * step_interval_sec )
    elif current_time >= ( acceleration_time_us + time_at_target_speed_us ) \
        and current_time < ( acceleration_time_us + time_at_target_speed_us + deceleration_time_us ):
      # Deceleration
      # dly = 1/(2*((Vm/1e6)-(Vm/(1e6*td))*(t-ta-t12)))
      step_interval_us = 1 / ( 2 * ( step_interval_sec - ( target_speed_step_per_sec / ( 1e6 * deceleration_time_us ) ) * ( current_time - acceleration_time_us - time_at_target_speed_us ) ) )
    #else:
    #  print('WRONG')

    # update the current time
    current_time = current_time + 2 * step_interval_us

    # Save some variables
    self.current_time = current_time
    self.step_interval_us = step_interval_us

  def compute_new_speed_steps(self):

    # acceleration time in microsec (ta)
    # acceleration_time_us = 3e6
    acceleration_time_us = self.acceleration_time_us
    # deceleration time in microsec (td)
    # deceleration_time_us = 3e6
    deceleration_time_us = self.deceleration_time_us
    # steady state velocity (pulse/sec) (Vm)
    target_speed_step_per_sec = 400
    # total number of pulses for move (1600 steps per rev) (Pt)
    target_steps = 1600

    # stepper pulse delay in microsec (dly)
    # step_interval_us = None
    step_interval_us = self.step_interval_us
    # current time in microsec (t)
    # You need to seed the initial time with something > 0
    # so you don't calculate to long of a delay
    # current_time = deceleration_time_us / 9
    current_time = self.current_time
    # time during constant velocity in microsec (t12)
    time_at_target_speed_us = 0

    # Calculate the time at constant velocity
    step_interval_sec = target_speed_step_per_sec / 1e6 # Vm / 1e6
    # total_time_us = target_steps / step_interval_sec
    # time_at_target_speed_us = total_time_us - 0.5 * ( acceleration_time_us + deceleration_time_us )

    acceleration_steps = 100
    deceleration_steps = 100
    steps_at_target_speed = target_steps - ( acceleration_steps + deceleration_steps )

    # Decide which part of the velocity curve youre at
    if self.current_steps < acceleration_steps:
      # Acceleration
      # dly = (ta)/(2*(Vm/1e6)*t)
      # step_interval_us = acceleration_time_us / ( 2 * step_interval_sec * current_time )

      step_interval_us = acceleration_steps / ( step_interval_sec * self.current_steps )

    elif self.current_steps >= acceleration_steps \
        and self.current_steps < ( acceleration_steps + steps_at_target_speed ):
      # Constant velocity
      # dly = 1/(2*(Vm/1e6))
      # step_interval_us = 1 / ( 2 * step_interval_sec )

      step_interval_us = 1 / ( step_interval_sec )

    elif self.current_steps >= ( acceleration_steps + steps_at_target_speed ) \
        and self.current_steps < target_steps:
      # Deceleration
      # dly = 1/(2*((Vm/1e6)-(Vm/(1e6*td))*(t-ta-t12)))
      # step_interval_us = 1 / ( 2 * ( step_interval_sec - ( target_speed_step_per_sec / ( 1e6 * deceleration_time_us ) ) * ( current_time - acceleration_time_us - time_at_target_speed_us ) ) )

      step_interval_us = deceleration_steps / ( step_interval_sec * (target_steps - self.current_steps) )

    else:
      step_interval_us = 0.0

    # update the current time
    # current_time = current_time + 2 * step_interval_us

    # Save some variables
    # self.current_time = current_time
    self.step_interval_us = step_interval_us
    self.current_speed = 0 if step_interval_us == 0 else 1000000.0 / step_interval_us

  def compute_new_speed(self):
    # Inputs
    target_position = target_steps = 1000
    target_speed = 500
    max_acceleration = 300
    accel_ramp = ACCEL_RAMP_MIN

    speed = self.current_speed
    position = self.current_steps

    current_time = micros()

    # print('INPUTS speed=%s position=%s current_time=%s' % (speed, position, current_time))

    # Limit dt (it should be around 1000 most times)
    dt = constrain(current_time - self.last_time, 0, 2000)
    self.last_time = current_time

    # We use an acceleration ramp to imitate an S-curve profile at the begining and end (depend on speed)
    acceleration = project(abs(speed), 0, accel_ramp, self.min_acceleration, max_acceleration)
    acceleration = constrain(acceleration, self.min_acceleration, max_acceleration)

    temp = speed * speed
    temp = temp / (1800 * acceleration) # 2000*0.85 = 1700 0.85 is a compensation for deceleration ramp (S-curve)
    pos_stop = position + ( sign(speed) * temp )
    # print('temp=%s pos_stop=%s' % (temp, pos_stop))
    if target_position > position:  # Positive move
      if pos_stop >= target_position:  # Start decelerating?
        # The deceleration ramp is done inside the setSpeed function
        self.setMotorSpeed(0, dt, acceleration, speed)
      else:
        # The aceleration ramp is done inside the setSpeed function
        self.setMotorSpeed(target_speed, dt, acceleration, speed)
    else: # Negative move
      if pos_stop <= target_position: # Start decelerating?
        self.setMotorSpeed(0, dt, acceleration, speed)
      else:
        self.setMotorSpeed(-target_speed, dt, acceleration, speed)

    # TODO
    self.step_interval_us = None
    # self.current_speed = speed

  #  Speed could be positive or negative
  def setMotorSpeed(self, tspeed, dt, acceleration, speed):
    # Limit max speed
    if tspeed > MAX_SPEED:
      tspeed = MAX_SPEED
    elif tspeed < -MAX_SPEED:
      tspeed = -MAX_SPEED

    # We limit acceleration => speed ramp
    accel = ( acceleration * dt ) / 1000;  # We divide by 1000 because dt are in microseconds
    # print('tspeed=%s speed=%s acceleration=%s dt=%s accel=%s' % (tspeed, speed, acceleration, dt, accel))
    if ( tspeed - speed ) > accel: # We use long here to avoid overflow on the operation
      speed += accel;
    elif ( speed - tspeed ) > accel:
      speed -= accel
    else:
      speed = tspeed

    self.current_speed = speed

    # Check if we need to change the direction pins
    """
    if speed == 0 and dir != 0 :
      dir = 0;
    elif speed > 0 and dir != 1:
      dir = 1
    elif speed < 0 and dir != -1:
      dir = -1
    """

    if speed == 0:
      timer_period = ZERO_SPEED
    elif speed > 0:
      timer_period = 2000000 / speed # 2Mhz timer
    else:
      timer_period = 2000000 / -speed

    if timer_period > 65535: # Check for minimun speed (maximun period without overflow)
      timer_period = ZERO_SPEED

  def compute_new_speed(self):

    self.last_time = self.current_time
    self.current_time = micros()

    self.current_speed, self.step_interval_us, self.current_acceleration = curve(
      self.current_steps, self.target_steps,
      self.start_speed, self.current_speed, self.target_speed,
      self.last_time, self.current_time)

    sleep_microseconds(self.step_interval_us)

  def step(self):
    self.current_steps = self.current_steps + 1
    self.compute_new_speed()

  def to_csv(self):
    print('%s,%s,%s,%s,%s' % ( self.current_time, self.step_interval_us, self.current_speed, self.current_acceleration, self.current_steps ) )

def curve(current_steps, target_steps, start_speed, current_speed, target_speed, last_time, current_time):
  # sigmoid = lambda x: 1 / (1 + x**16)
  # sigmoid = lambda x: math.tanh(x)
  sigma = lambda x: 1 / ( 1 + math.e**-x ) # x = decending, -x = ascending
  # sigmoid = lambda x: x / 1 + abs(x)
  steep = 2
  ulimit = 2 # max y = ulimit - 1
  llimit = 1 # min y
  sigmoid = lambda x: ulimit * sigma( steep * x ) - llimit

  x = project(current_steps, 0, target_steps, -10, 10)

  y = sigmoid(x)
  next_speed = project(y, -1, 1, start_speed, target_speed)

  # Determine speed
  step_interval_us = calc_step_interval_us(next_speed)

  # Current accel in steps per second per second (step/sec^2)
  # print(next_speed, current_speed, current_time)
  current_acceleration = ( next_speed - current_speed ) / ( ( current_time - last_time ) / 1e6 )

  # current_time, step_interval_us, current_speed, current_acceleration, current_steps
  return (next_speed, step_interval_us, current_acceleration)

if __name__ == '__main__':
  profile = SCurveProfile()
  for i in range(0, 400):
    profile.step()
    # profile.dump()
    profile.to_csv()
