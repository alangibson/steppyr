import functools, logging, math, time
from RaspberryPiStepperDriver import DIRECTION_CW, DIRECTION_CCW, DIRECTION_NONE, project, micros
from . import RampProfile

log = logging.getLogger(__name__)

def curve1(a=1, f=1, p=0):
  """
  Implements
    y=A*sin(xb+p)
  aka
    y(t)=A*sin(wt+o)

  A = amplitude
  x = t = time, distance or angle
  p = o = phase angle
  b = w = 2pi/T = 2pi*f = angular frequency, meaning how many oscillations in unit time.

  Period is the time or distance it takes for sine to start repeating. Time
  it takes to go through 1 full revolution of the unit circle.

    Period = 2pi/b = 360deg/b

  Frequency is the number of occurences of a repeating event per unit time.

    Frequency = events/time = steps/sec

  Example:
    A = 1
    f = 200 steps / 1 sec
    w = 2pi*f
    o = 0
    t = timestamp in sec
    y = A*sin(2pi*f*t + o) = 1*sin(2*pi*(200/1)*t + 0)

  Arguments
  a: amplitude. Always positive.
  x: x axis aka time
  b: period. Number of cycles in 2pi or 360deg
  p: phase shift
  """
  return lambda t: a * math.sin(2*math.pi*f*t + p)

def curve2(a=1, b=1, p=0):
  return lambda x: a * math.sin((x * b) + p)

def reduce_amplitude(a, b):
  if (hasattr(a, '__iter__')):
    a = a[0]
  return a + b[0]

class SinusoidProfile(RampProfile):

  def __init__(self, waves):
    super().__init__()
    # Total lenght of y axis in full steps
    # Equal to 2 x radius of unit circle
    # self._length_full_steps = length_full_steps
    self._last_y = None
    self._last_step = None
    # self._length_steps = length_full_steps * microsteps
    # self._amplitude = self._length_steps / 2
    self._waves = waves
    # Calculate the sum of all amplitudes
    self._sum_amplitudes = functools.reduce(reduce_amplitude, waves)
    # TODO this assumes all amplitudes are the same
    self._diameter_steps = self._waves[0][0] * 2
    self._start_time_sec = time.time()
    self._next_step = None

  def should_step(self):
    # Units of t must match units of f (preferably seconds)
    t = time.time() - self._start_time_sec
    # Calculate y based on time
    y = 0
    for amplitude, frequency, phase_shift in self._waves:
      y += curve1(a=amplitude, f=frequency, p=phase_shift)(t)
    y = project(y, -self._sum_amplitudes, self._sum_amplitudes, 0, self._diameter_steps)
    y = math.ceil(y)
    # log.debug('should_step: y=%s max_amplitude=%s _current_steps=%s',
    #   y, max_amplitude, self._current_steps)
    # Figure out if it is time to step
    if y != self._current_steps:
      self._next_step = y
      return True
    else:
      return False

  def step(self):
    """
    Register a step in the current direction.
    This method is for drivers to notify the profile that they are taking a step.
    """
    self._current_steps = self._next_step
    # else: do nothing
    self.compute_new_speed()
    self._last_step_time_us = micros()

  # FIXME needs to look ahead to next next_step to determine direction
  # otherwise we return DIRECTION_NONE between step() and should_step()
  @property
  def direction(self):
    if self._next_step > self._current_steps:
      return DIRECTION_CW
    elif self._next_step < self._current_steps:
      return DIRECTION_CCW
    else:
      return DIRECTION_NONE
