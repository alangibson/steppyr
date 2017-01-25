import functools, math, time

def curve1(a=1, f=1, p=0):
  return lambda t: a * math.sin(2*math.pi*f*t + p)

def curve2(a=1, b=1, p=0):
  return lambda x: a * math.sin((x * b) + p)

def reduce_amplitude(a, b):
  if (hasattr(a, '__iter__')):
    a = a[0]
  return a + b[0]

class SinusoidProfile:

  def __init__(self):
    self._target_speed =

    self._last_time = time.time()
    self._length_steps
    self._last_y =
    self._last_step =

  def should_step(self):
    # Units of t must match units of f (preferably seconds)
    t = time.time() - self._last_time
    # Calculate the sum of all amplitudes
    max_amplitude = functools.reduce(reduce_amplitude, self._curves)
    # Calculate y based on time
    y = 0
    for amplitude, frequency, phase_shift in self._curves:
      y += curve1(a=amplitude, f=frequency, p=phase_shift)(t)
    y = interpolate(y, -max_amplitude, max_amplitude, 0, self._length_steps)
    y = math.ceil(y)
    # Figure out if it is time to step
    last_y = self._last_y
    self._last_y = y
    if y != last_y:
      self._last_step = y
      return True
    else:
      return False
