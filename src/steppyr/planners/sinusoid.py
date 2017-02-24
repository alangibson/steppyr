import asyncio, functools, logging, math, time
from steppyr.lib.functions import  project, sign

log = logging.getLogger(__name__)

def reduce_amplitude(amplitude_a, wave_b):
  if hasattr(amplitude_a, 'amplitude'):
    amplitude_a = amplitude_a.amplitude
  return amplitude_a + wave_b.amplitude

def wave(t, a=1, f=1, p=0):
  """
  Arguments
    a: amplitude. Always positive.
    t: x axis aka time
    f: frequency
    p: phase shift

  Implements the general sine function
    y(t) = A*sin(2πft + o) = A*sin(wt + o)

  A = the amplitude, the peak deviation of the function from zero.
  f = Hz = the ordinary frequency, the number of oscillations/cycles that occur each second of time.
  w = 2πf = the angular frequency, the rate of change of the function argument in units of radians per second.
  o = the phase, specifies (in radians) where in its cycle the oscillation is at t = 0.
      When o is non-zero, the entire waveform appears to be shifted in time by the amount o/w seconds.
      A negative value represents a delay, and a positive value represents an advance.
  x = t = time, distance or angle

  Period is the time or distance it takes for sine to start repeating.
  Time it takes to go through 1 full revolution of the unit circle.

    Period = 1/f

  Frequency is the number of occurrences of a repeating event per unit time.

    Frequency = events/time = steps/sec

  Example:
    A = 1
    f = 200 steps / 1 sec
    o = 0
    t = timestamp in sec
    y(t) = A*sin(2pi*f*t + o) = 1*sin(2*pi*(200/1)*t + 0)
  """
  return a * math.sin(2*math.pi*f*t + p)

class Wave:
  """
  Implements configuration of a standard sine wave.
  """
  def __init__(self, a=1, f=1, p=0):
    self.amplitude = a
    self.frequency = f
    self.phase = p
  def __call__(self, t):
    return wave(t, self.amplitude, self.frequency, self.phase)

class StepperWave(Wave):
  def __init__(self, length_full_steps, target_speed, phase_shift=math.radians(270)):
    """
    :param length_full_steps: Length of slider in full steps aka diameter of unit circle aka length of 2 radians
    :param target_speed: Target speed in full steps per second
    :param phase_shift: By default, Phase shift by 45 so we always start at the lowest point of negative amplitude
    """
    amplitude = length_full_steps / 2
    # Set frequency based on target_speed
    # frequency = target_speed = events/time
    # Since 1 period/full oscillation traverses the y axis 2 times:
    #   f = 1 hz = (length_full_steps*2) steps/1 sec
    # We can do this with full steps since the ratio is the same with microstepping
    # frequency = target_speed / ( length_full_steps * 2 )
    # Since max amplitude == radius (i.e. length_full_steps/2), we multiply amplitude by 4
    frequency = target_speed / (amplitude * 4)
    super().__init__(a=amplitude, f=frequency, p=phase_shift)

class SinusoidPlan:

  def __init__(self, waves, controller, sample_hz=0, sample_peaks=False):
    """
    :param waves:
    :param controller:
    :param sample_hz: in seconds. 1/Hz
    """
    # An initialized instance of StepController
    self._controller = controller
    # Total length of y axis in full steps
    # Equal to 2 x radius of unit circle
    # self._length_full_steps = length_full_steps
    self._last_y = 0
    # Last timepoint
    self._last_x = 0
    # self._length_steps = length_full_steps * microsteps
    # self._amplitude = self._length_steps / 2
    self._waves = waves
    # Calculate the sum of all amplitudes
    self._sum_amplitudes = functools.reduce(reduce_amplitude, waves)
    # TODO this assumes all amplitudes are the same
    self._diameter_steps = self._waves[0].amplitude * 2
    self._start_time_sec = time.time()
    self._sample_hz = sample_hz
    self._last_direction = None
    self._stopped = False
    self._sample_peaks = sample_peaks

  def _y(self):
    # Units of t must match units of f (preferably seconds)
    t = time.time() - self._start_time_sec
    self._last_x = t
    # Calculate y based on time
    y = 0
    for wave_callable in self._waves:
      y += wave_callable(t)
    y = project(y, -self._sum_amplitudes, self._sum_amplitudes, 0, self._diameter_steps)
    y = math.ceil(y)
    return y

  async def run(self):
    y = self._y()
    # Move based on last y position
    # should_move = y != self._last_y
    # Figure out direction and if we have gone through a crest or trough
    if y == self._last_y:
      return False
    direction = sign(y - self._last_y)
    # See if we should sample only peaks and valleys
    if self._sample_peaks:
      should_move = direction != self._last_direction
    else:
      should_move = True
    # Remember last y position and direction
    self._last_y = y
    self._last_direction = direction
    # Figure out if it is time to step
    if should_move:
      log.debug('y %s _last_y %s direction %s _last_direction %s should_move %s',
                y, self._last_y, direction, self._last_direction, should_move)
      await self._controller.move_to(y)

  async def run_forever(self):
    self._stopped = False
    while not self._stopped:
      await asyncio.sleep(1 / self._sample_hz if self._sample_hz else 0)
      await self.run()

  def stop(self):
    self._stopped = True