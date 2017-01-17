import time

DIRECTION_NONE = -1 # Stationary
DIRECTION_CCW = 0   # Clockwise
DIRECTION_CW  = 1   # Counter-Clockwise

def sleep_microseconds(us_to_sleep):
  time.sleep(us_to_sleep / float(1000000))

def constrain(value, minn, maxn):
  return max(min(maxn, value), minn)

def micros():
  """
  Mymics the Arduino micros() function.
  """
  return int(time.time() * 1000000)

def project(x, in_min, in_max, out_min, out_max):
  """
  Mymics the arduino map function
  """
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Extract sign of a variable
def sign(val):
  if val < 0:
    return -1
  else:
    return 1
