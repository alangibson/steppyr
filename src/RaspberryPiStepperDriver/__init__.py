import time

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
