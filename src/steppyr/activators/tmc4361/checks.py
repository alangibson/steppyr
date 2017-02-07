
OPERATION_MODE_POSITIONING = 1

def check_1(operational_mode, clock_frequency, v_max):
  """
  Do NOT exceed VMAX ≤ fCLK 1/4 pulses for positioning mode
  """
  if operation_mode == OPERATION_MODE_POSITIONING and v_max > ( clock_frequency / 4):
    return False
  else:
    return True

def check_2(operation_mode, v_start, v_stop):
  """
  Use VSTART without setting VSTOP > VSTART only in positioning mode if there
  is enough distance between the current position XACTUAL and the target
  position XTARGET.
  """
  if operation_mode == OPERATION_MODE_POSITIONING and v_stop <= v_start:
    return False
  else
    return True

def check_3():
  """
  Set VBREAK > VSTOP.
  """
  pass
  
def check_4():
  """
  Set VSTART < VSTOP.
  """
  pass
