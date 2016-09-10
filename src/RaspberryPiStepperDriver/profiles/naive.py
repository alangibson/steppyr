import logging

log = logging.getLogger(__name__)

# HACK put this somewhere common to accelstepper too
DIRECTION_CCW = 0   # Clockwise
DIRECTION_CW  = 1   # Counter-Clockwise

class NaiveProfile:
  def set_target_speed(self, speed):
    """
    Set our requested ultimate cruising speed.

    Arguments:
      speed (float): Steps per second
    """
    if speed == self.parent._speed:
      return
    # speed = constrain(speed, -self._target_speed, self._target_speed)
    if speed == 0.0:
      self.parent._step_interval_us = 0
    else:
      self.parent._step_interval_us = self.parent.calc_step_interval_us(speed)
      self.parent._direction = self.parent.calc_direction(speed)
    self.parent._speed = speed
    log.debug('Set speed %s _step_interval_us %s _direction %s',
      speed, self.parent._step_interval_us, self.parent._direction )
    return speed

  def set_acceleration(self, acceleration):
    """
    Ignored since this algorithm doesn't do acceleration.
    """
    pass

  def compute_new_speed(self):
    if self.parent.distance_to_go == 0:
      # We are at the target and its time to stop
      # TODO remove the need to 0 these out. Right now, if we do, then we
      # never stop running.
      # self.parent._step_interval_us = 0
      # self.parent._speed = 0.0
      return

    # TODO not necessary to reset these if we resolve the above TODO
    # self.parent._step_interval_us = self.parent.calc_step_interval_us(self.parent._speed)
    # self.parent._direction = self.parent.calc_direction(self.parent._speed)

    # Figure out Direction
    self.parent._direction = DIRECTION_CW if self.parent.distance_to_go > 0 else DIRECTION_CCW
    # if self.parent._direction == DIRECTION_CCW:
    #  self.parent._speed = -self.parent._speed

    log.debug('Computed new speed. _direction=%s, _current_steps=%s, _target_steps=%s, distance_to_go=%s, _speed=%s, _step_interval_us=%s',
      self.parent._direction, self.parent._current_steps,
      self.parent._target_steps, self.parent.distance_to_go,
      self.parent._speed, self.parent._step_interval_us)

  def set_current_position(self, position):
    self._target_steps = self._current_steps = position
