import asyncio, threading

import logging

import math
from bokeh.plotting import figure, curdoc
from bokeh.driving import linear
from bokeh.client import push_session
import random

from steppyr.planners.sinusoid import StepperWave, SinusoidPlan

"""
To run and open in web browser:
  pip3 install bokeh
  bokeh serve
"""

logging.basicConfig(level=logging.INFO)
log = logging.getLogger(__name__)

sliding_window_size = 100
refresh_rate_hz = 20
refresh_rate_ms = 1000/refresh_rate_hz
refresh_rate_sec = 1/refresh_rate_hz
min_position, max_position = 0, 300

class MockController:
  def move_to(self, position):
    log.debug('MockController.move_to(%s)', position)

# Length of slider in full steps aka diameter of unit circle aka length of 2 radians
length_full_steps = max_position
# Target speed in full steps per second
target_speed = 500
waves = [
  StepperWave(length_full_steps, target_speed * 0.5),
  StepperWave(length_full_steps, target_speed * 0.75),
  StepperWave(length_full_steps, target_speed, phase_shift=math.radians(270) + 1)
]
planner = SinusoidPlan(waves=waves, controller=MockController(), sample_rate=refresh_rate_sec)

plot = figure(plot_width=800, plot_height=600, y_range=(min_position, max_position), webgl=True)
line1 = plot.line([], [], color="firebrick", line_width=2)
bar = plot.vbar(x=[-1], width=1, bottom=0, top=[max_position], color="#CAB2D6")

@linear()
def update(step):

  last_y = planner._last_y
  # last_y = random.randint(min_position,max_position)

  ds1 = line1.data_source
  if len(ds1.data['x']) >= sliding_window_size:
    ds1.data['x'].pop(0)
  ds1.data['x'].append(planner._last_x)
  if len(ds1.data['y']) >= sliding_window_size:
    ds1.data['y'].pop(0)
  ds1.data['y'].append(last_y)
  ds1.trigger('data', ds1.data, ds1.data)

  ds_bar = bar.data_source
  ds_bar.data['top'] =[ds1.data['y'][-1]]
  # Vertical bar is centered over first x value, shifted left by 1
  ds_bar.data['x'] = [ds1.data['x'][0] - 1]
  ds_bar.trigger('data', ds_bar.data, ds_bar.data)

curdoc().add_root(plot)

# open a session to keep our local document in sync with server
session = push_session(curdoc())
curdoc().add_periodic_callback(update, refresh_rate_ms)  # period in ms

bokeh_thread = threading.Thread(target=session.loop_until_closed)

loop = asyncio.get_event_loop()
planner_future = asyncio.ensure_future(planner.run_forever())
try:
  log.info('Starting main program loop')
  session.show()  # open the document in a browser
  bokeh_thread.start()
  loop.run_forever()
except:
  log.info('Shutting down loop')
  session.close()
  loop.close()