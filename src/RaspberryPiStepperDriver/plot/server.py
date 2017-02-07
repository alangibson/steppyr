from datetime import datetime
from bokeh.server.server import Server
from bokeh.plotting import figure, curdoc
from bokeh.driving import linear
import random

"""
To run and open in web browser:
  pip3 install bokeh
  bokeh serve --show  src/RaspberryPiStepperDriver/plot/server.py
"""

class Plot:

  def __init__(self):
    self._last_position = None

  async def plot(self, driver):
    # driver.distance_to_go
    # driver.acceleration
    if driver.position != self._last_position:
      self._last_position = driver.position

#server = Server(
#    bokeh_applications,  # list of Bokeh applications
#    io_loop=loop,        # Tornado IOLoop
#    **server_kwargs      # port, num_procs, etc.
#)

min_position, max_position = 0, 100

p = figure(plot_width=800, plot_height=400, y_range=(min_position, max_position), webgl=True)
r1 = p.line([], [], color="firebrick", line_width=2)
r2 = p.line([], [], color="navy", line_width=2)
bar = p.vbar(x=[-1], width=1, bottom=0, top=[max_position], color="#CAB2D6")

ds1 = r1.data_source
ds2 = r2.data_source

@linear()
def update(step):
    ds1.data['x'].append(step)
    ds1.data['y'] = ds1.data['y'][max(0, len(ds1.data['y'])-9):]+[random.randint(min_position,max_position)]
    ds2.data['x'].append(step)
    ds2.data['y'] = ds2.data['y'][max(0, len(ds2.data['y'])-9):]+[random.randint(min_position,max_position)]
    ds1.trigger('data', ds1.data, ds1.data)
    ds2.trigger('data', ds2.data, ds2.data)
    bar.data_source.data['top'] =[ds1.data['y'][-1]]
    print(bar.data_source['top'])

curdoc().add_root(p)

# Add a periodic callback to be run every 500 milliseconds
curdoc().add_periodic_callback(update, 200)
