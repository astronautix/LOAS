import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np
import math

from .output import Output

plt.rcParams["figure.figsize"] = (15,6)

class Plotter(Output):
    def __init__(self, interval = 1/5):
        super().__init__()
        self.ts = [0]
        self.ws = [0]
        self.data = {}
        self.interval = interval

    def update(self, t, **kwargs):
        if 'satellite' in kwargs:
            self._save_data(
                'satellite_rotation_speed',
                t,
                np.linalg.norm(kwargs['satellite'].W)
            )

        if 'parasite_drag' in kwargs:
            self._save_data(
                'parasite_drag',
                t,
                kwargs['parasite_drag']
            )

    def _save_data(self, name, t, y):
        if not name in self.data:
            self.data[name] = ([], [])
        self.data[name][0].append(t)
        self.data[name][1].append(y)

    def _animate(self, i):
        plt.clf()
        nb_graphs = len(self.data)
        cols = math.ceil(nb_graphs**(1/2))
        rows = math.ceil(nb_graphs/cols)
        for noitem, (name, values) in enumerate(self.data.items()):
            ax = plt.subplot(rows,cols,noitem+1)
            ax.plot(values[0], values[1])
            ax.set_title(name)

    def run(self):
        ani = animation.FuncAnimation(plt.figure(), self._animate, interval=self.interval)
        plt.show()
