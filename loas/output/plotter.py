import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np

from .output import Output

class Plotter(Output):
    def __init__(self, interval = 1/5):
        super().__init__()
        self.ts = [0]
        self.ws = [0]
        self.interval = interval

    def update(self, **kwargs):
        if 'satellite' in kwargs:
            self.ts.append(self.ts[-1]+kwargs['satellite'].dt)
            self.ws.append(np.linalg.norm(kwargs['satellite'].W))

    def _animate(self, i):
        plt.clf()
        plt.subplot(1,1,1)
        plt.plot(self.ts, self.ws)

    def run(self):
        ani = animation.FuncAnimation(plt.figure(), self._animate, interval=self.interval)
        plt.show()
