"""This script shows an example of using the PyWavefront module."""
import ctypes
import os
import numpy as np
from quaternion import Quaternion
import pyglet
from pyglet.gl import *
from pywavefront import visualization
import pywavefront


class Viewer(pyglet.window.Window):
    def __init__(self, modelFile, Qgetter, fps=30):
        super().__init__(resizable=True)

        self.fps = fps
        self.getQ = Qgetter
        self.Q = Quaternion(1,0,0,0)
        self.rotation = 0
        self.meshes = pywavefront.Wavefront(modelFile)
        self.lightfv = ctypes.c_float * 4


    def on_resize(self, width, height):
        viewport_width, viewport_height = self.get_framebuffer_size()
        glViewport(0, 0, viewport_width, viewport_height)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(60., float(width)/height, 1., 100.)
        glMatrixMode(GL_MODELVIEW)
        return True


    def on_draw(self):
        self.clear()
        glLoadIdentity()

        glLightfv(GL_LIGHT0, GL_POSITION, self.lightfv(-1.0, 1.0, 1.0, 0.0))
        glEnable(GL_LIGHT0)

        glTranslated(0.0, 0.0, -3.0)
        glRotatef(self.Q.angle()*180/3.14, *self.Q.axis())
        glEnable(GL_LIGHTING)

        visualization.draw(self.meshes)

    def update(self, dt):
        self.Q = self.getQ()

    def run(self):
        pyglet.clock.schedule_interval(self.update, 1/self.fps)
        pyglet.app.run()
