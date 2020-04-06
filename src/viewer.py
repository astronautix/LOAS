"""This script shows an example of using the PyWavefront module."""
import ctypes
import os
import numpy as np
from quaternion import Quaternion
import pyglet
from pyglet.gl import *
from ctypes import *
from pywavefront import visualization
import pywavefront
from math import sin, cos

vehicle_line_length = 1.5
reference_line_length = 0.5


def drawReference(lineLength):
    ref_axis = pyglet.graphics.Batch()
    ref_axis.add(2, GL_LINES, None,
        ('v3f', (0.,0.,0.,lineLength,0.,0.)),
        ('c3B', (255,0,0,255,0,0))
    )
    ref_axis.add(2, GL_LINES, None,
        ('v3f', (0.,0.,0.,0.,lineLength,0.)),
        ('c3B', (0,255,0,0,255,0))
    )
    ref_axis.add(2, GL_LINES, None,
        ('v3f', (0.,0.,0.,0.,0.,lineLength)),
        ('c3B', (0,0,255,0,0,255))
    )
    ref_axis.draw()

class Viewer(pyglet.window.Window):
    def __init__(self, modelFile, Qgetter, fps=30):
        super().__init__(resizable=True)
        self.fps = fps
        self.getQ = Qgetter
        self.Q = Quaternion(1,0,0,0)
        self.rotation = 0
        self.meshes = pywavefront.Wavefront(modelFile)
        self.lightfv = ctypes.c_float * 4
        self.cameraPos = {
            "theta": 0,
            "phi": 0,
            "dist": 10
        }
        self.keyboard = pyglet.window.key.KeyStateHandler()
        self.push_handlers(self.keyboard)

        glClearColor(0, 0.3, 0.5, 0)


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
        #glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_DEPTH_TEST)

        gluLookAt(
            self.cameraPos['dist']*cos(self.cameraPos["phi"])*sin(self.cameraPos['theta']),
            self.cameraPos['dist']*sin(self.cameraPos["phi"]),
            self.cameraPos['dist']*cos(self.cameraPos["phi"])*cos(self.cameraPos['theta']),
            0,
            0,
            0,
            0,
            1,
            0
        )

        glTranslated(-1.5,-1.5,0.)
        drawReference(reference_line_length)
        glTranslated(1.5,1.5,0.)

        glRotatef(self.Q.angle()*180/3.14, *self.Q.axis())
        drawReference(vehicle_line_length)
        visualization.draw(self.meshes)

    def update(self, dt):
        #self.Q = self.getQ()
        if self.keyboard[pyglet.window.key.UP]:
            self.cameraPos['phi'] += 0.08
        if self.keyboard[pyglet.window.key.DOWN]:
            self.cameraPos['phi'] -= 0.08
        if self.keyboard[pyglet.window.key.LEFT]:
            self.cameraPos['theta'] -= 0.08
        if self.keyboard[pyglet.window.key.RIGHT]:
            self.cameraPos['theta'] += 0.08
        if self.keyboard[pyglet.window.key.NUM_ADD]:
            self.cameraPos['dist'] -= 0.3
        if self.keyboard[pyglet.window.key.NUM_SUBTRACT]:
            self.cameraPos['dist'] += 0.3

    def run(self):
        pyglet.clock.schedule_interval(self.update, 1/self.fps)
        pyglet.app.run()
