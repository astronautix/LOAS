"""
Inspired by
 - https://github.com/mikedh/trimesh/blob/master/trimesh/viewer/windowed.py
"""
import ctypes
import os
import numpy as np
import loas
import pyglet
from pyglet.gl import *
from ctypes import *
from pywavefront import visualization
import pywavefront
from math import sin, cos
import trimesh

vehicle_line_length = 1.5
reference_line_length = 0.5

class CustomBatch(pyglet.graphics.Batch):
    def __init__(self):
        super().__init__()

    def add_reference_axis(self,origin = (0,0,0),lineLength=2):
        x,y,z = [float(i) for i in origin]
        self.add_line(origin,(lineLength,0,0),(255,0,0))
        self.add_line(origin,(0,lineLength,0),(0,255,0))
        self.add_line(origin,(0,0,lineLength),(0,0,255))

    def add_line(self, origin=(0,0,0), dir = (1,0,0), color = (255,0,0)):
        x,y,z = [float(i) for i in origin]
        a,b,c = [float(i) for i in dir]
        self.add(2, GL_LINES, None,
            ('v3f', (x,y,z,x+a,y+b,z+c)),
            ('c3B', color*2)
        )

    def add_pyramid(self, pos=(0,0,0), size=.05, color=(255,0,0)):
        x,y,z = [float(i) for i in pos]
        a = (x + size, y + size, z)
        b = (x + size, y - size, z)
        c = (x - size, y - size, z)
        d = (x - size, y + size, z)
        e = (x, y, z + size)
        f = (x, y, z - size)
        self.add(24, GL_TRIANGLES, None,
            ('v3f', (
                *a, *b, *e,
                *a, *b, *f,
                *b, *c, *e,
                *b, *c, *f,
                *c, *d, *e,
                *c, *d, *f,
                *d, *a, *e,
                *d, *a, *f
            )),
            ('c3B', color*24)
        )

class Viewer(pyglet.window.Window):
    def __init__(self, satellite, fps=30):
        super().__init__(resizable=True)
        self.satellite = satellite
        self.fps = fps
        self.rotation = 0
        self.lightfv = ctypes.c_float * 4
        self.cameraPos = {
            "theta": 0,
            "phi": 0,
            "dist": 10
        }
        self.keyboard = pyglet.window.key.KeyStateHandler()
        self.push_handlers(self.keyboard)

        self.unlight_dyn_batch = CustomBatch()
        self.unlight_dyn_batch.add_reference_axis()

        self.light_dyn_batch = CustomBatch()
        self.light_dyn_batch.add_indexed(
            *trimesh.rendering.mesh_to_vertexlist( satellite.mesh )
        )

        self.stat_batch = CustomBatch()
        self.stat_batch.add_reference_axis(origin=(-3,0,0))

        glClearColor(0, 0.3, 0.5, 0)

    def on_resize(self, width, height):
        viewport_width, viewport_height = self.get_framebuffer_size()
        glViewport(0, 0, viewport_width, viewport_height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(60., float(width)/height, 1., 1000.)
        glMatrixMode(GL_MODELVIEW)
        return True


    def on_draw(self):
        self.clear()
        glLoadIdentity()
        glLightfv(GL_LIGHT0, GL_POSITION, self.lightfv(-1.0, 1.0, 1.0, 0.0))
        glEnable(GL_LIGHT0)
        glEnable(GL_DEPTH_TEST)

        # set camera position
        gluLookAt(
            self.cameraPos['dist']*cos(self.cameraPos["phi"])*sin(self.cameraPos['theta']),
            self.cameraPos['dist']*sin(self.cameraPos["phi"]),
            self.cameraPos['dist']*cos(self.cameraPos["phi"])*cos(self.cameraPos['theta']),
            0, 0, 0, 0, 1, 0
        )

        # draw static objects (e.g. ref frames)
        self.stat_batch.draw()

        #glRotatef(self.satellite.Q.angle()*180/3.14, *self.satellite.Q.axis())

        # draw objects that move along with the satellites
        #the ones that are not handled by GL_LIGHTING
        self.unlight_dyn_batch.draw()

        #and the ones that are handled by GL_LIGHTING
        glEnable(GL_LIGHTING)
        self.light_dyn_batch.draw()
        glDisable(GL_LIGHTING)

    def update(self, dt):
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
