import ctypes
import os
import numpy as np
import pyglet
from pyglet.gl import *
from ctypes import *
from math import sin, cos
import trimesh

import loas
from .output import Output

pyglet.options['graphics_vbo'] = 0


class Viewer(Output):
    """
    loas.output.Output inheriting pyglet wrapper
    """

    def __init__(self, mesh, draw_frames = True):
        super().__init__()
        self.base = BaseViewer()

        self.draw_frames = draw_frames
        if draw_frames:
            tmp = CustomBatch()
            tmp.add_reference_axis(origin=(-3,0,0))
            self.base.set_batch("static_frame", tmp)

            tmp = CustomBatch()
            tmp.add_reference_axis()
            self.base.set_batch("vehicle_frame", tmp)

        tmp = CustomBatch( gl_lightning = True )
        tmp.add_mesh(mesh)
        self.base.set_batch("vehicle_mesh", tmp)


    def update(self, **kwargs):

        if 'satellite' in kwargs:
            self.base.get_batch('vehicle_mesh').set_Q(kwargs['satellite'].Q)
            if self.draw_frames:
                self.base.get_batch('vehicle_frame').set_Q(kwargs['satellite'].Q)

        if 'parasite_torque' in kwargs:
            batch = CustomBatch()
            batch.add_line(origin=(0,0,0), dir=kwargs['parasite_torque'][:,0]/100, color=(255,255,255))

            if 'parasite_particle_data' in kwargs and 'satellite_speed' in kwargs:
                for origin, location in kwargs['parasite_particle_data']:
                    batch.add_line(origin, kwargs['satellite_speed'][:,0])
                    batch.add_pyramid(location[:,0])

            self.base.set_batch('parasite', batch)

    def run(self):
        self.base.run()


class BaseViewer(pyglet.window.Window):
    """
    Extends pyglet Window class, handles 3D graphics

    Can be used directly as loas.Viewer

    Inspired by https://github.com/mikedh/trimesh/blob/master/trimesh/viewer/windowed.py
    """

    def __init__(self, fps=30):
        """
        :param satellite: The used satellite instance for the simulation
        :type satellite: loas.Satellite
        :param fps: Frames per second
        :type fps: int
        """

        self.fps = fps
        self.rotation = 0
        self.lightfv = ctypes.c_float * 4
        self.cameraPos = {
            "theta": 0,
            "phi": 0,
            "dist": 10
        }
        self.keyboard = pyglet.window.key.KeyStateHandler()

        self.batches = {}

    def run(self):
        """
        Launches the viewer
        """
        super().__init__(700,700,resizable=True) #create the window
        self.push_handlers(self.keyboard)
        glClearColor(0, 0.3, 0.5, 0)

        pyglet.clock.schedule_interval(self.update, 1/self.fps)
        pyglet.app.run()

    def set_batch(self, name, batch):
        """
        Handy way to display dynamic batches from the outside of the class.

        Every batch has a name (it is the key of the batch), and calling this method will override the exisitng batch of the same name with the newly provided one (if exisiting), otherwise it will add it.

        :param name: Name to give to the batch
        :type name: str
        :param batch: Batch to add
        :type batch: loas.output.viewer.CustomBatch
        """

        self.batches[name] = batch

    def get_batch(self, name):
        if name not in self.batches:
            return None
        else:
            return self.batches[name]

    def on_resize(self, width, height):
        """
        Required by pyglet
        """
        viewport_width, viewport_height = self.get_framebuffer_size()
        glViewport(0, 0, viewport_width, viewport_height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(60., float(width)/height, 1., 1000.)
        glMatrixMode(GL_MODELVIEW)
        return True

    def on_draw(self):
        """
        Required by pyglet, defines how to draw the visuals
        """
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

        for batch in self.batches.values():
            batch.draw()

    def update(self, dt):
        """
        Called at every new frame, is used to update visual values (e.g. camera position)
        """

        self.batch_buffer = []

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



class CustomBatch(pyglet.graphics.Batch):
    """
    Extends pyglet's Batch class for our purpose (makes simple to display some objects)
    """

    def __init__(self, gl_lightning=False, attitude_getter=None):
        """
        :param gl_lightning: If GL_LIGHTING has to be enabled or not during drawing
        :type gl_lightning: bool
        :param attitude_getter: A getter that sets the orientation of the batch. If set to None, the batch will be static.
        :type attitude_getter: function () -> loas.utils.Quaternion
        """

        super().__init__()
        self.Q = loas.utils.Quaternion(1,0,0,0)
        self.gl_lightning = gl_lightning
        self.hidden = False

    def set_Q(self, Q):
        self.Q = Q

    def add_mesh(self, mesh):
        """
        Add mesh to the batch

        :param mesh: The trimesh instance that has to be added to the batch
        :type mesh: trimesh.Trimesh
        """

        self.add_indexed(
            *trimesh.rendering.mesh_to_vertexlist( mesh )
        )

    def add_reference_axis(self,origin = (0,0,0),lineLength=2):
        """
        Add a frame of reference to the visual.

        :param origin: Origin of the frame of reference
        :type origin: (3,) float
        :param lineLength: Length of the drawn lines
        :type lineLength: float
        """

        x,y,z = [float(i) for i in origin]
        self.add_line(origin,(lineLength,0,0),(255,0,0))
        self.add_line(origin,(0,lineLength,0),(0,255,0))
        self.add_line(origin,(0,0,lineLength),(0,0,255))

    def add_line(self, origin=(0,0,0), dir=(1,0,0), color = (255,0,0)):
        """
        Add a simple line to the visual

        :param origin: Origin of the segment
        :type origin: (3,) float
        :param dir: Direction of the segment
        :type dir: (3,) float
        :param color: RGB color of the drawn line
        :type color: (3,) float
        """
        x,y,z = [float(i) for i in origin]
        a,b,c = [float(i) for i in dir]
        self.add(2, GL_LINES, None,
            ('v3f', (x,y,z,x+a,y+b,z+c)),
            ('c3B', color*2)
        )

    def add_pyramid(self, pos=(0,0,0), size=.05, color=(255,0,0)):
        """
        Create a simple diamon-shaped pyramid in the visual

        :param pos: Position of the pyramid
        :type pos: (3, float)
        :param size: Size of the pyramid
        :type size: float
        :param color: RGB color of the pyramid
        :type color: (3,) float
        """
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

    def hide(self):
        """
        Disables draw method of the instance, the object will disappear at the next frame
        """

        self.hidden = True

    def draw(self):
        """
        Wrapper of pyglet batch draw() method. Sets the gl_lightning (if needed), and change the orientation.
        """

        if self.hidden:
            return
        if self.gl_lightning:
            glEnable(GL_LIGHTING)
        else:
            glDisable(GL_LIGHTING)

        glPushMatrix() # saves the current projection matrix
        glRotatef(self.Q.angle()*180/3.14, *self.Q.axis())
        super().draw()
        glPopMatrix() #uses back initial projection matrix (revert rotation)
