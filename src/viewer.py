from threading import Thread
from quaternion import Quaternion
import pi3d
import numpy as np

class Viewer():
    def __init__(self, modelFile, Qgetter, fps = 30):
        self.running = False
        self.getQ = Qgetter

        # Setup display and initialise pi3d
        self.display = pi3d.Display.create(x=100, y=100, frames_per_second=fps)
        self.display.set_background(0,0,0,1)    	# r,g,b,alpha
        pi3d.Light(lightpos=(1.0, 0.0, 0.1))

        # load model and texture
        self.model = pi3d.Model(
          file_string=modelFile,
          name="Teapot", sx=1, sy=1, sz=1, z=5.8
        )

        # set model display properties
        self.model.set_shader(pi3d.Shader("uv_light"))
        self.model.set_material((3,3,3))

        # create the object that will handle keyboard
        self.keys = pi3d.Keyboard()

    def stop(self):
        self.running = False

    def run(self):
        self.running = True
        while self.running and self.display.loop_running():
            # Actualisation de l'affichage graphique
            Q = self.getQ()
            self.model.rotate_to_direction(Q.V2R(np.array([[0.],[0.],[1.]]))[:,0])
            self.model.draw()

            #Press ESCAPE to terminate
            k = self.keys.read()
            if k > -1:
                if k == 27:    #Escape key
                    self.keys.close()
                    self.display.destroy()
                    self.stop()
        self.stop()
