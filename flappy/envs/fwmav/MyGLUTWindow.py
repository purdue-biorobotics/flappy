##########################  FWMAV Simulation  #########################
# Version 0.3
# Ruoyu Wu, Fan Fei		Feb 2019
# Direct motor driven flapping wing MAV simulation
#######################################################################

from flappy.envs.fwmav import pydart2 as pydart
from flappy.envs.fwmav.pydart2.gui.glut.window import GLUTWindow
from flappy.envs.fwmav.pydart2.gui.trackball import Trackball
from threading import Thread, Condition
import OpenGL.GLUT as GLUT
import sys


class MyGLUTWindow(GLUTWindow):
	def __init__(self, world, sim, env, title, cv):
		self.sim_ob = sim
		self.env = env
		self.cv = cv
		super().__init__(world, title)
		self.window_size = (800, 600)

	def keyPressed(self, key, x, y):
		keycode = ord(key)
		key = key.decode('utf-8')

		if keycode == 27:
			GLUT.glutDestroyWindow(self.window)
		elif key == 'q':
			GLUT.glutDestroyWindow(self.window)
		elif key == 'r':
			self.env.reset()
		elif key == ' ':
			self.env.is_sim_on = not self.env.is_sim_on
	

	def idle(self, ):
		pass

	def drawGL(self, ):
		self.cv.acquire()

		# rendering
		self.scene.render(self.sim)
		GLUT.glutSwapBuffers()

		self.cv.notify()
		self.cv.release()

class GUI(Thread):
	def __init__(self, env, title):
		Thread.__init__(self)
		self.cv = Condition()
		self.title = title
		self.env = env
		self.sim = env.sim
		self.win = MyGLUTWindow(self.sim.world, self.sim, self.env, self.title, self.cv)

		self.camera_theta = 75#, for mirror 85
		self.camera_phi = 135#, for mirror -75
		self.camera_horizontal = 0.0
		self.camera_vertical = -0.25
		self.camera_depth = -1.25
		self.camera_0 = Trackball(theta = self.camera_theta, phi = self.camera_phi, trans=[self.camera_horizontal, self.camera_vertical, self.camera_depth])
		self.win.scene.replace_camera(0,self.camera_0)

	def run(self):
		pydart.gui.viewer.launch_window(self.sim.world, self.win, 0)


