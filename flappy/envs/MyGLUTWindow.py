import pydart2 as pydart
from pydart2.gui.glut.window import GLUTWindow
import threading
from threading import Thread

class MyGLUTWindow(GLUTWindow):
	def __init__(self, world, sim, title):
		self.sim_ob = sim
		self.lock = threading.Lock()
		super().__init__(world, title)

	def keyPressed(self, key, x, y):
		keycode = ord(key)
		key = key.decode('utf-8')

		if keycode == 27:
			GLUT.glutDestroyWindow(self.window)
		elif key == 'q':
			GLUT.glutDestroyWindow(self.window)
		elif key == 'r':
			self.sim_ob.reset()
		elif key == ' ':
			self.lock.acquire()
			try:
				self.sim_ob.sim_on = not self.sim_ob.sim_on
			finally:
				self.lock.release()

	def idle(self, ):
		pass


class GUI(Thread):
	def __init__(self, sim):
		Thread.__init__(self)
		self.sim = sim
		self.win = MyGLUTWindow(self.sim.world, self.sim, None) 

	def run(self):
		pydart.gui.viewer.launch_window(self.sim.world, self.win, None)

