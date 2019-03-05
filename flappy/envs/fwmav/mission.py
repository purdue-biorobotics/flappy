##########################  FWMAV Simulation  #########################
# Version 0.2
# Fan Fei       Mar 2018
# Quadrotor simulation with simplified aerodynamics
#######################################################################

import numpy as np

class Mission:
	def __init__(self):
		self.pos_target_x_ = 0
		self.pos_target_y_ = 0
		self.pos_target_z_ = 0

	def reset(self):
		self.pos_target_x_ = 0
		self.pos_target_y_ = 0
		self.pos_target_z_ = 0

	def update_waypoint(self,time):
		# update mission here
		self.pos_target_x_ = 0
		self.pos_target_y_ = 0
		self.pos_target_z_ = 0