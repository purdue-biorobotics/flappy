##########################  FWMAV Simulation  #########################
# Version 0.3
# Fan Fei		Feb 2019
# Direct motor driven flapping wing MAV simulation
#######################################################################

import numpy as np

from flappy.envs.fwmav.fwmav_sim_env import FWMAVSimEnv
import time

class FWMAVSimEnvSimple(FWMAVSimEnv):
	def __init__(self):
		super().__init__()
		self.action_lb = np.array([-18.0, -18.0])
		self.action_ub = np.array([18.0, 18.0])
		self.total_action_lb = np.array([-18.0, -18.0])
		self.total_action_ub = np.array([18.0, 18.0])
		self.action_old = np.array([0, 0])

		self.random_init = False


	def step(self, action):
		# scale action from [-1,1] to [action_lb, action_ub]
		# since baseline does not support asymmetric action space
		scaled_action = (action+1)*0.5*(self.action_ub-self.action_lb)+self.action_lb
		scaled_action = np.clip(scaled_action, self.action_lb, self.action_ub)

		input_voltage = np.clip(scaled_action, self.total_action_lb, self.total_action_ub)

		# run simulation and down sample from f_sim to f_control
		while self.sim.world.time() < self.next_control_time:
			self.sim.step(input_voltage)
		self.next_control_time += self.sim.dt_c

		# update control target
		self.mission.update_waypoint(self.sim.world.time())

		# update observation, reward, terminal
		self.observation = self.get_observation()
		reward = self.get_reward(action)
		done = self.get_terminal()

		# visulization
		if self.is_visual_on and time.time() >= self.next_visualization_time:
			self.gui.cv.wait()
			self.next_visualization_time = time.time() + self.dt_v

		info = {'time' : self.sim.world.time()}
		return self.observation, reward, done, info
