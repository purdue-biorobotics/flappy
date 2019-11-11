##########################  FWMAV Simulation  #########################
# Version 0.3
# Fan Fei		Feb 2019
# Direct motor driven flapping wing MAV simulation
#######################################################################

import gym
import flappy

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines.common import set_global_seeds

from flappy.envs.fwmav.controllers.arc_xy_arc_z import ARCController
from flappy.envs.fwmav.controllers.pid_controller import PIDController

import time
import argparse
import importlib
import numpy as np
'''
Configures and returns the environment.
env_id: Environment type to construct.
rank: Rank is added to seed while generating the environment. Helps to give different seeds in multiprocessing.
seed: The seed used to generate a random environment. 
random_init: Enable random initalization.
randomize_sim: Configure the environment to be randomized.
phantom_sensor: Used in env config.
'''
def make_env(env_id, rank, seed=0, random_init = True, randomize_sim = True, phantom_sensor = False):
	def _init():
		env = gym.make(env_id)
		env.config(random_init, randomize_sim, phantom_sensor)
		if rank == 0:
			env.enable_visualization()
			env.enable_print()
		env.seed(seed + rank)
		return env

	# set_global_seeds(seed)
	return _init

'''
The model used in testing. 
'''
class LazyModel:
	def __init__(self,env,model_type):
		self.action_lb = env.action_lb
		self.action_ub = env.action_ub
		self.observation_bound = env.observation_bound
		self.frequency = 34
		self.env = env
		if model_type == 'PID':
			self.policy = PIDController(env.sim.dt_c)
		elif model_type == 'ARC':
			self.policy = ARCController(env.sim.dt_c)
		else:
			raise Exception('Error')

	def predict(self, obs):
		time = self.env.sim.world.time()
		action = self.policy.get_action(obs[0]*self.observation_bound)

		max_voltage = action[0]
		voltage_diff = action[1]
		voltage_bias = action[2]
		split_cycle = action[3]

		input_voltage = np.zeros([2],dtype=np.float64)
		input_voltage[0] = self.generate_control_signal(self.frequency, max_voltage, voltage_diff, voltage_bias, -split_cycle, time, 0)
		input_voltage[1] = self.generate_control_signal(self.frequency, max_voltage, -voltage_diff, voltage_bias, split_cycle, time, 0)

		# scale action from [action_lb, action_ub] to [-1,1]
		# since baseline does not support asymmetric action space
		normalized_action = (input_voltage-self.action_lb)/(self.action_ub - self.action_lb)*2 - 1
		action = np.array([normalized_action])
		return action, None

	def generate_control_signal(self, f, Umax, delta, bias, sc, t, phase_0):
		V = Umax + delta
		V0 = bias
		sigma = 0.5+sc

		T = 1/f
		t_phase = phase_0/360*T
		t = t+t_phase
		period = np.floor(t/T)
		t = t-period*T

		if 0<=t and t<sigma/f:
			u = V*np.cos(2*np.pi*f*(t)/(2*sigma))+V0
		elif sigma/f<=t and t<1/f:
			u = V*np.cos((2*np.pi*f*(t)-2*np.pi)/(2*(1-sigma)))+V0
		else:
			u=0
		return u

def main(args):
	env_id = 'fwmav_hover-v1'

	env = DummyVecEnv([make_env(env_id, 0, random_init = args.rand_init, randomize_sim = args.rand_dynamics, phantom_sensor = args.phantom_sensor)])

	model = LazyModel(env.envs[0],args.model_type)

	obs = env.reset()

	while True:
		if env.envs[0].is_sim_on == False:
			env.envs[0].gui.cv.wait()
		elif env.envs[0].is_sim_on:
			action, _ = model.predict(obs)
			obs, rewards, done, info = env.step(action)
			if done:
				obs = env.reset()

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--model_type', required=True)
	parser.add_argument('--rand_init', action='store_true', default=False)
	parser.add_argument('--rand_dynamics', action='store_true', default=False)
	parser.add_argument('--phantom_sensor', action='store_true', default=False)

	args = parser.parse_args()

	main(args)
