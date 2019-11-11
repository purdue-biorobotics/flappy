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
Contructing the environment our agent is going ti interact with.
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
The lazy model class. 

Properties
---------

action_lb: Lower bound for the action.
action_ub: Upper bounf for the action.
observation_bound: The range of the possible observations.
policy: The policy used for decision making.


'''

class LazyModel:
	def __init__(self,env,model_type):
		self.action_lb = env.action_lb
		self.action_ub = env.action_ub
		self.observation_bound = env.observation_bound
		if model_type == 'PID':
			self.policy = PIDController(env.sim.dt_c)
		elif model_type == 'ARC':
			self.policy = ARCController(env.sim.dt_c)
		else:
			raise Exception('Error')
	#Predict based on policy and observation.
	def predict(self, obs):
		action = self.policy.get_action(obs[0]*self.observation_bound)
		# scale action from [action_lb, action_ub] to [-1,1]
		# since baseline does not support asymmetric action space
		normalized_action = (action-self.action_lb)/(self.action_ub - self.action_lb)*2 - 1
		action = np.array([normalized_action])
		return action, None

def main(args):
	env_id = 'fwmav_hover-v0'

	env = DummyVecEnv([make_env(env_id, 0, random_init = args.rand_init, randomize_sim = args.rand_dynamics, phantom_sensor = args.phantom_sensor)])

	if args.model_type != 'PID' and args.model_type != 'ARC':
		try:
			model_cls = getattr(
				importlib.import_module('stable_baselines'), args.model_type)
		except AttributeError:
			print(args.model_type, "Error: wrong model type")
			return
		try:
			model = model_cls.load(args.model_path)
		except:
			print(args.model_path, "Error: wrong model path")
	else:
		model = LazyModel(env.envs[0],args.model_type)

	obs = env.reset()

	while True:
		if env.envs[0].is_sim_on == False:
			env.envs[0].gui.cv.wait()
		elif env.envs[0].is_sim_on:
			action, _ = model.predict(obs)
			obs, rewards, done, info = env.step(action)
			# if done:
			# 	obs = env.reset()

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--model_type', required=True)
	parser.add_argument('--model_path')
	parser.add_argument(
		'--policy_type', const='MlpPolicy', default='MlpPolicy', nargs='?')
	parser.add_argument('--rand_init', action='store_true', default=False)
	parser.add_argument('--rand_dynamics', action='store_true', default=False)
	parser.add_argument('--phantom_sensor', action='store_true', default=False)

	args = parser.parse_args()

	main(args)
