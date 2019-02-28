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


import time
import argparse
import importlib
import numpy as np

def make_env(env_id, rank, seed=0):
	def _init():
		env = gym.make(env_id)
		if rank == 0:
			env.enable_viz()
			env.enable_print()
		env.seed(seed + rank)
		return env

	# set_global_seeds(seed)
	return _init


class LazyModel:
	def __init__(self):
		pass

	def predict(self, obs):
		return np.array([[0, 0, 0, 0]]), None


def main(args):
	env_id = 'fwmav-v0'
	env = DummyVecEnv([make_env(env_id, 0)])

	if args.model_type != 'PID':
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
		model = LazyModel()

	obs = env.reset()
	while True:
		action, _ = model.predict(obs)
		obs, rewards, done, info = env.step(action)
		if done:
			obs = env.reset()

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--model_type', required=True)
	parser.add_argument('--model_path')
	parser.add_argument(
		'--policy_type', const='MlpPolicy', default='MlpPolicy', nargs='?')
	args = parser.parse_args()

	main(args)