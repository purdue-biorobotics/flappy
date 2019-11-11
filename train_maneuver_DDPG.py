##########################  FWMAV Simulation  #########################
# Version 0.3
# Fan Fei       Feb 2019
# Direct motor driven flapping wing MAV simulation
#######################################################################

import gym
import flappy
import numpy as np
import tensorflow as tf

from stable_baselines.ddpg.policies import FeedForwardPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines.common import set_global_seeds
from stable_baselines.ddpg.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
from stable_baselines import DDPG

import time
import argparse
import importlib

'''
DDPG: Deep Deterministic Policy Gradients.

Class for representing ddpg policy. Used in Q-Learning.

FeedForwardPolicy: A policy that passes the controlling signal from the source to external environment.
'''
class MyDDPGPolicy(FeedForwardPolicy):
	def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=False, **_kwargs):
		super(MyDDPGPolicy, self).__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse,
										act_fun=tf.nn.tanh, layers=[32, 32], feature_extraction="mlp", **_kwargs)

	def make_critic(self, obs=None, action=None, reuse=False, scope="qf"):
		critic_layers = [128, 128]

		if obs is None:
			obs = self.processed_obs
		if action is None:
			action = self.action_ph

		with tf.variable_scope(scope, reuse=reuse):
			qf_h = tf.layers.flatten(obs)

			for i, layer_size in enumerate(critic_layers):
				qf_h = tf.layers.dense(qf_h, layer_size, name='fc' + str(i))
				if self.layer_norm:
					qf_h = tf.contrib.layers.layer_norm(qf_h, center=True, scale=True)
				qf_h = self.activ(qf_h)
				if i == 0:
					qf_h = tf.concat([qf_h, action], axis=-1)

			qvalue_fn = tf.layers.dense(qf_h, 1, name=scope,
										kernel_initializer=tf.random_uniform_initializer(minval=-3e-3,
																						 maxval=3e-3))

			self.qvalue_fn = qvalue_fn
			self._qvalue = qvalue_fn[:, 0]
		return self.qvalue_fn

		
'''
Configures and returns the environment given the arguments.
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

def main(args):

	start = time.time()

	env_id = 'fwmav_maneuver-v0'
	#Making a vector with size 1 that only has the environment.
	env = DummyVecEnv([make_env(env_id, 0)])
	# env = SubprocVecEnv([make_env(env_id, i) for i in range(args.n_cpu)])

	#When the argument is -1, the shape will be found automatically.
	n_actions = env.action_space.shape[-1]
	param_noise = None
	action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions))

	model = DDPG(
			policy = MyDDPGPolicy,
			env = env,
			gamma = 1.0,
			nb_train_steps=5000,
			nb_rollout_steps=10000,
			nb_eval_steps=10000,
			param_noise=param_noise,
			action_noise=action_noise,
			tau=0.003,
			batch_size=256,
			observation_range=(-np.inf, np.inf),
			actor_lr=0.0001,
			critic_lr=0.001,
			reward_scale=0.05,
			memory_limit=10000000,
			verbose=1,
	)

	model.learn(total_timesteps=args.time_step)
	model.save(args.model_path)

	end = time.time()
	print("Time used: ", end - start)


if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--model_path', required=True, nargs='?')
	parser.add_argument('--time_step', required=True, type=int, nargs='?')
	args = parser.parse_args()

	main(args)
