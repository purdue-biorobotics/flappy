##########################  FWMAV Simulation  #########################
# Version 0.2
# Fan Fei       Mar 2018
# FWMAV simulation with simplified aerodynamics
#######################################################################


from rllab.algos.ddpg import DDPG
from rllab.envs.normalized_env import normalize
from rllab.exploration_strategies.ou_strategy import OUStrategy
from rllab.policies.deterministic_mlp_policy import DeterministicMLPPolicy
from rllab.q_functions.continuous_mlp_q_function import ContinuousMLPQFunction
import lasagne.nonlinearities as NL

from fwmav_sim_env_maneuver import FWMAVSimEnv
import os
import rllab.misc.logger as logger
import pickle
import sys
import lasagne.init as LI

env = normalize(FWMAVSimEnv())
policy = DeterministicMLPPolicy(
	env_spec=env.spec,
	hidden_nonlinearity=NL.tanh,#NL.rectify,LeakyRectify
	output_nonlinearity=NL.tanh,
	hidden_sizes=(32, 32),

)

es = OUStrategy(env_spec=env.spec, theta = 0.15, sigma = 0.3) #theta = decay rate of noise (small decay slower, fluctuate more, theta = 0.01 is about 220 steps, theta = 0.1 is about 20 steps, 0.15 is 15 step, 0.022 is 100 step), sigma = variation or the size of the noise

qf = ContinuousMLPQFunction(
	env_spec=env.spec,
	hidden_nonlinearity=NL.tanh,
	output_nonlinearity=None,
	hidden_sizes=(128, 128),
	output_W_init=LI.Uniform(-3e-6, 3e-6),
	output_b_init=LI.Uniform(-3e-6, 3e-6),
)

algo = DDPG(
		env=env,
		policy=policy,
		es=es,
		qf=qf,
		batch_size=256,                 # Number of samples for each minibatch.
		max_path_length=1500,			# 5 seconds
		epoch_length=15000,              # How many timesteps for each epoch.
		min_pool_size=15000,             # Minimum size of the pool to start training.
		replay_pool_size=15000000,
		n_epochs=1000,                 # Number of epochs. Policy will be evaluated after each epoch.
		eval_samples=15000,              # Number of samples (timesteps) for evaluating the policy.
		discount=1.0,
		scale_reward=0.1,             # The scaling factor applied to the rewards when training
		qf_learning_rate=1e-3,          # Learning rate for training Q function
		policy_learning_rate=1e-4,      # Learning rate for training the policy
		#qf_weight_decay=0.01,
		soft_target_tau=0.005,      # Interpolation parameter for doing the soft target update.
		# Uncomment both lines (this and the plot parameter below) to enable plotting
		# plot=True,
)

log_dir = os.path.join(os.getcwd(),'data')
logger.set_snapshot_dir(log_dir)
logger.add_text_output(os.path.join(log_dir,'debug.log'))
logger.add_tabular_output(os.path.join(log_dir,'progress.csv'))
logger.set_snapshot_mode('last')

algo.train()

# save parameters
with open(os.path.join(log_dir,'final_policy.pkl'), 'wb') as output:
	trained_policy = algo.policy
	pickle.dump(trained_policy, output, pickle.HIGHEST_PROTOCOL)
print('Final policy saved')

def save_large_pickled_object(obj, filepath):
	"""
	This is a defensive way to write pickle.write, allowing for very large files on all platforms
	"""
	max_bytes = 2**31 - 1
	bytes_out = pickle.dumps(obj)
	n_bytes = sys.getsizeof(bytes_out)
	with open(filepath, 'wb') as f_out:
		for idx in range(0, n_bytes, max_bytes):
			f_out.write(bytes_out[idx:idx+max_bytes])

pool = algo.pool
save_large_pickled_object(pool,os.path.join(log_dir,'final_pool.pkl'))
print('Final pool saved')
