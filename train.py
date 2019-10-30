##########################  FWMAV Simulation  #########################
# Version 0.3
# Fan Fei       Feb 2019
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

'''
Creates the environment state for training.
env_id: Environment ID for the training.
rank:
seed: Seed for the randomization algorithm in the environment.
random_init: Used in configuring the env object.
randomize_sim: Used in configuring the env object. States whether the simulator
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

    try:
        model_cls = getattr(importlib.import_module(
            'stable_baselines'), args.model_type)
    except AttributeError:
        print(args.model_type, "Error: wrong model type")
        return

    try:
        policy_cls = getattr(importlib.import_module(
            'stable_baselines.common.policies'), args.policy_type)
    except AttributeError:
        print(args.policy_type, "Error: wrong policy type")
        return

    start = time.time()

    env_id = 'fwmav_hover-v0'
    # env = DummyVecEnv([make_env(env_id, 1)])
    env = SubprocVecEnv([make_env(env_id, i) for i in range(args.n_cpu)])

    model = model_cls(policy_cls, env, verbose=0)
    model.learn(total_timesteps=args.time_step)
    model.save(args.model_path)

    end = time.time()
    print("Time used: ", end - start)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model_type', const='PPO2', default='PPO2', nargs='?')
    parser.add_argument('--model_path', required=True, nargs='?')
    parser.add_argument('--policy_type',
                        const='MlpPolicy', default='MlpPolicy', nargs='?')
    parser.add_argument('--n_cpu', const=4, default=4, type=int, nargs='?')
    parser.add_argument('--time_step', required=True, type=int, nargs='?')
    args = parser.parse_args()

    main(args)
