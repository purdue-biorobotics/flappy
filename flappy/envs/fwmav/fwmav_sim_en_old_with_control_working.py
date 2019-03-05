##########################  FWMAV Simulation  #########################
# Version 0.3
# Fan Fei		Feb 2019
# Direct motor driven flapping wing MAV simulation
#######################################################################

import gym
from gym.spaces import Box
import numpy as np
import json

from flappy.envs.fwmav.simulation import Simulation
# from flappy.envs.controllers.pid_controller import PIDController
from flappy.envs.fwmav.controllers.arc_xy_arc_z import PIDController
from flappy.envs.fwmav.mission import Mission

# GUI
from flappy.envs.fwmav.MyGLUTWindow import GUI
import time

class FWMAVSimEnv(gym.Env):
	def __init__(self):
		with open ('./flappy/envs/fwmav/config/sim_config.json') as file:
			sim_config = json.load(file)
		with open('./flappy/envs/fwmav/config/mav_config_list.json') as file:
			mav_config_list = json.load(file)
		self.sim = Simulation(mav_config_list, sim_config)
		self.observation = np.zeros([18], dtype=np.float64)
		self.observation_bound = np.array([
			1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
			1.0, 1.0, 1.0, 1.0, 1.0
		])

		self.action_lb = np.array([-5, -3, -3.5, -0.15])
		self.action_ub = np.array([11, 3, 3.5, 0.15])
		self.total_action_lb = np.array([0, -3, -3.5, -0.15])
		self.total_action_ub = np.array([18.0, 3, 3.5, 0.15])
		self.action_old = np.array([0, 0, 0, 0])
		self.pid_policy = PIDController(self.sim.dt_c)
		self.mission = Mission(self.pid_policy)

		self.done = False
		self.reward = 0
		
		self.is_sim_on = False
		self.is_print_on = False
		self.is_visual_on = False
		self.dt_v = 1/sim_config['f_visual']

	def enable_visualization(self):
		self.is_visual_on = True
		self.next_visualization_time = time.time()

		self.gui = GUI(self, "FWMAV")
		self.gui.cv.acquire()
		print("init GUI")
		self.gui.start()

	def enable_print(self):
		self.is_print_on = True

	@property
	def observation_space(self):
		return Box(np.array([-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf]), np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf]))

	@property
	def action_space(self):
		return Box(np.array([-5.0, -3, -3.5, -0.15]), np.array([11.0, 3, 3.5, 0.15]))

	def reset(self):
		# rpy_limit = 0.785398 	# 45 deg
		# pqr_limit = 3.14159		# 180 deg/s
		# xyz_limit = 0.1			# 10 cm
		# xyz_dot_limit = 0.1		# 10cm/s
		rpy_limit = 0.0 	# 45 deg
		pqr_limit = 0.0		# 180 deg/s
		xyz_limit = 0.0			# 10 cm
		xyz_dot_limit = 0.0		# 10cm/s

		positions = np.zeros([10,1], dtype=np.float64)
		velocities = np.zeros([10,1], dtype=np.float64)

		init_attitude = np.random.uniform(-rpy_limit, rpy_limit, size=(3,1))
		init_angular_velocity = np.random.uniform(-pqr_limit, pqr_limit, size=(3,1))
		init_position = np.random.uniform(-xyz_limit, xyz_limit, size=(3, 1))
		init_body_velocity = np.random.uniform(-xyz_dot_limit, xyz_dot_limit, size=(3,1))

		positions[0:3] = init_attitude
		positions[3:6] = init_position
		velocities[0:6] = np.concatenate((init_angular_velocity,init_body_velocity), axis = 0)

		self.sim.reset()
		self.sim.set_states(positions, velocities)
		self.observation = self.get_observation()
		self.next_control_time = self.sim.dt_c
		self.mission.reset()
		self.is_sim_on = False
		self.next_visualization_time = time.time()

		return self.observation

	def seed(self, seed=0):
		# np.random.seed(seed)
		return
		
	def step(self, action):
		# scale action from [-1,1] to [action_lb, action_ub]
		scaled_action = self.action_lb + \
			(action + 1.) * 0.5 * (self.action_ub - self.action_lb)
		scaled_action = np.clip(scaled_action, self.action_lb, self.action_ub)

		self.mission.update_waypoint(self.sim.world.time())

		action_pid = np.squeeze(
			self.pid_policy.get_action(self.sim.states, self.sim.dt_c,
									   self.sim.sensor_fusion))

		# input_signal = policy_action + pid_action
		input_signal = np.clip(scaled_action + action_pid, self.total_action_lb, self.total_action_ub)

		# run simulation and down sample from f_sim to f_control
		while self.sim.world.time() < self.next_control_time:
			self.sim.step(input_signal)
		self.next_control_time += self.sim.dt_c

		self.observation = self.get_observation()
		reward = self.get_reward(action)
		done = self.get_terminal()

		# visulization
		if self.is_visual_on and time.time() >= self.next_visualization_time:
			self.gui.cv.wait()
			self.next_visualization_time = time.time() + self.dt_v

		info = {'time' : self.sim.world.time()}
		return self.observation, reward, done, info

	def get_observation(self):
		# define observations here3
		# 
		# observations are the following
		# rotation matrix
		# positions
		# linear velocities
		# angular velocities
		
		observation = np.zeros([18],dtype=np.float64)
		# get full states
		flapper1_states = self.sim.states
		# create rotation matrix
		roll_angle = flapper1_states['body_positions'][0]
		pitch_angle = flapper1_states['body_positions'][1]
		yaw_angle = flapper1_states['body_positions'][2]

		R = self.euler_2_R(roll_angle, pitch_angle, yaw_angle)
		observation[0:9] = R.reshape(-1)

		# other states
		observation[9] = flapper1_states['body_positions'][3] - self.mission.pos_target_x_	# special x
		observation[10] = flapper1_states['body_positions'][4] - self.mission.pos_target_y_	# special y
		observation[11] = flapper1_states['body_positions'][5] - self.mission.pos_target_z_	# special z
		observation[12] = flapper1_states['body_spatial_velocities'][0]	# spatial x_dot
		observation[13] = flapper1_states['body_spatial_velocities'][1]	# spatial y_dot
		observation[14] = flapper1_states['body_spatial_velocities'][2]	# spatial z_dot
		observation[15] = flapper1_states['body_velocities'][0]	# p
		observation[16] = flapper1_states['body_velocities'][1]	# q
		observation[17] = flapper1_states['body_velocities'][2]	# r

		observation = observation/self.observation_bound

		return observation

	def get_reward(self, action):
		reward = 0

		flapper1_states = self.sim.states

		position = flapper1_states['body_positions'][3:6]
		position_target = np.array([self.mission.pos_target_x_,self.mission.pos_target_y_,self.mission.pos_target_z_])
		position_error = position_target - position
		
		angular_position = flapper1_states['body_positions'][0:3]
		angular_position_target = np.array([[0.0], [0.0], [0.0]])
		angular_position_error = angular_position_target - angular_position

		linear_velocity = flapper1_states['body_spatial_velocities']
		angular_velocity =  flapper1_states['body_velocities'][0:3]

		d_action = action - self.action_old
		self.action_old = action

		control_cost = 2e-4*np.sum(np.square(action))
		d_control_cost = 2*np.sum(np.square(d_action))

		position_cost = 4e-3*np.linalg.norm(position_error)
		angular_position_cost = 8e-3*np.linalg.norm(angular_position_error)
		velocity_cots = 5e-4*np.linalg.norm(linear_velocity)
		angular_velocity_cost = 6e-4*np.linalg.norm(angular_velocity)

		stability_cost = position_cost + angular_position_cost + velocity_cots + angular_velocity_cost
		
		cost = stability_cost + control_cost + d_control_cost
		reward = 1 / (cost**2 + 1e-6)

		return reward

	def get_terminal(self):
		if self.sim.check_collision():
			return True
		if self.sim.world.time() > 20:
			return True

		flapper1_states = self.sim.states
		x = flapper1_states['body_positions'][3]
		y = flapper1_states['body_positions'][4]
		z = flapper1_states['body_positions'][5]

		distance = (x**2 + y**2 + z**2)**0.5
		if distance > 1:
			return True
		return False

	def euler_2_R(self, phi, theta, psi):
		R = np.zeros([3, 3], dtype=np.float64)

		C_phi = np.cos(phi)
		S_phi = np.sin(phi)
		C_theta = np.cos(theta)
		S_theta = np.sin(theta)
		C_psi = np.cos(psi)
		S_psi = np.sin(psi)

		R[0, 0] = C_psi * C_theta
		R[0, 1] = C_psi * S_theta * C_phi - S_psi * C_phi
		R[0, 2] = C_psi * S_theta * C_phi + S_psi * S_phi
		R[1, 0] = S_psi * C_theta
		R[1, 1] = S_psi * S_theta * S_phi + C_psi * C_phi
		R[1, 2] = S_psi * S_theta * C_phi - C_psi * S_phi
		R[2, 0] = -S_theta
		R[2, 1] = C_theta * S_phi
		R[2, 2] = C_theta * C_phi

		return R

