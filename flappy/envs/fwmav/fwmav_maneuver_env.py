##########################  FWMAV Simulation  #########################
# Version 0.3
# Fan Fei		Feb 2019
# Direct motor driven flapping wing MAV simulation
#######################################################################

import gym
from gym.spaces import Box
import numpy as np
import json
import pydart2 as pydart

from flappy.envs.fwmav.simulation import Simulation
from flappy.envs.fwmav.mission import Mission
from flappy.envs.fwmav.controllers.arc_xy_arc_z import ARCController

# GUI
from flappy.envs.fwmav.MyGLUTWindow import GUI
import time

class FWMAVManeuverEnv(gym.Env):
	def __init__(self):
		# initialize simulation and fwmav
		with open ('./flappy/envs/fwmav/config/sim_config.json') as file:
			sim_config = json.load(file)
		with open('./flappy/envs/fwmav/config/mav_config_list.json') as file:
			mav_config_list = json.load(file)
		self.sim = Simulation(mav_config_list, sim_config)
		# ground_skel = self.sim.world.add_skeleton('./flappy/urdf/ground.urdf')

		self.observation = np.zeros([18], dtype=np.float64)
		self.observation_bound = np.array([
			1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
			1.0, 1.0, 1.0, 1.0, 1.0
		])

		self.action_lb = np.array([-5.0, -3, -3.5, -0.15])
		self.action_ub = np.array([8.0, 3, 3.5, 0.15])
		self.total_action_lb = np.array([0.0, -3.0, -3.5, -0.15])
		self.total_action_ub = np.array([18.0, 3.0, 3.5, 0.15])
		self.action_old = np.array([0, 0, 0, 0])
		self.mission = Mission()

		self.done = False
		self.reward = 0
		self.random_init = True

		self.is_sim_on = False
		self.is_print_on = False
		self.is_visual_on = False
		self.dt_v = 1/sim_config['f_visual']

	def config(self, random_init=True, randomize_sim=True, phantom_sensor=False):
		self.random_init = random_init
		self.sim.randomize = randomize_sim
		self.sim.phantom_sensor = phantom_sensor
		self.sim.sensor_fusion.phantom = phantom_sensor
		if randomize_sim == False:
			self.sim.flapper1.nominal()

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
		return Box(np.array([-1, -1, -1, -1]), np.array([1, 1, 1, 1]))

	def reset(self):
		self.controller = ARCController(self.sim.dt_c)
		if self.random_init:
			rpy_limit = 0.2 	# 11.4 deg
			pqr_limit = 0.1		# 5.7 deg/s
			xyz_limit = 0.05	# 10 cm
			xyz_dot_limit = 0.1	# 10 cm/s
		else:
			rpy_limit = 0.0
			pqr_limit = 0.0
			xyz_limit = 0.0
			xyz_dot_limit = 0.0

		positions = np.zeros([10,1],dtype=np.float64)
		velocities = np.zeros([10,1],dtype=np.float64)

		init_attitude = np.random.uniform(-rpy_limit, rpy_limit, size=(3,1))
		init_angular_velocity = np.random.uniform(-pqr_limit, pqr_limit, size=(3,1))
		init_position = np.random.uniform(-xyz_limit, xyz_limit, size=(3,1))
		init_body_velocity = np.random.uniform(-xyz_dot_limit, xyz_dot_limit, size=(3,1))

		self.x_t = 0.21	# escape distance (m)

		init_attitude[2] = init_attitude[2]+np.pi
		init_position[0] = init_position[0]-self.x_t
		init_position[2] = init_position[2]+0.15

		positions[0:3] = init_attitude
		positions[3:6] = init_position
		# positions[5] = 0.0	# initial z
		if positions[3] < -self.x_t:
			positions[3] = -self.x_t	# initial x
		if positions[3] > -self.x_t+0.03:
			positions[3] = -self.x_t+0.03	# initial x
		velocities[0:6] = np.concatenate((init_angular_velocity,init_body_velocity), axis = 0)

		self.sim.reset()
		self.sim.set_states(positions, velocities)
		self.observation = self.get_observation()
		self.next_control_time = self.sim.dt_c
		self.mission.reset()
		self.is_sim_on = False
		self.next_visualization_time = time.time()

		self.reached = False

		return self.observation

	def seed(self, seed=0):
		# np.random.seed(seed)
		return

	def step(self, action):
		# scale action from [-1,1] to [action_lb, action_ub]
		# since baseline does not support asymmetric action space
		scaled_action = (action+1)*0.5*(self.action_ub-self.action_lb)+self.action_lb
		scaled_action = np.clip(scaled_action, self.action_lb, self.action_ub)

		# feedback controller action
		action_fb = np.squeeze(self.controller.get_action(self.observation*self.observation_bound))
		# total_action = policy_action + pid_action
		total_action = np.clip(scaled_action + action_fb, self.total_action_lb, self.total_action_ub)

		# check if reached target position/orientation
		flapper1_states = self.sim.states
		x = flapper1_states['body_positions'][3]
		x_dot = flapper1_states['body_spatial_velocities'][0]
		yaw_angle = flapper1_states['body_positions'][2]
		if self.reached == False and x > -0.05 and abs(x_dot) < 1 and abs(yaw_angle) < np.pi/4:
			self.reached = True
		if self.reached:
			total_action = np.clip((action_fb), self.total_action_lb, self.total_action_ub)

		# convert action to voltage signal
		max_voltage = total_action[0]
		voltage_diff = total_action[1]
		voltage_bias = total_action[2]
		split_cycle = total_action[3]
		input_voltage = np.zeros([2],dtype=np.float64)		
		input_voltage[0] = self.generate_control_signal(self.sim.flapper1.frequency, max_voltage, voltage_diff, voltage_bias, -split_cycle, self.sim.world.time(), 0)
		input_voltage[1] = self.generate_control_signal(self.sim.flapper1.frequency, max_voltage, -voltage_diff, voltage_bias, split_cycle, self.sim.world.time(), 0)
		input_voltage[0] = np.clip(input_voltage[0], -18, 18)
		input_voltage[1] = np.clip(input_voltage[1], -18, 18)

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
		roll_angle = self.sim.sensor_fusion.out_roll_
		pitch_angle = self.sim.sensor_fusion.out_pitch_
		yaw_angle = self.sim.sensor_fusion.out_yaw_

		R = self.euler_2_R(roll_angle, pitch_angle, yaw_angle)
		observation[0:9] = R.reshape(-1)

		observation[9] = self.sim.sensor_fusion.out_x_ - self.mission.pos_target_x_
		observation[10] = self.sim.sensor_fusion.out_y_ - self.mission.pos_target_y_
		observation[11] = self.sim.sensor_fusion.out_z_ - self.mission.pos_target_z_
		observation[12] = self.sim.sensor_fusion.out_x_dot_
		observation[13] = self.sim.sensor_fusion.out_y_dot_
		observation[14] = self.sim.sensor_fusion.out_z_dot_
		observation[15] = self.sim.sensor_fusion.IMU_gx_
		observation[16] = self.sim.sensor_fusion.IMU_gy_
		observation[17] = self.sim.sensor_fusion.IMU_gz_

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
		
		cost = (control_cost + d_control_cost + stability_cost)*3

		# body x axis projection along X axis
		R = self.sim.flapper1.flapper_skel.bodynode('torso').world_transform()
		i_hat = R[0:3,0]
		k_hat = R[0:3,2]
		x_projection = np.dot(i_hat,np.array([1,0,0]))
		z_projection = np.dot(k_hat,np.array([0,0,1]))

		# only negative reward when head (z) points downward
		if z_projection > 0:
			z_projection = 0

		#negative reward from x= -x_t to x=-0.05
		dis_yz_reward = flapper1_states['body_positions'][3]+0.05
		if dis_yz_reward > 0:
			dis_yz_reward = 0
		dis_yz_reward = 4* dis_yz_reward#scale to 1

		reward = 1/(cost**2+1e-6) + 4*x_projection + 1*z_projection + 6*dis_yz_reward

		# print("===========================time = %.4f ===========================" % self.world.time(), end="\n\r")
		# # # print("action", end="\n\r")
		# # # print(action, end="\n\r")
		# # # print("normalized_action", end="\n\r")
		# # # print(normalized_action, end="\n\r")
		# print("control_cost = %.8f" % control_cost, end="\n\r")
		# print("d_control_cost = %.8f" % d_control_cost, end="\n\r")
		# print("position_cost = %.8f" % position_cost, end="\n\r")
		# print("angular_position_cost = %.8f" % angular_position_cost, end="\n\r")
		# print("velocity_cots = %.8f" % velocity_cots, end="\n\r")
		# print("angular_velocity_cost = %.8f" % angular_velocity_cost, end="\n\r")
		# print("cost = %.8f" % cost, end="\n\r")
		# print("1/(cost**2+1e-6) = %.8f" % (1/(cost**2+1e-6)), end="\n\r")
		# print("4*_projection = %.8f" % (4*x_projection), end="\n\r")
		# print("1*z_projection = %.8f" % (1*z_projection), end="\n\r")
		# print("12*dis_yz_reward = %.8f" % (12*dis_yz_reward), end="\n\r")
		# print("reward = %.8f" % reward, end="\n\r")

		return reward

	def get_terminal(self):
		flapper1_states = self.sim.states
		x = flapper1_states['body_positions'][3]
		y = flapper1_states['body_positions'][4]
		z = flapper1_states['body_positions'][5]

		if self.sim.check_collision():
			print("collided")
			return True
		if self.sim.world.time() > 3:
			print("overtime")
			return True
		if x > 0.1:
			print("x overshoot")
			return True
		if x < -0.3:
			print(self.observation[9])
			print("x undershoot")
			return True
		if abs(y) > 0.2:
			print("y drifted")
			return True
		if abs(z) > 0.2:
			print("z drifted")
			return True
		R = self.sim.flapper1.flapper_skel.bodynode('torso').world_transform()
		i_hat = R[0:3,0]
		x_projection = np.dot(i_hat,np.array([1,0,0]))
		if self.sim.world.time() > 0.3 and x_projection<0:
			print("x aligned too slow")
			return True
		if self.sim.world.time() > 0.35 and x < -0.08:
			print("x translate too slow")
			return True
		return False

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
