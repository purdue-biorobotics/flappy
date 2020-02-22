from rllab.envs.base import Env, Step
from rllab.spaces import Box
import numpy as np
import json
#from simulation_maneuver import Simulation
from Flappy.envs.simulation_maneuver import Simulation
#from pid_xy_arc_z_maneuver import PIDController
from Flappy.envs.controllers.pid_controller import PIDController

class FWMAVSimEnv(Env):
	def __init__(self):
		with open ('./Flappy/envs/config/sim_config.json') as file:
			sim_config = json.load(file)
		with open('./Flappy/envs/config/mav_config_list.json') as file:
			mav_config_list = json.load(file)
		self.sim = Simulation(mav_config_list, sim_config)
		self.action_lb = np.array([0.0, -3, -3.5, -0.15])
		self.action_ub = np.array([18.0, 3, 3.5, 0.15])

	
	@property
	def observation_space(self):
		return Box(np.array([-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -100.0, -100.0, -100.0]), np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 100.0, 100.0, 100.0]))

	@property
	def action_space(self):
		return Box(np.array([-5.0, -3, -3.5, -0.15]), np.array([8.0, 3, 3.5, 0.15]))	# double the control authority range here so the policy can over ride the controller action

	# def reset(self):
	# 	self.pid_policy = PIDController(self.sim.dt_c)
	# 	rpy_limit = 0.2 	# 11.4 deg
	# 	pqr_limit = 0.1		# 5.7 deg/s
	# 	xyz_limit = 0.1			# 10 cm
	# 	xyz_dot_limit = 0.1		# 10 cm/s

	# 	positions = np.zeros([10,1],dtype=np.float64)
	# 	velocities = np.zeros([10,1],dtype=np.float64)

	# 	init_attitude = np.random.uniform(-rpy_limit, rpy_limit, size=(3,1))
	# 	init_angular_velocity = np.random.uniform(-pqr_limit, pqr_limit, size=(3,1))
	# 	init_position = np.random.uniform(-xyz_limit, xyz_limit, size=(3,1))
	# 	init_body_velocity = np.random.uniform(-xyz_dot_limit, xyz_dot_limit, size=(3,1))

	# 	init_attitude[2] = init_attitude[2]+np.pi
	# 	init_position[0] = init_position[0]-0.35

	# 	positions[0:3] = init_attitude
	# 	positions[3:6] = init_position
	# 	# positions[5] = 0.0	# initial z
	# 	if positions[3] < -0.35:
	# 		positions[3] = -0.35	# initial x
	# 	velocities[0:6] = np.concatenate((init_angular_velocity,init_body_velocity), axis = 0)

	# 	self.sim.reset()
	# 	self.sim.set_states(positions, velocities)
	# 	observation = self.sim.observation

	# 	self.reached = 0
	# 	return observation


	# 	# pid controller input
	# 	action_pid = np.squeeze(self.pid_policy.get_action(self.sim.states, self.sim.dt_c, self.reached))
		
	# 	flapper1_states = self.sim.states

	# 	x = flapper1_states['body_positions'][3]

	# 	x_dot = flapper1_states['body_spatial_velocities'][0]	# spatial x_dot

	# 	yaw_angle = flapper1_states['body_positions'][2]

	# 	# combine controller action and policy action
	# 	total_action = np.clip((action + action_pid), self.action_lb, self.action_ub)

	# 	if self.reached == 0 and x > -0.1 and abs(x_dot) < 1 and abs(yaw_angle) < np.pi/4:
	# 		self.reached = 1

	# 	if self.reached == 1:
	# 		total_action = np.clip((action_pid), self.action_lb, self.action_ub)


	# 	# down sample to 500Hz
	# 	for i in range(20):
	# 		self.sim.step(total_action)
	# 	#self.sim.step(action)
	# 	next_observation = self.sim.observation
	# 	reward = self.sim.reward
	# 	done = self.sim.done
	# 	if (np.mod(self.sim.world.frame,60)==0):
	# 		self.sim.render()
	# 	return Step(observation=next_observation, reward=reward, done=done)
		
	'''replaced with code from v2/fwmav_sim_env_maneuver.py'''
	def reset(self):
		self.pid_policy = PIDController(self.sim.dt_c)
		rpy_limit = 0.2 	# 11.4 deg
		pqr_limit = 0.1		# 5.7 deg/s
		xyz_limit = 0.05			# 10 cm
		xyz_dot_limit = 0.1		# 10 cm/s

		positions = np.zeros([10,1],dtype=np.float64)
		velocities = np.zeros([10,1],dtype=np.float64)

		init_attitude = np.random.uniform(-rpy_limit, rpy_limit, size=(3,1))
		init_angular_velocity = np.random.uniform(-pqr_limit, pqr_limit, size=(3,1))
		init_position = np.random.uniform(-xyz_limit, xyz_limit, size=(3,1))
		init_body_velocity = np.random.uniform(-xyz_dot_limit, xyz_dot_limit, size=(3,1))

		init_attitude[2] = init_attitude[2]+0
		init_position[0] = init_position[0]-0
		init_position[2] = init_position[2]+0

		positions[0:3] = init_attitude # zhi shi dian
		positions[3:6] = init_position
		positions[5] = 0.5	# initial z

		# if positions[3] < -0.28:
		# 	positions[3] = -0.28	# initial x

		velocities[0:6] = np.concatenate((init_angular_velocity,init_body_velocity), axis = 0)

		self.sim.reset()
		self.sim.set_states(positions, velocities)
		observation = self.sim.observation
		self.sim.upside_down = 0 #zt
		self.sim.reached = 0
		return observation

	def step(self, action):	
		# pid controller input
		action_pid = np.squeeze(self.pid_policy.get_action(self.sim.states, self.sim.dt_c, self.sim.reached))
		
		flapper1_states = self.sim.states

		# x = flapper1_states['body_positions'][3]

		# x_dot = flapper1_states['body_spatial_velocities'][0]	# spatial x_dot

		# yaw_angle = flapper1_states['body_positions'][2]

		# combine controller action and policy action

		action[1] = 0 #zt#switched from 2 to 1
		action[3] = 0

		total_action = np.clip((action + action_pid), self.action_lb, self.action_ub)

		# zt
		R = self.sim.flapper1.flapper_skel.bodynode('torso').world_transform()

		i_hat = R[0:3,0] #added x project for back flip
		k_hat = R[0:3,2]
		x_projection = np.dot(i_hat,np.array([1,0,0]))
		z_projection = np.dot(k_hat,np.array([0,0,1]))

		roll_vel = flapper1_states['body_velocities'][0]	# p
		pitch_vel = flapper1_states['body_velocities'][1]	# q
        #if self.sim.reached == 0 and self.sim.upside_down == 1 and z_projection > 0.8 and abs(roll_vel) < 160: #roll_vel<+-200;currently switched to pitch to train back flip
		#self.sim.reach
		if self.sim.reached == 0 and x_projection < 0.1 and self.sim.upside_down == 1 and z_projection > 0.8 and abs(pitch_vel) < 160: #roll_vel<+-200;currently switched to pitch to train back flip
			self.sim.reached = 1


		# print("reached = %d" % self.sim.reached, end="\n\r")
		# print("upside_down = %d" % self.sim.upside_down, end="\n\r")
		# print("z_projection = %d" % z_projection, end="\n\r")
		

		if self.sim.reached == 1:
			total_action = np.clip((action_pid), self.action_lb, self.action_ub)


		# down sample to 500Hz
		for i in range(20):
			self.sim.step(total_action)
		#self.sim.step(action)
		next_observation = self.sim.observation
		reward = self.sim.reward
		done = self.sim.done
		if (np.mod(self.sim.world.frame,60)==0):
			self.sim.render()
		return Step(observation=next_observation, reward=reward, done=done)

	def render(self):
		self.sim.render()
