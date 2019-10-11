##########################  FWMAV Simulation  #########################
# Version 0.1
# Fan Fei		Feb 2018
# FWMAV simulation with dual motor driven robotic flapper
# PID controller using split cycle mechanism
#######################################################################


import numpy as np
import random
from fwmav import FWMAV
from flappy.envs.fwmav import pydart2 as pydart
import threading
import click
import time
from flappy.envs.fwmav.pydart2.gui.glut.window import GLUTWindow
from flappy.envs.fwmav.pydart2.gui.trackball import Trackball
import OpenGL.GLUT as GLUT

#from controller_no_base import PIDController
#from arc_xy_arc_z_no_base import PIDController
#from controller import PIDController
#from pid_xy_arc_z_maneuver import PIDController
from controller_maneuver import PIDController
#from arc_xy_arc_z import PIDController

class Simulation:
	metadata = {
		'render.modes' : ['human', 'rgb_array'],
		'video.frames_per_second' : 20
	}
	def __init__(self, mav_config_list, sim_config):
		self.num_mav = len(mav_config_list)
		self.dt = 1/sim_config['f_sim']
		self.dt_c = 1/sim_config['f_control']
		self.fps = 24
		
		self.observation = np.zeros([18],dtype=np.float64)
		self.reward = 0
		self.done = False

		self.config = sim_config
		
		# initialize pydart world and create ground
		self.init_world()

		# add flapper in world, flapper skeleton wrapped in FWMAV and configured in FWMAV
		# 0 = no trim, 1 = with trim
		self.flapper1 = FWMAV(mav_config_list[6], self.world)		#2 is latest trim with base, 4 is without base, 6 is with small base
		self.states = self.flapper1.get_states()

		# initialize glut window
		self.init_glut((800, 600))
		self.seed() # set random seed
		self.reset() # call reset before simulation

		# initialize output data
		self.data={}
		self.data['t']=[]
		self.data['left_FN']=[]
		self.data['left_stroke']=[]
		self.data['left_rotate']=[]
		self.data['left_spanCoP']=[]
		self.data['left_chordCoP']=[]
		self.data['left_M_aero']=[]
		self.data['left_M_rd']=[]
		self.data['right_FN']=[]
		self.data['right_stroke']=[]
		self.data['right_rotate']=[]
		self.data['right_spanCoP']=[]
		self.data['right_chordCoP']=[]
		self.data['right_M_aero']=[]
		self.data['right_M_rd']=[]
		self.data['motor_torque']=[]
		self.data['magnetic_torque']=[]
		self.data['inertia_torque']=[]
		self.data['damping_torque']=[]
		self.data['friction_torque']=[]
		self.data['back_EMF']=[]
		self.data['current']=[]
		self.data['max_voltage']=[]
		self.data['voltage_diff']=[]
		self.data['voltage_bias']=[]
		self.data['split_cycle']=[]
		self.data['left_voltage']=[]
		self.data['right_voltage']=[]
		self.data['z']=[]
		self.data['z_dot']=[]
		self.data['z_ddot']=[]
		self.data['roll']=[]
		self.data['pitch']=[]
		self.data['yaw']=[]
		self.data['x']=[]
		self.data['y']=[]

		self.total_action_lb = np.array([0, -3, -3.5, -0.15])
		self.total_action_ub = np.array([18.0, 3, 3.5, 0.15])

		self.normalized_action_old = np.array([0, 0, 0, 0])


	def update_state(self):
		self.states = self.flapper1.get_states()

	def get_state(self):
		state = self.states
		return state

	def get_observation(self):
		# observations are the following
		# rotation matrix
		# positions
		# linear velocities
		# angular velocities
		
		observation = np.zeros([18],dtype=np.float64)
		# get full states
		flapper1_states = self.states
		# create rotation matrix
		roll_angle = flapper1_states['body_positions'][0]
		pitch_angle = flapper1_states['body_positions'][1]
		yaw_angle = flapper1_states['body_positions'][2]

		# wrap yaw error 180
		# print('yaw_error in observation')
		# print(yaw_error)


		R = self.euler_2_R(roll_angle, pitch_angle, yaw_angle)
		observation[0:9] = R.reshape(-1)
		# R_ = self.flapper1.flapper_skel.bodynode('torso').world_transform()
		# print(R)
		# print(R_)

		# other states
		observation[9] = flapper1_states['body_positions'][3]	# special x
		observation[10] = flapper1_states['body_positions'][4]	# special y
		observation[11] = flapper1_states['body_positions'][5]	# special z
		observation[12] = flapper1_states['body_spatial_velocities'][0]	# spatial x_dot
		observation[13] = flapper1_states['body_spatial_velocities'][1]	# spatial y_dot
		observation[14] = flapper1_states['body_spatial_velocities'][2]	# spatial z_dot
		observation[15] = flapper1_states['body_velocities'][0]	# p
		observation[16] = flapper1_states['body_velocities'][1]	# q
		observation[17] = flapper1_states['body_velocities'][2]	# r

		return observation

	def get_reward(self, action):
		reward = 0

		flapper1_states = self.states

		position = flapper1_states['body_positions'][3:6]
		position_target = np.array([[0.0], [0.0], [0.0]])
		position_error = position_target - position
		
		angular_position = flapper1_states['body_positions'][0:3]
		angular_position_target = np.array([[0.0], [0.0], [0.0]])
		angular_position_error = angular_position_target - angular_position

		# wrap 180
		# if (angular_position_error[2] > 3*np.pi or angular_position_error[2] < -3*np.pi):
		# 	angular_position_error[2] = np.fmod(angular_position_error[2],2*np.pi)
		# if (angular_position_error[2] > np.pi):
		# 	angular_position_error[2] = angular_position_error[2] - 2*np.pi
		# if (angular_position_error[2] < - np.pi):
		# 	angular_position_error[2] = angular_position_error[2] + 2*np.pi

		linear_velocity = flapper1_states['body_spatial_velocities']
		angular_velocity =  flapper1_states['body_velocities'][0:3]

		# roll = self.states['body_positions'][0]
		# pitch = self.states['body_positions'][1]
		# yaw = self.states['body_positions'][2]
		# x = self.states['body_positions'][3]
		# y = self.states['body_positions'][4]
		# z = self.states['body_positions'][5]
		# distance =((x+0.35)**2 + y**2 + z**2)**0.5

		normalized_action = action/(self.total_action_ub - self.total_action_lb)*2
		normalized_action[0] = (action[0] - self.total_action_lb[0])/(self.total_action_ub[0] - self.total_action_lb[0])
		# print('normalized_action:')
		# print(normalized_action)
		# the change of control as part of the cost
		d_normalized_action = normalized_action - self.normalized_action_old
		self.normalized_action_old = normalized_action

		control_cost = 2e-2*np.sum(np.square(normalized_action))
		d_control_cost = 2*np.sum(np.square(d_normalized_action))
		# if distance < 0.075 and abs(yaw-np.pi)<0.5:	# 0.5rad = 29deg
		# 	control_cost = 1e-2*np.sum(np.square(action))

		#control_cost = 1e-3*np.sum(np.square(action))
		position_cost = 4e-1*np.linalg.norm(position_error)
		angular_position_cost = 1e-1*np.linalg.norm(angular_position_error)
		velocity_cots = 5e-2*np.linalg.norm(linear_velocity)
		angular_velocity_cost = 5e-3*np.linalg.norm(angular_velocity)

		stability_cost = position_cost + angular_position_cost + velocity_cots + angular_velocity_cost
		
		cost = (control_cost + d_control_cost + stability_cost)*3 # 5 is tight, good for position control, 3 is looser
		

		# body x axis projection along X axis
		R = self.flapper1.flapper_skel.bodynode('torso').world_transform()
		i_hat = R[0:3,0]
		k_hat = R[0:3,2]
		x_projection = np.dot(i_hat,np.array([1,0,0]))
		z_projection = np.dot(k_hat,np.array([0,0,1]))

		# only negative reward when head (z) points downward
		if z_projection > 0:
			z_projection = 0

		# distance to -0.35yz plane
		dis_yz = flapper1_states['body_positions'][3]+0.35

		#negative reward from x= -0.35 to x=-0.1
		dis_yz_reward = -0.25 + dis_yz
		if dis_yz_reward > 0:
			dis_yz_reward = 0


		reward = 1/(cost**2+1e-6) + 4*x_projection + 1*z_projection + 12*dis_yz_reward

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

	def check_collision(self):
		collided = False
		if self.world.collision_result.num_contacted_bodies()>0:
			collided = True
		elif self.world.collision_result.num_contacts()>0:
			collided = True
		return collided

	def get_terminal(self):
		done = False
		if self.check_collision():
			done = True
		if self.world.time() > 3:
			done = True
		if self.observation[9] > 0.1:
			done = True
		if self.observation[9] < -0.375:
			done = True
		if abs(self.observation[10]) > 0.2:
			done = True
		if abs(self.observation[11]) > 0.1:
			done = True
		R = self.flapper1.flapper_skel.bodynode('torso').world_transform()
		i_hat = R[0:3,0]
		x_projection = np.dot(i_hat,np.array([1,0,0]))
		if self.world.time() > 0.35 and x_projection<0:
			done = True
		return done

	def render(self):
		fault = False
		self.glutwindow.drawGL()
		# if time.time() >= self.next_visulizaiton_time:
		# 	if self.visulization:
		# 		self.glutwindow.drawGL()
		# 	self.next_visulizaiton_time = self.next_visulizaiton_time + 1/self.fps
		return fault

	def seed(self):
		"""
		if there is randomness in simulation, set the random seed here
		implementation here
		"""
		return

	def reset(self):
		self.world.reset()
		self.flapper1.reset()

		self.sim_on = False
		self.paused = False
		self.visulization = True
		self.next_visulizaiton_time = 1/self.fps + time.time()
		self.next_control_time = 0.0
		self.elapse_time = 0
		self.reached = 0
		self.start_time = time.time()

		self.states = self.get_state()
		self.observation = self.get_observation()
		print("Simulation reset", end="\n\r")
		return self.states

	def set_states(self, positions, velocities):
		# two 10x1 column np array
		# 0: roll
		# 1: pitch
		# 2: yaw
		# 3: x
		# 4: y
		# 5: z
		# 6: left_stroke
		# 7: left_rotate
		# 8: right_stroke
		# 9: right_rotate
		self.flapper1.set_states(positions, velocities)
		self.update_state()

	def random_init(self):
		rpy_limit = 0.2 	# 11.4 deg
		pqr_limit = 0.1		# 5.7 deg/s
		xyz_limit = 0.1			# 10 cm
		xyz_dot_limit = 0.1		# 10 cm/s

		positions = np.zeros([10,1],dtype=np.float64)
		velocities = np.zeros([10,1],dtype=np.float64)

		init_attitude = np.random.uniform(-rpy_limit, rpy_limit, size=(3,1))
		init_angular_velocity = np.random.uniform(-pqr_limit, pqr_limit, size=(3,1))
		init_position = np.random.uniform(-xyz_limit, xyz_limit, size=(3,1))
		init_body_velocity = np.random.uniform(-xyz_dot_limit, xyz_dot_limit, size=(3,1))

		init_attitude[2] = init_attitude[2]+np.pi
		init_position[0] = init_position[0]-0.35

		positions[0:3] = init_attitude
		positions[3:6] = init_position
		# positions[5] = 0.0	# initial z
		if positions[3] < -0.35:
			positions[3] = -0.35	# initial x
		velocities[0:6] = np.concatenate((init_angular_velocity,init_body_velocity), axis = 0)

		self.reset()
		self.set_states(positions, velocities)
		observation = self.observation
		self.render()


	def step(self, action):

		max_voltage = action[0]
		voltage_diff = action[1]
		voltage_bias = action[2]
		split_cycle = action[3]

		input_voltage = np.zeros([2],dtype=np.float64)
		
		input_voltage[0] = self.generate_control_signal(34, max_voltage, voltage_diff, voltage_bias, -split_cycle, self.world.time(), 0)
		input_voltage[1] = self.generate_control_signal(34, max_voltage, -voltage_diff, voltage_bias, split_cycle, self.world.time(), 0)

		input_voltage[0] = np.clip(input_voltage[0], -18, 18)
		input_voltage[1] = np.clip(input_voltage[1], -18, 18)

		# generate input voltage or use external input
		#input_voltage[0] = 12*np.cos(self.flapper1.frequency*2*np.pi*self.world.time())
		#input_voltage[1] = 12*np.cos(self.flapper1.frequency*2*np.pi*self.world.time())

		# drive flapper aero and motor one step
		self.flapper1.step(self.world.time(), input_voltage)
		#print(self.flapper1.states['left_stroke_acceleration'], end="\n\r")
		
		# apply stroke force
		torques = np.zeros(self.flapper1.flapper_skel.num_dofs())
		torques[self.flapper1.flapper_skel.dof('left_stroke').id] = self.flapper1.left_motor.get_torque()
		torques[self.flapper1.flapper_skel.dof('right_stroke').id] = self.flapper1.right_motor.get_torque()
		self.flapper1.flapper_skel.set_forces(torques)

		# left_drive_torque = np.array([0, 0, self.flapper1.left_motor.get_torque()])
		# right_drive_torque = np.array([0, 0, -self.flapper1.right_motor.get_torque()])
		# self.flapper1.flapper_skel.bodynode('left_leading_edge').set_ext_torque(left_drive_torque, True)
		# self.flapper1.flapper_skel.bodynode('right_leading_edge').set_ext_torque(right_drive_torque, True)
		
		# get aero force
		left_FN = np.array([self.flapper1.left_wing.GetNormalForce(), 0, 0])	# in wing x direction
		right_FN = 	np.array([self.flapper1.right_wing.GetNormalForce(), 0, 0])
		left_CoP = np.array([0, self.flapper1.left_wing.GetSpanCoP(), (-1)*self.flapper1.left_wing.GetChordCoP()])
		right_CoP = np.array([0, (-1)*self.flapper1.right_wing.GetSpanCoP(), (-1)*self.flapper1.right_wing.GetChordCoP()])
		left_M_rd = np.array([0, self.flapper1.left_wing.GetM_rd(), 0])		# in wing y direction
		right_M_rd = np.array([0, self.flapper1.right_wing.GetM_rd(), 0])

		# apply aero force and moment on wing
		self.flapper1.flapper_skel.bodynode('left_wing').add_ext_force(left_FN, left_CoP, True, True)
		self.flapper1.flapper_skel.bodynode('right_wing').add_ext_force(right_FN, right_CoP, True, True)
		self.flapper1.flapper_skel.bodynode('left_wing').add_ext_torque(left_M_rd, True)
		self.flapper1.flapper_skel.bodynode('right_wing').add_ext_torque(right_M_rd, True)

		# step forward the dart simulation
		self.world.step()
		self.update_state()

		# save data
		# self.data['t'].append(self.world.time())
		# self.data['left_FN'].append(self.flapper1.left_wing.GetNormalForce())
		# self.data['left_stroke'].append(self.flapper1.flapper_skel.positions()[self.flapper1.flapper_skel.dof('left_stroke').id])
		# self.data['left_rotate'].append(self.flapper1.flapper_skel.positions()[self.flapper1.flapper_skel.dof('left_rotate').id])
		# self.data['left_spanCoP'].append(self.flapper1.left_wing.GetSpanCoP())
		# self.data['left_chordCoP'].append(self.flapper1.left_wing.GetChordCoP())
		# self.data['left_M_aero'].append(self.flapper1.left_wing.GetM_aero())
		# self.data['left_M_rd'].append(self.flapper1.left_wing.GetM_rd())
		# self.data['right_FN'].append(self.flapper1.right_wing.GetNormalForce())
		# self.data['right_stroke'].append(self.flapper1.flapper_skel.positions()[self.flapper1.flapper_skel.dof('right_stroke').id])
		# self.data['right_rotate'].append(self.flapper1.flapper_skel.positions()[self.flapper1.flapper_skel.dof('right_rotate').id])
		# self.data['right_spanCoP'].append(self.flapper1.right_wing.GetSpanCoP())
		# self.data['right_chordCoP'].append(self.flapper1.right_wing.GetChordCoP())
		# self.data['right_M_aero'].append(self.flapper1.right_wing.GetM_aero())
		# self.data['right_M_rd'].append(self.flapper1.right_wing.GetM_rd())
		# self.data['motor_torque'].append(self.flapper1.left_motor.motor_torque)
		# self.data['magnetic_torque'].append(self.flapper1.left_motor.magnetic_torque)
		# self.data['inertia_torque'].append(self.flapper1.left_motor.inertia_torque)
		# self.data['damping_torque'].append(self.flapper1.left_motor.damping_torque)
		# self.data['friction_torque'].append(self.flapper1.left_motor.friction_torque)
		# self.data['back_EMF'].append(self.flapper1.left_motor.back_EMF)
		# self.data['current'].append(self.flapper1.left_motor.current)
		# self.data['max_voltage'].append(max_voltage)
		# self.data['voltage_diff'].append(voltage_diff)
		# self.data['voltage_bias'].append(voltage_bias)
		# self.data['split_cycle'].append(split_cycle)
		# self.data['left_voltage'].append(input_voltage[0])
		# self.data['right_voltage'].append(input_voltage[1])
		# self.data['z'].append(self.pid_policy.altitude_)
		# self.data['z_dot'].append(self.pid_policy.velocity_z_)
		# self.data['z_ddot'].append(self.pid_policy.acceleration_z_)
		# self.data['roll'].append(self.pid_policy.roll_angle_)
		# self.data['pitch'].append(self.pid_policy.pitch_angle_)
		# self.data['yaw'].append(self.pid_policy.yaw_angle_)
		# self.data['x'].append(self.pid_policy.pos_current_x_)
		# self.data['y'].append(self.pid_policy.pos_current_y_)


		
		# if self.check_collision():
		# 	print("Colision detectecd!", end="\n\r")
		# 	# call reset here
		# 	# self.reset()

		self.observation = self.get_observation()
		self.reward = self.get_reward(action)
		self.done = self.get_terminal()
		
		return self.states

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
		R = np.zeros([3,3],dtype=np.float64)

		C_phi = np.cos(phi)
		S_phi = np.sin(phi)
		C_theta = np.cos(theta)
		S_theta = np.sin(theta)
		C_psi = np.cos(psi)
		S_psi = np.sin(psi)

		R[0,0] = C_psi*C_theta
		R[0,1] = C_psi*S_theta*S_phi - S_psi*C_phi
		R[0,2] = C_psi*S_theta*C_phi + S_psi*S_phi
		R[1,0] = S_psi*C_theta
		R[1,1] = S_psi*S_theta*S_phi + C_psi*C_phi
		R[1,2] = S_psi*S_theta*C_phi - C_psi*S_phi
		R[2,0] = -S_theta
		R[2,1] = C_theta*S_phi
		R[2,2] = C_theta*C_phi

		return R

	def record_data(self):
		data_file = open("pydata.txt", "w")
		for i in range(len(self.data['t'])):
			data_file.write(str(self.data['t'][i]) + "\t" +
							str(self.data['left_FN'][i]) + "\t" +
							str(self.data['left_stroke'][i]) + "\t" +
							str(self.data['left_rotate'][i]) + "\t" +
							str(self.data['left_spanCoP'][i]) + "\t" +
							str(self.data['left_chordCoP'][i]) + "\t" +
							str(self.data['left_M_aero'][i]) + "\t" +
							str(self.data['left_M_rd'][i]) + "\t" +
							str(self.data['right_FN'][i]) + "\t" +
							str(self.data['right_stroke'][i]) + "\t" +
							str(self.data['right_rotate'][i]) + "\t" +
							str(self.data['right_spanCoP'][i]) + "\t" +
							str(self.data['right_chordCoP'][i]) + "\t" +
							str(self.data['right_M_aero'][i]) + "\t" +
							str(self.data['right_M_rd'][i]) + "\t" +
							str(self.data['motor_torque'][i]) + "\t" +
							str(self.data['magnetic_torque'][i]) + "\t" +
							str(self.data['inertia_torque'][i]) + "\t" +
							str(self.data['damping_torque'][i]) + "\t" +
							str(self.data['friction_torque'][i]) + "\t" +
							str(self.data['back_EMF'][i]) + "\t" +
							str(self.data['current'][i]) + "\t" +
							str(self.data['max_voltage'][i]) + "\t" +
							str(self.data['voltage_diff'][i]) + "\t" +
							str(self.data['voltage_bias'][i]) + "\t" +
							str(self.data['split_cycle'][i]) + "\t" +
							str(self.data['left_voltage'][i]) + "\t" +
							str(self.data['right_voltage'][i]) + "\t" +
							str(self.data['z'][i]) + "\t" +
							str(self.data['z_dot'][i]) + "\t" +
							str(self.data['z_ddot'][i]) + "\t" +
							str(self.data['roll'][i]) + "\t" +
							str(self.data['pitch'][i]) + "\t" +
							str(self.data['yaw'][i]) + "\t" +
							str(self.data['x'][i]) + "\t" +
							str(self.data['y'][i]) + "\n"
							)
		data_file.close()
	
	def init_world(self):
		pydart.init(verbose=False)
		print('pydart initialization OK')
		self.world = MyWorld(self.dt)


	def init_glut(self,window_size = (800,600)):
		self.glutwindow = GLUTWindow(self.world, "FWMAV")
		GLUT.glutInit(())
		GLUT.glutInitDisplayMode(GLUT.GLUT_RGBA |
								 GLUT.GLUT_DOUBLE |
								 GLUT.GLUT_MULTISAMPLE |
								 GLUT.GLUT_ALPHA |
								 GLUT.GLUT_DEPTH)
		GLUT.glutInitWindowSize(*window_size)
		GLUT.glutInitWindowPosition(0, 0)
		self.glutwindow.window = GLUT.glutCreateWindow(self.glutwindow.title)
		self.glutwindow.initGL(*window_size)
		self.camera_theta = 75
		self.camera_phi = 135
		self.camera_horizontal = 0.0
		self.camera_vertical = -0.25
		self.camera_depth = -1.25
		self.camera_angle_increment = 5
		self.camera_position_increment = 0.05
		self.update_camera()
		#self.glutwindow.scene.add_camera(Trackball(theta = self.camera_theta, phi = self.camera_phi, trans=[self.camera_horizontal, self.camera_vertical, self.camera_depth]),"Camera Z up close")
		#self.glutwindow.scene.set_camera(2)
		self.glutwindow.scene.resize(*window_size)
		self.glutwindow.drawGL()

	def update_camera(self):
		self.glutwindow.scene.replace_camera(0,Trackball(theta = self.camera_theta, phi = self.camera_phi, trans=[self.camera_horizontal, self.camera_vertical, self.camera_depth]))
		self.glutwindow.scene.set_camera(0)
		self.glutwindow.drawGL()
		print("\r")

	def run(self,policy = None):
		keylistener = KeyListener(1,'Key Thread', 1)
		keylistener.start()
		self.pid_policy = PIDController(self.dt_c)
		if policy != 'pid' and policy !=None:
			policy = MyPolicy(policy)
		control_action = np.zeros(4)

		print("\n")
		print("space bar: simulation on/off")
		print("'r': reset simulation")
		print("'w, a, s, d' to rotate camera")
		print("'z, x' to zoom camera")
		print("'e, c' to adjust camera height")
		print("'v': toggle visulization")
		print("'q': quit")

		# main loop
		while True:
			key = keylistener.last_key_press
			if key == 'q':
				print("Simulation terminated, buh bye!", end="\n\r")
				break
			elif key == 'r':
				self.reset()
			elif key == 'o':
				self.record_data()
			elif key == 'm':
				self.random_init()
			elif key == ' ':
				self.sim_on = not self.sim_on
			elif key == 'v':
				self.visulization = not self.visulization
				print("Visulization = %r" % self.visulization, end="\n\r")
			elif key == 'w':
				self.camera_theta -= self.camera_angle_increment
				self.update_camera()
			elif key == 's':
				self.camera_theta += self.camera_angle_increment
				self.update_camera()
			elif key == 'a':
				self.camera_phi -= self.camera_angle_increment
				self.update_camera()
			elif key == 'd':
				self.camera_phi += self.camera_angle_increment
				self.update_camera()
			elif key == 'z':
				self.camera_depth += self.camera_position_increment
				self.update_camera()
			elif key == 'x':
				self.camera_depth -= self.camera_position_increment
				self.update_camera()
			elif key == 'e':
				self.camera_vertical += self.camera_position_increment
				self.update_camera()
			elif key == 'c':
				self.camera_vertical -= self.camera_position_increment
				self.update_camera()
			keylistener.last_key_press = None

			if self.world.time()>=20.0:
				self.sim_on = False

			if self.sim_on:
				if self.paused:
					print("Simulation running", end="\n\r")
					self.paused = False
					self.start_time = time.time()
					self.next_visulizaiton_time = 1/self.fps + time.time()

				if self.world.time()>=self.next_control_time:
					self.next_control_time += self.dt_c
					# print('update control', end="\n\r")
					if policy == None:
						control_action = [10,0,0,0]
					elif policy == 'pid':
						control_action = np.squeeze(self.pid_policy.get_action(self.states,self.dt_c, 1))
					else:
						action = policy.get_action(self.observation)

						# get reached

						action_pid = np.squeeze(self.pid_policy.get_action(self.states, self.dt_c, self.reached))

						flapper1_states = self.states
						x = flapper1_states['body_positions'][3]
						x_dot = flapper1_states['body_spatial_velocities'][0]	# spatial x_dot
						yaw_angle = flapper1_states['body_positions'][2]
						# print('yaw_error in action')
						# print(yaw_error)

						# combine controller action and policy action
						control_action = np.clip((action + action_pid), self.total_action_lb, self.total_action_ub)

						if self.reached == 0 and x > -0.1 and abs(x_dot) < 1 and abs(yaw_angle) < np.pi/4:
							self.reached = 1

						if self.reached == 1:
							control_action = np.squeeze(self.pid_policy.get_action(self.states,self.dt_c, 1))
						# print("reached", end="\n\r")
						# print(self.reached, end="\n\r")

				# for i in range(20):
				# 	self.step(control_action)
				self.step(control_action)
				
				# if policy == None:
				# 	for i in range(20):
				# 		self.step([10,0,0,0])

				# elif policy == 'pid':
				# 	action_pid = np.squeeze(self.pid_policy.get_action(self.states,0.002))
				# 	for i in range(20):
				# 		self.step(action_pid)

				# else:
				# 	action = policy.get_action(self.observation)
				# 	for i in range(20):
				# 		self.step(action)
					
				
				if time.time() >= self.next_visulizaiton_time:
					if self.world.time()>=20.0:
						self.sim_on = not self.sim_on
					if self.visulization:
						self.glutwindow.drawGL()
					self.next_visulizaiton_time = self.next_visulizaiton_time + 1/self.fps

			else:
				if not self.paused:
					self.glutwindow.drawGL()
					self.elapse_time += (time.time() - self.start_time)
					print("Simulation paused, running time = %.4f" % self.elapse_time, end="\n\r")
				self.paused = True
				time.sleep(0.01)
		return

class MyPolicy():
	def __init__(self, policy_name):
		import pickle
		import os

		log_dir = os.path.join(os.getcwd(),'data')

		with open(os.path.join(log_dir,policy_name), 'rb') as input:
			policy = pickle.load(input)

		self.h0w = policy._cached_params[()][0].get_value()
		self.h0b = policy._cached_params[()][1].get_value()
		self.h1w = policy._cached_params[()][2].get_value()
		self.h1b = policy._cached_params[()][3].get_value()
		self.ow = policy._cached_params[()][4].get_value()
		self.ob = policy._cached_params[()][5].get_value()

		# action lb ub are set in fwmav_sim_env.py
		self.action_lb = np.array([-5.0, -3, -3.5, -0.15])
		self.action_ub = np.array([8.0, 3, 3.5, 0.15])

		# ologstd = policy._cached_params[()][6].get_value()
	def get_action(self,observation):
		h0_out = np.tanh(np.matmul(observation.T, self.h0w) + self.h0b)
		h1_out = np.tanh(np.matmul(h0_out, self.h1w) + self.h1b)
		action = np.tanh(np.matmul(h1_out, self.ow) + self.ob)

		# scale action
		scaled_action = self.action_lb + (action + 1.) * 0.5 * (self.action_ub - self.action_lb)
		scaled_action = np.clip(scaled_action, self.action_lb, self.action_ub)

		return scaled_action

# my world class
class MyWorld(pydart.World):
	def __init__(self, dt):
		pydart.World.__init__(self, dt)
		self.set_gravity([0.0, 0.0, -9.81])
		#self.set_collision_detector(0)
		print('pydart create_world OK')

		# create ground
		# filename = "./urdf/ground.urdf"
		# self.ground = self.add_skeleton(filename)

	def step(self, ):
		super(MyWorld, self).step()

		# other stuff here
		#print("t = %.4f" % self.time(), end="\n\r")
		#print(len(self.collision_result.contacts), end="\n\r")

	def draw_with_ri(self, ri):
		ri.set_color(0, 0, 0)
		ri.draw_text([20, 40], "time = %.4fs" % self.t)
		ri.set_color(0, 0, 0)
		ri.draw_text([20, 80], "frame = %d" % self.frame)
		R = self.skeletons[0].bodynode('torso').world_transform()
		ri.draw_text([20, 100], "roll = %.4f" % (np.arctan2(R[2,1],R[2,2])/np.pi*180))
		ri.draw_text([20, 120], "pitch = %.4f" % (np.arcsin(-R[2,0])/np.pi*180))
		ri.draw_text([20, 140], "yaw = %.4f" % (np.arctan2(R[1,0],R[0,0])/np.pi*180))
		ri.draw_text([20, 160], "p = %.4f" % (self.skeletons[0].bodynode('torso').com_spatial_velocity()[0]/np.pi*180))
		ri.draw_text([20, 180], "q = %.4f" % (self.skeletons[0].bodynode('torso').com_spatial_velocity()[1]/np.pi*180))
		ri.draw_text([20, 200], "r = %.4f" % (self.skeletons[0].bodynode('torso').com_spatial_velocity()[2]/np.pi*180))
		ri.draw_text([20, 220], "x = %.4f" % self.skeletons[0].positions()[self.skeletons[0].dof('torso_to_world_pos_x').id])
		ri.draw_text([20, 240], "y = %.4f" % self.skeletons[0].positions()[self.skeletons[0].dof('torso_to_world_pos_y').id])
		ri.draw_text([20, 260], "z = %.4f" % self.skeletons[0].positions()[self.skeletons[0].dof('torso_to_world_pos_z').id])
		ri.draw_text([20, 280], "x_dot = %.4f" % self.skeletons[0].bodynode('torso').com_linear_velocity()[0])
		ri.draw_text([20, 300], "y_dot = %.4f" % self.skeletons[0].bodynode('torso').com_linear_velocity()[1])
		ri.draw_text([20, 320], "z_dot = %.4f" % self.skeletons[0].bodynode('torso').com_linear_velocity()[2])
		# visulize force not working
		# pl0 = self.skeletons[1].bodynode('left_wing').C
		# pl1 = pl0 + 0.01 * np.array([1,0,0])
		# ri.set_color(1.0, 0.0, 0.0)
		# ri.render_arrow(pl0, pl1, r_base=0.05, head_width=0.1, head_len=0.1)


# keyboard input thread
class KeyListener(threading.Thread):
	def __init__(self,threadID,name,counter):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.counter = counter
		self.last_key_press = None
		self.new_key_pressed = False

	def run(self):
		print("Key listening thread started")
		# print('Starting ' + self.name)
		while True:
			self.last_key_press = click.getchar()
			if self.last_key_press == 'q':
				print("Key listening thread terminated")
				break
