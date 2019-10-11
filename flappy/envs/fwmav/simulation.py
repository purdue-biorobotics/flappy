##########################  FWMAV Simulation  #########################
# Version 0.3
# Fan Fei		Feb 2019
# Direct motor driven flapping wing MAV simulation
#######################################################################

import numpy as np
import random
from flappy.envs.fwmav import pydart2 as pydart
import threading
import click
import time

# robot
from flappy.envs.fwmav.fwmav import FWMAV

# sensors
from flappy.envs.fwmav.sensor import Sensor_IMU_Vicon
from flappy.envs.fwmav.sensor_fusion_pos import SensorFusion



class Simulation:
	def __init__(self, mav_config_list, sim_config):
		self.dt = 1/sim_config['f_sim']
		self.dt_d = 1/sim_config['f_driver']
		self.dt_c = 1/sim_config['f_control']
		self.dt_s = 1/sim_config['f_sensor']
		self.dt_imu = 1/sim_config['f_imu']
		self.dt_vicon = 1/sim_config['f_vicon']
		self.phantom_sensor = False
		self.randomize = True
		self.fps = 24

		self.config = sim_config
		
		# initialize pydart world and create ground
		self.init_world()

		# add flapper in world, flapper skeleton wrapped in FWMAV and configured in FWMAV
		# 0 = no trim, 1 = with trim
		self.flapper1 = FWMAV(mav_config_list[0], self.world, self.dt_d)		#2 is latest trim with base, 4 is without base
		self.states = self.flapper1.get_states()

		# initialize glut window
		# self.init_glut((800, 600))
		
		self.reset() # call reset before simulation

		# # initialize output data
		# self.data={}
		# self.data['t']=[]
		# self.data['left_FN']=[]
		# self.data['left_stroke']=[]
		# self.data['left_rotate']=[]
		# self.data['left_spanCoP']=[]
		# self.data['left_chordCoP']=[]
		# self.data['left_M_aero']=[]
		# self.data['left_M_rd']=[]
		# self.data['right_FN']=[]
		# self.data['right_stroke']=[]
		# self.data['right_rotate']=[]
		# self.data['right_spanCoP']=[]
		# self.data['right_chordCoP']=[]
		# self.data['right_M_aero']=[]
		# self.data['right_M_rd']=[]
		# self.data['motor_torque']=[]
		# self.data['magnetic_torque']=[]
		# self.data['inertia_torque']=[]
		# self.data['damping_torque']=[]
		# self.data['friction_torque']=[]
		# self.data['back_EMF']=[]
		# self.data['current']=[]
		# self.data['max_voltage']=[]
		# self.data['voltage_diff']=[]
		# self.data['voltage_bias']=[]
		# self.data['split_cycle']=[]
		# self.data['left_voltage']=[]
		# self.data['right_voltage']=[]
		# self.data['roll']=[]
		# self.data['pitch']=[]
		# self.data['yaw']=[]
		# self.data['p']=[]
		# self.data['q']=[]
		# self.data['r']=[]
		# self.data['x']=[]
		# self.data['y']=[]
		# self.data['z']=[]
		# self.data['x_dot']=[]
		# self.data['y_dot']=[]
		# self.data['z_dot']=[]
		# self.data['x_ddot']=[]
		# self.data['y_ddot']=[]
		# self.data['z_ddot']=[]
		# self.data['vicon_roll']=[]
		# self.data['vicon_pitch']=[]
		# self.data['vicon_yaw']=[]
		# self.data['vicon_x']=[]
		# self.data['vicon_y']=[]
		# self.data['vicon_z']=[]
		# self.data['vicon_x_dot']=[]
		# self.data['vicon_y_dot']=[]
		# self.data['vicon_z_dot']=[]
		# self.data['fuse_roll']=[]
		# self.data['fuse_pitch']=[]
		# self.data['fuse_yaw']=[]
		# self.data['fuse_x']=[]
		# self.data['fuse_y']=[]
		# self.data['fuse_z']=[]
		# self.data['fuse_x_dot']=[]
		# self.data['fuse_y_dot']=[]
		# self.data['fuse_z_dot']=[]
		# self.data['gx']=[]
		# self.data['gy']=[]
		# self.data['gz']=[]
		# self.data['ax']=[]
		# self.data['ay']=[]
		# self.data['az']=[]
		# self.data['fuse_x_ddot']=[]
		# self.data['fuse_y_ddot']=[]
		# self.data['fuse_z_ddot']=[]

	def update_state(self):
		self.states = self.flapper1.get_states()

	def get_state(self):
		return self.states

	def check_collision(self):
		if self.world.collision_result.num_contacted_bodies()>0:
			return True
		if self.world.collision_result.num_contacts()>0:
			return True
		return False

	def radonmize(self):
		if self.randomize:
			self.flapper1.randomize()
		return

	def reset(self):
		self.world.reset()
		self.flapper1.reset()

		
		# self.visulization = True
		# self.next_visulizaiton_time = 1/self.fps + time.time()
		self.next_sensor_fusion_time = 0.0
		self.states = self.get_state()
		self.sensor = Sensor_IMU_Vicon(self.dt_imu, self.dt_vicon, self.states)
		self.sensor_fusion = SensorFusion(self.dt_s, self.states, self.phantom_sensor)
		self.radonmize()

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


	def step(self, input_voltage):
		# print('===================== time = %.4f =====================' % self.world.time(), end="\n\r")
		self.sensor.update_raw_data(self.world.time(), self.states)
		if self.world.time()>=self.next_sensor_fusion_time:
			self.next_sensor_fusion_time += self.dt_s
			self.sensor_fusion.run(self.sensor, self.states)
			#print('fusion_roll = %.4f, roll = %.4f' % (self.sensor_fusion.out_roll_, self.states['body_positions'][0,0]), end="\n\r")
			#print('fusion_pitch = %.4f, pitch = %.4f' % (self.sensor_fusion.out_pitch_, self.states['body_positions'][1,0]), end="\n\r")
			#print('fusion_yaw = %.4f, yaw = %.4f' % (self.sensor_fusion.out_yaw_, self.states['body_positions'][2,0]), end="\n\r")

		# 12V sine wave for debugging
		#input_voltage[0] = 12*np.cos(self.flapper1.frequency*2*np.pi*self.world.time())
		#input_voltage[1] = 12*np.cos(self.flapper1.frequency*2*np.pi*self.world.time())

		# drive flapper aero and motor one step
		self.flapper1.step(self.world.time(), input_voltage)

		# step forward the dart simulation
		self.world.step()
		self.update_state()

		# # save data
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
		
		# self.data['roll'].append(self.states['body_positions'][0,0])
		# self.data['pitch'].append(self.states['body_positions'][1,0])
		# self.data['yaw'].append(self.states['body_positions'][2,0])
		# self.data['p'].append(self.states['body_velocities'][0,0])
		# self.data['q'].append(self.states['body_velocities'][1,0])
		# self.data['r'].append(self.states['body_velocities'][2,0])
		# self.data['x'].append(self.states['body_positions'][3,0])
		# self.data['y'].append(self.states['body_positions'][4,0])
		# self.data['z'].append(self.states['body_positions'][5,0])
		# self.data['x_dot'].append(self.states['body_spatial_velocities'][0,0])
		# self.data['y_dot'].append(self.states['body_spatial_velocities'][1,0])
		# self.data['z_dot'].append(self.states['body_spatial_velocities'][2,0])
		# self.data['x_ddot'].append(self.states['body_spatial_accelerations'][0,0])
		# self.data['y_ddot'].append(self.states['body_spatial_accelerations'][1,0])
		# self.data['z_ddot'].append(self.states['body_spatial_accelerations'][2,0])
		# self.data['vicon_roll'].append(self.sensor.vicon_roll_)
		# self.data['vicon_pitch'].append(self.sensor.vicon_pitch_)
		# self.data['vicon_yaw'].append(self.sensor.vicon_yaw_)
		# self.data['vicon_x'].append(self.sensor.vicon_x_)
		# self.data['vicon_y'].append(self.sensor.vicon_y_)
		# self.data['vicon_z'].append(self.sensor.vicon_z_)
		# self.data['vicon_x_dot'].append(self.sensor_fusion.vicon_x_dot_)
		# self.data['vicon_y_dot'].append(self.sensor_fusion.vicon_y_dot_)
		# self.data['vicon_z_dot'].append(self.sensor_fusion.vicon_z_dot_)
		# self.data['fuse_roll'].append(self.sensor_fusion.out_roll_)
		# self.data['fuse_pitch'].append(self.sensor_fusion.out_pitch_)
		# self.data['fuse_yaw'].append(self.sensor_fusion.out_yaw_)
		# self.data['fuse_x'].append(self.sensor_fusion.out_x_)
		# self.data['fuse_y'].append(self.sensor_fusion.out_y_)
		# self.data['fuse_z'].append(self.sensor_fusion.out_z_)
		# self.data['fuse_x_dot'].append(self.sensor_fusion.out_x_dot_)
		# self.data['fuse_y_dot'].append(self.sensor_fusion.out_y_dot_)
		# self.data['fuse_z_dot'].append(self.sensor_fusion.out_z_dot_)
		# self.data['gx'].append(self.sensor.IMU_gx_)
		# self.data['gy'].append(self.sensor.IMU_gy_)
		# self.data['gz'].append(self.sensor.IMU_gz_)
		# self.data['ax'].append(self.sensor.IMU_ax_)
		# self.data['ay'].append(self.sensor.IMU_ay_)
		# self.data['az'].append(self.sensor.IMU_az_)
		# self.data['fuse_x_ddot'].append(self.sensor_fusion.p_ddot[0][0])
		# self.data['fuse_y_ddot'].append(self.sensor_fusion.p_ddot[1][0])
		# self.data['fuse_z_ddot'].append(self.sensor_fusion.p_ddot[2][0])

		
		# if self.check_collision():
		# 	print("Colision detectecd!", end="\n\r")
		# 	# call reset here
		# 	# self.reset()
		
		return self.states

	


	def record_data(self):
		data_file = open("pydata.txt", "w")
		for i in range(len(self.data['t'])):
			# data_file.write(str(self.data['t'][i]) + "\t" +
			# 				str(self.data['left_FN'][i]) + "\t" +
			# 				str(self.data['left_stroke'][i]) + "\t" +
			# 				str(self.data['left_rotate'][i]) + "\t" +
			# 				str(self.data['left_spanCoP'][i]) + "\t" +
			# 				str(self.data['left_chordCoP'][i]) + "\t" +
			# 				str(self.data['left_M_aero'][i]) + "\t" +
			# 				str(self.data['left_M_rd'][i]) + "\t" +
			# 				str(self.data['right_FN'][i]) + "\t" +
			# 				str(self.data['right_stroke'][i]) + "\t" +
			# 				str(self.data['right_rotate'][i]) + "\t" +
			# 				str(self.data['right_spanCoP'][i]) + "\t" +
			# 				str(self.data['right_chordCoP'][i]) + "\t" +
			# 				str(self.data['right_M_aero'][i]) + "\t" +
			# 				str(self.data['right_M_rd'][i]) + "\t" +
			# 				str(self.data['motor_torque'][i]) + "\t" +
			# 				str(self.data['magnetic_torque'][i]) + "\t" +
			# 				str(self.data['inertia_torque'][i]) + "\t" +
			# 				str(self.data['damping_torque'][i]) + "\t" +
			# 				str(self.data['friction_torque'][i]) + "\t" +
			# 				str(self.data['back_EMF'][i]) + "\t" +
			# 				str(self.data['current'][i]) + "\t" +
			# 				str(self.data['max_voltage'][i]) + "\t" +
			# 				str(self.data['voltage_diff'][i]) + "\t" +
			# 				str(self.data['voltage_bias'][i]) + "\t" +
			# 				str(self.data['split_cycle'][i]) + "\t" +
			# 				str(self.data['left_voltage'][i]) + "\t" +
			# 				str(self.data['right_voltage'][i]) + "\t" +
			# 				str(self.data['z'][i]) + "\t" +
			# 				str(self.data['z_dot'][i]) + "\t" +
			# 				str(self.data['z_ddot'][i]) + "\t" +
			# 				str(self.data['roll'][i]) + "\t" +
			# 				str(self.data['pitch'][i]) + "\t" +
			# 				str(self.data['yaw'][i]) + "\t" +
			# 				str(self.data['x'][i]) + "\t" +
			# 				str(self.data['y'][i]) + "\t" +
			# 				str(self.data['vicon_roll'][i]) + "\t" +
			# 				str(self.data['vicon_pitch'][i]) + "\t" +
			# 				str(self.data['vicon_yaw'][i]) + "\t" +
			# 				str(self.data['fuse_roll'][i]) + "\t" +
			# 				str(self.data['fuse_pitch'][i]) + "\t" +
			# 				str(self.data['fuse_yaw'][i]) + "\t" +
			# 				str(self.data['gx'][i]) + "\t" +
			# 				str(self.data['gy'][i]) + "\t" +
			# 				str(self.data['gz'][i]) + "\n"
			# 				)
			data_file.write(str(self.data['t'][i]) + "\t" +
							str(self.data['roll'][i]) + "\t" +
							str(self.data['pitch'][i]) + "\t" +
							str(self.data['yaw'][i]) + "\t" +
							str(self.data['x'][i]) + "\t" +
							str(self.data['y'][i]) + "\t" +
							str(self.data['z'][i]) + "\t" +
							str(self.data['p'][i]) + "\t" +
							str(self.data['q'][i]) + "\t" +
							str(self.data['r'][i]) + "\t" +
							str(self.data['x_dot'][i]) + "\t" +
							str(self.data['y_dot'][i]) + "\t" +
							str(self.data['z_dot'][i]) + "\t" +
							str(self.data['x_ddot'][i]) + "\t" +
							str(self.data['y_ddot'][i]) + "\t" +
							str(self.data['z_ddot'][i]) + "\t" +
							str(self.data['vicon_roll'][i]) + "\t" +
							str(self.data['vicon_pitch'][i]) + "\t" +
							str(self.data['vicon_yaw'][i]) + "\t" +
							str(self.data['vicon_x'][i]) + "\t" +
							str(self.data['vicon_y'][i]) + "\t" +
							str(self.data['vicon_z'][i]) + "\n"
							)
		data_file.close()
	
	def init_world(self):
		pydart.init(verbose=False)
		print('pydart initialization OK')
		self.world = MyWorld(self.dt)


	# def init_glut(self,window_size = (800,600)):
	# 	self.glutwindow = GLUTWindow(self.world, "FWMAV")
	# 	GLUT.glutInit(())
	# 	GLUT.glutInitDisplayMode(GLUT.GLUT_RGBA |
	# 							 GLUT.GLUT_DOUBLE |
	# 							 GLUT.GLUT_MULTISAMPLE |
	# 							 GLUT.GLUT_ALPHA |
	# 							 GLUT.GLUT_DEPTH)
	# 	GLUT.glutInitWindowSize(*window_size)
	# 	GLUT.glutInitWindowPosition(0, 0)
	# 	self.glutwindow.window = GLUT.glutCreateWindow(self.glutwindow.title)
	# 	self.glutwindow.initGL(*window_size)
	# 	self.camera_theta = 75#, for mirror 85
	# 	self.camera_phi = 135#, for mirror -75
	# 	self.camera_horizontal = 0.0
	# 	self.camera_vertical = -0.25
	# 	self.camera_depth = -1.25
	# 	self.camera_angle_increment = 5
	# 	self.camera_position_increment = 0.05
	# 	self.update_camera()
	# 	#self.glutwindow.scene.add_camera(Trackball(theta = self.camera_theta, phi = self.camera_phi, trans=[self.camera_horizontal, self.camera_vertical, self.camera_depth]),"Camera Z up close")
	# 	#self.glutwindow.scene.set_camera(2)
	# 	self.glutwindow.scene.resize(*window_size)
	# 	self.glutwindow.drawGL()

	# def update_camera(self):
	# 	self.glutwindow.scene.replace_camera(0,Trackball(theta = self.camera_theta, phi = self.camera_phi, trans=[self.camera_horizontal, self.camera_vertical, self.camera_depth]))
	# 	self.glutwindow.scene.set_camera(0)
	# 	self.glutwindow.drawGL()
	# 	print("theta = %.4f, phi = %.4f" % (self.camera_theta, self.camera_phi))
	# 	print("\r")

	# # def run(self,policy = None):
	# 	keylistener = KeyListener(1,'Key Thread', 1)
	# 	keylistener.start()
	# 	self.pid_policy = PIDController(self.dt_c)
	# 	if policy != 'pid' and policy !=None:
	# 		policy = MyPolicy(policy)
	# 	control_action = np.zeros(4)

	# 	print("\n")
	# 	print("space bar: simulation on/off", end="\n\r")
	# 	print("'r': reset simulation", end="\n\r")
	# 	print("'w, a, s, d' to rotate camera", end="\n\r")
	# 	print("'z, x' to zoom camera", end="\n\r")
	# 	print("'e, c' to adjust camera height", end="\n\r")
	# 	print("'v': toggle visulization", end="\n\r")
	# 	print("'q': quit", end="\n\r")

	# 	# main loop
	# 	while True:
	# 		key = keylistener.last_key_press
	# 		if key == 'q':
	# 			print("Simulation terminated, buh bye!", end="\n\r")
	# 			break
	# 		elif key == 'r':
	# 			self.reset()
	# 		elif key == 'o':
	# 			self.record_data()
	# 		elif key == 'm':
	# 			self.random_init()
	# 		elif key == ' ':
	# 			self.sim_on = not self.sim_on
	# 		elif key == 'v':
	# 			self.visulization = not self.visulization
	# 			print("Visulization = %r" % self.visulization, end="\n\r")
	# 		elif key == 'w':
	# 			self.camera_theta -= self.camera_angle_increment
	# 			self.update_camera()
	# 		elif key == 's':
	# 			self.camera_theta += self.camera_angle_increment
	# 			self.update_camera()
	# 		elif key == 'a':
	# 			self.camera_phi -= self.camera_angle_increment
	# 			self.update_camera()
	# 		elif key == 'd':
	# 			self.camera_phi += self.camera_angle_increment
	# 			self.update_camera()
	# 		elif key == 'z':
	# 			self.camera_depth += self.camera_position_increment
	# 			self.update_camera()
	# 		elif key == 'x':
	# 			self.camera_depth -= self.camera_position_increment
	# 			self.update_camera()
	# 		elif key == 'e':
	# 			self.camera_vertical += self.camera_position_increment
	# 			self.update_camera()
	# 		elif key == 'c':
	# 			self.camera_vertical -= self.camera_position_increment
	# 			self.update_camera()
	# 		keylistener.last_key_press = None

	# 		if self.world.time()>=63.0:
	# 			self.sim_on = False

	# 		if self.sim_on:
	# 			if self.paused:
	# 				print("Simulation running", end="\n\r")
	# 				self.paused = False
	# 				self.start_time = time.time()
	# 				self.next_visulizaiton_time = 1/self.fps + time.time()

	# 			if self.world.time()>=self.next_control_time:
	# 				self.next_control_time += self.dt_c
	# 				# print('update control', end="\n\r")
	# 				if policy == None:
	# 					control_action = [10,0,0,0]
	# 				elif policy == 'pid':
	# 					control_action = np.squeeze(self.pid_policy.get_action(self.states,self.dt_c,self.world.time(),self.sensor_fusion))
	# 				else:
	# 					action = policy.get_action(self.observation)
	# 					action_pid = np.squeeze(self.pid_policy.get_action(self.states,self.dt_c,self.sensor_fusion))
	# 					control_action = action + action_pid

	# 			self.step(control_action)
	# 			# if policy == None:
	# 			# 	for i in range(20):
	# 			# 		self.step([10,0,0,0])

	# 			# elif policy == 'pid':
	# 			# 	action_pid = np.squeeze(self.pid_policy.get_action(self.states,0.002))
	# 			# 	for i in range(20):
	# 			# 		self.step(action_pid)

	# 			# else:
	# 			# 	action = policy.get_action(self.observation)
	# 			# 	for i in range(20):
	# 			# 		self.step(action)
					
				
	# 			if time.time() >= self.next_visulizaiton_time:
	# 				if self.world.time()>=63.0:
	# 					self.sim_on = not self.sim_on
	# 				if self.visulization:
	# 					self.glutwindow.drawGL()
	# 				self.next_visulizaiton_time = self.next_visulizaiton_time + 1/self.fps

	# 		else:
	# 			if not self.paused:
	# 				self.glutwindow.drawGL()
	# 				self.elapse_time += (time.time() - self.start_time)
	# 				print("Simulation paused, running time = %.4f" % self.elapse_time, end="\n\r")
	# 			self.paused = True
	# 			time.sleep(0.01)
	# 	return

# class MyPolicy():
# 	def __init__(self, policy_name):
# 		import pickle
# 		import os

# 		log_dir = os.path.join(os.getcwd(),'data')

# 		with open(os.path.join(log_dir,policy_name), 'rb') as input:
# 			policy = pickle.load(input)

# 		self.h0w = policy._cached_params[()][0].get_value()
# 		self.h0b = policy._cached_params[()][1].get_value()
# 		self.h1w = policy._cached_params[()][2].get_value()
# 		self.h1b = policy._cached_params[()][3].get_value()
# 		self.ow = policy._cached_params[()][4].get_value()
# 		self.ob = policy._cached_params[()][5].get_value()

# 		# action lb ub are set in fwmav_sim_env.py
# 		self.action_lb = np.array([-5.0, -3, -3.5, -0.15])
# 		self.action_ub = np.array([8.0, 3, 3.5, 0.15])

# 		# ologstd = policy._cached_params[()][6].get_value()
# 	def get_action(self,observation):
# 		h0_out = np.tanh(np.matmul(observation.T, self.h0w) + self.h0b)
# 		h1_out = np.tanh(np.matmul(h0_out, self.h1w) + self.h1b)
# 		action = np.tanh(np.matmul(h1_out, self.ow) + self.ob)

# 		# scale action
# 		scaled_action = self.action_lb + (action + 1.) * 0.5 * (self.action_ub - self.action_lb)
# 		scaled_action = np.clip(scaled_action, self.action_lb, self.action_ub)

# 		return scaled_action

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
		# visulize force not working
		# pl0 = self.skeletons[1].bodynode('left_wing').C
		# pl1 = pl0 + 0.01 * np.array([1,0,0])
		# ri.set_color(1.0, 0.0, 0.0)
		# ri.render_arrow(pl0, pl1, r_base=0.05, head_width=0.1, head_len=0.1)


# keyboard input thread
# class KeyListener(threading.Thread):
# 	def __init__(self,threadID,name,counter):
# 		threading.Thread.__init__(self)
# 		self.threadID = threadID
# 		self.name = name
# 		self.counter = counter
# 		self.last_key_press = None
# 		self.new_key_pressed = False

# 	def run(self):
# 		print("Key listening thread started")
# 		# print('Starting ' + self.name)
# 		while True:
# 			self.last_key_press = click.getchar()
# 			if self.last_key_press == 'q':
# 				print("Key listening thread terminated")
# 				break
