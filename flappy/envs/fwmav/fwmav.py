##########################  FWMAV Simulation  #########################
# Version 0.3
# Fan Fei		Feb 2019
# Direct motor driven flapping wing MAV simulation
#######################################################################

import numpy as np
import random
import pydart2 as pydart

from types import SimpleNamespace

from flappy.envs.Wing import Wing


class FWMAV:
	def __init__(self, mav_config, world, dt_d):
		config = SimpleNamespace(**mav_config)
		self.id = config.id
		self.frequency = config.frequency
		self.urdf_file = config.urdf_file
		self.dt = 0
		
		self.left_spring_stiffness = config.left_spring_stiffness
		self.right_spring_stiffness = config.right_spring_stiffness
		self.left_stroke_lower = config.left_stroke_lower
		self.left_stroke_upper = config.left_stroke_upper
		self.left_rotate_lower = config.left_rotate_lower
		self.left_rotate_upper = config.left_rotate_upper
		self.right_stroke_lower = config.right_stroke_lower
		self.right_stroke_upper = config.right_stroke_upper
		self.right_rotate_lower = config.right_rotate_lower
		self.right_rotate_upper = config.right_rotate_upper
		self.left_stroke_mid = config.left_stroke_mid
		self.right_stroke_mid = config.right_stroke_mid
		
		self.left_wing = Wing(0,
							  config.wing_length,
							  config.mean_chord,
							  config.r33,
							  config.r22,
							  config.r11,
							  config.r00,
							  config.z_cp2,
							  config.z_cp1,
							  config.z_cp0,
							  config.z_rd,
							  config.left_shoulder_width,
							  config.stroke_plane_offset)
		self.right_wing = Wing(1,
							  config.wing_length,
							  config.mean_chord,
							  config.r33,
							  config.r22,
							  config.r11,
							  config.r00,
							  config.z_cp2,
							  config.z_cp1,
							  config.z_cp0,
							  config.z_rd,
							  config.right_shoulder_width,
							  config.stroke_plane_offset)

		# self.left_wing = Wing(0, config.left_shoulder_width, config.stroke_plane_offset)
		# self.right_wing = Wing(1, config.right_shoulder_width, config.stroke_plane_offset)
		self.add_flapper(world, z=0.0)

		self.nominal_torso_mass = self.flapper_skel.bodynode('torso').mass()
		self.nominal_torso_inertia = self.flapper_skel.bodynode('torso').inertia()
		self.nominal_left_wing_mass = self.flapper_skel.bodynode('left_wing').mass()
		self.nominal_left_wing_inertia = self.flapper_skel.bodynode('left_wing').inertia()
		self.nominal_right_wing_mass = self.flapper_skel.bodynode('right_wing').mass()
		self.nominal_right_wing_inertia = self.flapper_skel.bodynode('right_wing').inertia()
		self.nominal_left_leading_edge_mass = self.flapper_skel.bodynode('left_leading_edge').mass()
		self.nominal_left_leading_edge_inertia = self.flapper_skel.bodynode('left_leading_edge').inertia()
		self.nominal_right_leading_edge_mass = self.flapper_skel.bodynode('right_leading_edge').mass()
		self.nominal_right_leading_edge_inertia = self.flapper_skel.bodynode('right_leading_edge').inertia()

		self.states = {	'body_positions': 				np.zeros([6,1]),
						'body_velocities': 				np.zeros([6,1]),
						'body_accelerations': 			np.zeros([6,1]),
						'body_spatial_velocities': 		np.zeros([3,1]),
						'body_spatial_accelerations': 	np.zeros([3,1]),
						'left_stroke_angle': 			0,
						'left_stroke_velocity': 		0,
						'left_rotate_angle': 			0,
						'left_rotate_velocity': 		0,
						'left_stroke_acceleration': 	0,
						'right_stroke_angle': 			0,
						'right_stroke_velocity': 		0,
						'right_rotate_angle': 			0,
						'right_rotate_velocity': 		0,
						'right_stroke_acceleration': 	0}

		self.left_motor = Actuator(config.left_motor_properties)
		self.right_motor = Actuator(config.right_motor_properties)
		self.driver_update_time = 0
		self.dt_driver = dt_d

		self.config = config
		self.reset() # call reset before simulation

	def randomize(self):
		k_rand_mech = 0.07	# 5 percent
		k_rand_mass = 0.1	# 10 percent
		k_rand_motor = 0.1	# 10 percent increase
		k_rand_mid_stroke = 0.05	# 2.86 deg

		# mechanical trim (angle limit, angle mid, spring stiffness)
		self.left_spring_stiffness = self.config.left_spring_stiffness * np.random.normal(1, k_rand_mech)
		self.right_spring_stiffness = self.config.right_spring_stiffness * np.random.normal(1, k_rand_mech)

		self.left_rotate_lower = self.config.left_rotate_lower * np.random.normal(1, k_rand_mech)
		self.left_rotate_upper = self.config.left_rotate_upper * np.random.normal(1, k_rand_mech)
		self.right_rotate_lower = self.config.right_rotate_lower * np.random.normal(1, k_rand_mech)
		self.right_rotate_upper = self.config.right_rotate_upper * np.random.normal(1, k_rand_mech)

		self.left_stroke_mid = np.random.normal(0, k_rand_mid_stroke)
		self.right_stroke_mid = np.random.normal(0, k_rand_mid_stroke)

		# mass properities
		k_rand_mass_torso = np.random.normal(1, k_rand_mass)
		self.flapper_skel.bodynode('torso').set_mass(self.nominal_torso_mass * k_rand_mass_torso)
		self.flapper_skel.bodynode('torso').set_inertia(self.nominal_torso_inertia * k_rand_mass_torso)

		k_rand_mass_left_wing = np.random.normal(1, k_rand_mass)
		self.flapper_skel.bodynode('left_wing').set_mass(self.nominal_left_wing_mass * k_rand_mass_left_wing)
		self.flapper_skel.bodynode('left_wing').set_inertia(self.nominal_left_wing_inertia * k_rand_mass_left_wing)

		k_rand_mass_right_wing = np.random.normal(1, k_rand_mass)
		self.flapper_skel.bodynode('right_wing').set_mass(self.nominal_right_wing_mass * k_rand_mass_right_wing)
		self.flapper_skel.bodynode('right_wing').set_inertia(self.nominal_right_wing_inertia * k_rand_mass_right_wing)

		k_rand_mass_left_leading_edge = np.random.normal(1, k_rand_mass)
		self.flapper_skel.bodynode('left_leading_edge').set_mass(self.nominal_left_leading_edge_mass * k_rand_mass_left_leading_edge)
		self.flapper_skel.bodynode('left_leading_edge').set_inertia(self.nominal_left_leading_edge_inertia * k_rand_mass_left_leading_edge)

		k_rand_mass_right_leading_edge = np.random.normal(1, k_rand_mass)
		self.flapper_skel.bodynode('right_leading_edge').set_mass(self.nominal_right_leading_edge_mass * k_rand_mass_right_leading_edge)
		self.flapper_skel.bodynode('right_leading_edge').set_inertia(self.nominal_right_leading_edge_inertia * k_rand_mass_right_leading_edge)

		# motor trim (resistance)
		self.left_motor.resistance = self.left_motor.config.resistance * (1+np.abs(np.random.normal(0,k_rand_motor)))
		self.right_motor.resistance = self.right_motor.config.resistance * (1+np.abs(np.random.normal(0,k_rand_motor)))

		# configure
		self.configure_flapper()
		return

	def nominal(self):
		self.left_spring_stiffness = self.config.left_spring_stiffness
		self.right_spring_stiffness = self.config.right_spring_stiffness
		self.left_rotate_lower = self.config.left_rotate_lower
		self.left_rotate_upper = self.config.left_rotate_upper
		self.right_rotate_lower = self.config.right_rotate_lower
		self.right_rotate_upper = self.config.right_rotate_upper
		self.left_stroke_mid = self.config.left_stroke_mid
		self.right_stroke_mid = self.config.right_stroke_mid

		self.flapper_skel.bodynode('torso').set_mass(self.nominal_torso_mass)
		self.flapper_skel.bodynode('torso').set_inertia(self.nominal_torso_inertia)
		self.flapper_skel.bodynode('left_wing').set_mass(self.nominal_left_wing_mass)
		self.flapper_skel.bodynode('left_wing').set_inertia(self.nominal_left_wing_inertia)
		self.flapper_skel.bodynode('right_wing').set_mass(self.nominal_right_wing_mass)
		self.flapper_skel.bodynode('right_wing').set_inertia(self.nominal_right_wing_inertia)
		self.flapper_skel.bodynode('left_leading_edge').set_mass(self.nominal_left_leading_edge_mass)
		self.flapper_skel.bodynode('left_leading_edge').set_inertia(self.nominal_left_leading_edge_inertia)
		self.flapper_skel.bodynode('right_leading_edge').set_mass(self.nominal_right_leading_edge_mass)
		self.flapper_skel.bodynode('right_leading_edge').set_inertia(self.nominal_right_leading_edge_inertia)

		self.left_motor.resistance = self.left_motor.config.resistance
		self.right_motor.resistance = self.right_motor.config.resistance

		# configure
		self.configure_flapper()
		return

	def get_states(self):
		self.update_body_states()
		return self.states

	def reset(self):
		self.configure_flapper(roll = 0/180*np.pi, pitch = 0/180*np.pi, yaw = 0/180*np.pi, x=0.0, y = 0.0, z=0.0)
		self.left_motor.reset()
		self.right_motor.reset()
		self.driver_update_time = 0
		states = self.get_states()
		# print("Flapper reset")
		return states

	def step(self, t, input_voltage):
		status = 1

		# aero
		self.left_wing.UpdateStates(self.flapper_skel.bodynode('torso').com_spatial_velocity()[0],
									self.flapper_skel.bodynode('torso').com_spatial_velocity()[1],
									self.flapper_skel.bodynode('torso').com_spatial_velocity()[2],
									self.flapper_skel.bodynode('torso').com_spatial_velocity()[3],
									self.flapper_skel.bodynode('torso').com_spatial_velocity()[4],
									self.flapper_skel.bodynode('torso').com_spatial_velocity()[5],
									0,
									0,
									self.flapper_skel.positions()[self.flapper_skel.dof('left_stroke').id],
									self.flapper_skel.velocities()[self.flapper_skel.dof('left_stroke').id],
									0,
									0,
									self.flapper_skel.positions()[self.flapper_skel.dof('left_rotate').id],
									self.flapper_skel.velocities()[self.flapper_skel.dof('left_rotate').id])

		self.right_wing.UpdateStates(self.flapper_skel.bodynode('torso').com_spatial_velocity()[0],
									self.flapper_skel.bodynode('torso').com_spatial_velocity()[1],
									self.flapper_skel.bodynode('torso').com_spatial_velocity()[2],
									self.flapper_skel.bodynode('torso').com_spatial_velocity()[3],
									self.flapper_skel.bodynode('torso').com_spatial_velocity()[4],
									self.flapper_skel.bodynode('torso').com_spatial_velocity()[5],
									0,
									0,
									self.flapper_skel.positions()[self.flapper_skel.dof('right_stroke').id],
									self.flapper_skel.velocities()[self.flapper_skel.dof('right_stroke').id],
									0,
									0,
									self.flapper_skel.positions()[self.flapper_skel.dof('right_rotate').id],
									self.flapper_skel.velocities()[self.flapper_skel.dof('right_rotate').id])
		
		self.left_wing.UpdateAeroForce()
		self.right_wing.UpdateAeroForce()

		# update voltage
		if t >= self.driver_update_time:
			self.driver_update_time +=self.dt_driver
			self.left_motor.update_driver_voltage(input_voltage[0])
			self.right_motor.update_driver_voltage(input_voltage[1])

		# update torque
		self.left_motor.update_torque(self.flapper_skel.velocities()[self.flapper_skel.dof('left_stroke').id],
										self.flapper_skel.accelerations()[self.flapper_skel.dof('left_stroke').id])
		self.right_motor.update_torque(self.flapper_skel.velocities()[self.flapper_skel.dof('right_stroke').id],
										self.flapper_skel.accelerations()[self.flapper_skel.dof('right_stroke').id])

		# apply stroke torque
		torques = np.zeros(self.flapper_skel.num_dofs())
		torques[self.flapper_skel.dof('left_stroke').id] = self.left_motor.get_torque()
		torques[self.flapper_skel.dof('right_stroke').id] = self.right_motor.get_torque()
		self.flapper_skel.set_forces(torques)
		
		# get aero forces
		left_FN = np.array([self.left_wing.GetNormalForce(), 0, 0])	# in wing x direction
		right_FN = 	np.array([self.right_wing.GetNormalForce(), 0, 0])
		left_CoP = np.array([0, self.left_wing.GetSpanCoP(), (-1)*self.left_wing.GetChordCoP()])
		right_CoP = np.array([0, (-1)*self.right_wing.GetSpanCoP(), (-1)*self.right_wing.GetChordCoP()])
		left_M_rd = np.array([0, self.left_wing.GetM_rd(), 0])		# in wing y direction
		right_M_rd = np.array([0, self.right_wing.GetM_rd(), 0])

		# apply aero force and moment on wing
		self.flapper_skel.bodynode('left_wing').add_ext_force(left_FN, left_CoP, True, True)
		self.flapper_skel.bodynode('right_wing').add_ext_force(right_FN, right_CoP, True, True)
		self.flapper_skel.bodynode('left_wing').add_ext_torque(left_M_rd, True)
		self.flapper_skel.bodynode('right_wing').add_ext_torque(right_M_rd, True)



	def update_body_states(self):
		# body rpyxyz
		body_positions_R_ = self.flapper_skel.bodynode('torso').world_transform()
		self.states['body_positions'][0] = np.arctan2(body_positions_R_[2,1],body_positions_R_[2,2])
		self.states['body_positions'][1] = np.arcsin(-body_positions_R_[2,0])
		self.states['body_positions'][2] = np.arctan2(body_positions_R_[1,0],body_positions_R_[0,0])
		self.states['body_positions'][3] = self.flapper_skel.bodynode('torso').com()[0]
		self.states['body_positions'][4] = self.flapper_skel.bodynode('torso').com()[1]
		self.states['body_positions'][5] = self.flapper_skel.bodynode('torso').com()[2]

		# body frame velocity (these tow methods are identical)
		self.states['body_velocities'] = self.flapper_skel.bodynode('torso').com_spatial_velocity().reshape(6,1)
		
		# body frame acceleration (these tow methods are identical)
		self.states['body_accelerations'] = self.flapper_skel.bodynode('torso').com_spatial_acceleration().reshape(6,1)

		# spatial frame velocity
		self.states['body_spatial_velocities'] = self.flapper_skel.bodynode('torso').com_linear_velocity().reshape(3,1)

		# spatial frame acceleration
		self.states['body_spatial_accelerations'] = self.flapper_skel.bodynode('torso').com_linear_acceleration().reshape(3,1)

		# left wing
		self.states['left_stroke_angle'] = self.flapper_skel.positions()[self.flapper_skel.dof('left_stroke').id]
		self.states['left_stroke_velocity'] = self.flapper_skel.velocities()[self.flapper_skel.dof('left_stroke').id]
		self.states['left_rotate_angle'] = self.flapper_skel.positions()[self.flapper_skel.dof('left_rotate').id]
		self.states['left_rotate_velocity'] = self.flapper_skel.velocities()[self.flapper_skel.dof('left_rotate').id]
		self.states['left_stroke_acceleration'] = self.flapper_skel.accelerations()[self.flapper_skel.dof('left_stroke').id]
		
		# right wing
		self.states['right_stroke_angle'] = self.flapper_skel.positions()[self.flapper_skel.dof('right_stroke').id]
		self.states['right_stroke_velocity'] = self.flapper_skel.velocities()[self.flapper_skel.dof('right_stroke').id]
		self.states['right_rotate_angle'] = self.flapper_skel.positions()[self.flapper_skel.dof('right_rotate').id]
		self.states['right_rotate_velocity'] = self.flapper_skel.velocities()[self.flapper_skel.dof('right_rotate').id]
		self.states['right_stroke_acceleration'] = self.flapper_skel.accelerations()[self.flapper_skel.dof('right_stroke').id]

		# notes
		# for world frame velocity
		# self.flapper_skel.velocities()[self.flapper_skel.dof('torso_to_world_rot_x').id] = self.flapper_skel.bodynode('torso').com_spatial_velocity()[0]
	
	def set_states(self, positions, velocities):
		# totle of 10 dofs, 20 states to set (position, velocity)
		# input should be two 10x1 column np array
		# see below commentted code for syntax.
		states = np.concatenate((positions,velocities), axis = 0)
		self.flapper_skel.set_states(states.reshape(-1))

		# Same as the below method
		# # body rpyxyz
		# self.flapper_skel.dof('torso_to_world_rot_x').set_position(positions[0])
		# self.flapper_skel.dof('torso_to_world_rot_y').set_position(positions[1])
		# self.flapper_skel.dof('torso_to_world_rot_z').set_position(positions[2])
		# self.flapper_skel.dof('torso_to_world_pos_x').set_position(positions[3])
		# self.flapper_skel.dof('torso_to_world_pos_y').set_position(positions[4])
		# self.flapper_skel.dof('torso_to_world_pos_z').set_position(positions[5])
		
		# # body velocity
		# self.flapper_skel.dof('torso_to_world_rot_x').set_velocity(velocities[0])
		# self.flapper_skel.dof('torso_to_world_rot_y').set_velocity(velocities[1])
		# self.flapper_skel.dof('torso_to_world_rot_z').set_velocity(velocities[2])
		# self.flapper_skel.dof('torso_to_world_pos_x').set_velocity(velocities[3])
		# self.flapper_skel.dof('torso_to_world_pos_y').set_velocity(velocities[4])
		# self.flapper_skel.dof('torso_to_world_pos_z').set_velocity(velocities[5])

		# # left wing
		# self.flapper_skel.dof('left_stroke').set_position(positions[6])
		# self.flapper_skel.dof('left_rotate').set_position(positions[7])
		# self.flapper_skel.dof('left_stroke').set_velocity(velocities[6])
		# self.flapper_skel.dof('left_rotate').set_velocity(velocities[7])

		# # right wing
		# self.flapper_skel.dof('right_stroke').set_position(positions[8])
		# self.flapper_skel.dof('right_rotate').set_position(positions[9])
		# self.flapper_skel.dof('right_stroke').set_velocity(velocities[8])
		# self.flapper_skel.dof('right_rotate').set_velocity(velocities[9])



	def add_flapper(self, world,
					roll = 0,
					pitch = 0,
					yaw = 0,
					x = 0,
					y = 0,
					z = 0,
					left_stroke = 0,
					left_rotate = 0,
					right_stroke = 0,
					right_rotate = 0):

		self.flapper_skel = world.add_skeleton(self.urdf_file)
		self.configure_flapper(roll, pitch, yaw, x, y, z, left_stroke, left_rotate, right_stroke, right_rotate)
		print('add_flapper OK')


	def configure_flapper(self,
							roll = 0,
							pitch = 0,
							yaw = 0,
							x = 0,
							y = 0,
							z = 0,
							left_stroke = 0,
							left_rotate = 0,
							right_stroke = 0,
							right_rotate = 0):
		
		# print(self.flapper_skel.self_collision_check())
		self.flapper_skel.set_self_collision_check(True)

		# joints
		# self.left_stroke_joint_id = self.flapper_skel.joint('left_stroke').id
		# self.left_rotate_joint_id = self.flapper_skel.joint('left_rotate').id
		# self.right_stroke_joint_id = self.flapper_skel.joint('right_stroke').id
		# self.right_rotate_joint_id = self.flapper_skel.joint('right_rotate').id

		# dofs
		# self.left_stroke_dof_id = self.flapper_skel.dof('left_stroke').id
		# self.left_rotate_dot_id = self.flapper_skel.dof('left_rotate').id
		# self.right_stroke_dof_id = self.flapper_skel.dof('right_stroke').id
		# self.right_rotate_dof_id = self.flapper_skel.dof('right_rotate').id
		
		# joint limit
		self.flapper_skel.joint('left_stroke').set_position_lower_limit(0, self.left_stroke_lower)
		self.flapper_skel.joint('left_stroke').set_position_upper_limit(0, self.left_stroke_upper)
		self.flapper_skel.joint('left_rotate').set_position_lower_limit(0, self.left_rotate_lower)
		self.flapper_skel.joint('left_rotate').set_position_upper_limit(0, self.left_rotate_upper)
		self.flapper_skel.joint('right_stroke').set_position_lower_limit(0, self.right_stroke_lower)
		self.flapper_skel.joint('right_stroke').set_position_upper_limit(0, self.right_stroke_upper)
		self.flapper_skel.joint('right_rotate').set_position_lower_limit(0, self.right_rotate_lower)
		self.flapper_skel.joint('right_rotate').set_position_upper_limit(0, self.right_rotate_upper)

		self.flapper_skel.joint('left_stroke').set_position_limit_enforced(True)
		self.flapper_skel.joint('left_rotate').set_position_limit_enforced(True)
		self.flapper_skel.joint('right_stroke').set_position_limit_enforced(True)
		self.flapper_skel.joint('right_rotate').set_position_limit_enforced(True)

		self.flapper_skel.joint('left_stroke').set_spring_stiffness(0, self.left_spring_stiffness)
		self.flapper_skel.joint('right_stroke').set_spring_stiffness(0, self.right_spring_stiffness)
		
		# set joint neutural position trim
		self.flapper_skel.joint('left_stroke').set_rest_position(0,self.left_stroke_mid)
		self.flapper_skel.joint('right_stroke').set_rest_position(0,self.right_stroke_mid)
		left_stroke = self.left_stroke_mid
		right_stroke = self.right_stroke_mid
		
		# set initial position
		self.flapper_skel.dof('torso_to_world_rot_x').set_position(roll)
		self.flapper_skel.dof('torso_to_world_rot_y').set_position(pitch)
		self.flapper_skel.dof('torso_to_world_rot_z').set_position(yaw)
		self.flapper_skel.dof('torso_to_world_pos_x').set_position(x)
		self.flapper_skel.dof('torso_to_world_pos_y').set_position(y)
		self.flapper_skel.dof('torso_to_world_pos_z').set_position(z)
		self.flapper_skel.dof('left_stroke').set_position(left_stroke)
		self.flapper_skel.dof('left_rotate').set_position(left_rotate)
		self.flapper_skel.dof('right_stroke').set_position(right_stroke)
		self.flapper_skel.dof('right_rotate').set_position(right_rotate)

		# create empty torque
		self.torques = np.zeros(self.flapper_skel.num_dofs())

		# testing
		# fix flapper for debugging
		# self.flapper_skel.joint('torso_to_world').set_actuator_type(pydart.joint.Joint.LOCKED)
		# self.flapper_skel.joint('left_rotate').set_actuator_type(pydart.joint.Joint.LOCKED)
		# self.flapper_skel.joint('right_rotate').set_actuator_type(pydart.joint.Joint.LOCKED)
		# print(self.flapper_skel.dof(9))
		# print(self.flapper_skel.joint('left_stroke').position_lower_limit(0))
		# get dof id in body skeleton
		# print(self.flapper_skel.dof('right_stroke').id)

class Actuator:
	def __init__(self,motor_properties):
		config = SimpleNamespace(**motor_properties)
		self.resistance = config.resistance
		self.torque_constant = config.torque_constant
		self.gear_ratio = config.gear_ratio
		self.mechanical_efficiency = config.mechanical_efficiency
		self.friction_coefficient = config.friction_coefficient
		self.damping_coefficient = config.damping_coefficient
		self.inertia = config.inertia

		self.inertia_torque = 0
		self.damping_torque = 0
		self.friction_torque = 0
		self.magnetic_torque = 0
		self.motor_torque =0

		self.voltage = 0

		self.current = 0
		self.back_EMF = 0
		self.output_torque = 0
		self.config = config
		self.reset()

	def update_driver_voltage(self, voltage):
		self.voltage = voltage

	def update_torque(self, stroke_velocity, stroke_acceleration):

		psi_dot = stroke_velocity
		psi_ddot = stroke_acceleration
		motor_vel = psi_dot*self.gear_ratio
		motor_accel = psi_ddot*self.gear_ratio
		if psi_dot>0:
			sign = 1
		elif psi_dot<0:
			sign = -1
		else:
			sign = 0

		self.back_EMF = self.torque_constant*motor_vel
		self.current = (self.voltage-self.back_EMF)/self.resistance

		self.inertia_torque = self.inertia*motor_accel
		self.damping_torque = self.damping_coefficient*motor_vel
		self.friction_torque = self.friction_coefficient*sign
		self.magnetic_torque = self.torque_constant*self.current

		self.motor_torque = self.magnetic_torque-self.inertia_torque-self.damping_torque-self.friction_torque

		self.output_torque = self.motor_torque*self.gear_ratio*self.mechanical_efficiency


	def get_torque(self):
		return self.output_torque

	def reset(self):
		self.inertia_torque = 0
		self.damping_torque = 0
		self.friction_torque = 0
		self.magnetic_torque = 0
		self.motor_torque =0

		self.current = 0
		self.back_EMF = 0
		self.output_torque = 0



