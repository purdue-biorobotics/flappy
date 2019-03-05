import numpy as np

class pid:
	def __init__(self):
		self.old_error = 0
		self.integral = 0
		self.int_max = 0
		self.Kp = 0
		self.Ki = 0
		self.Kd = 0
		self.p = 0
		self.i = 0
		self.d = 0

class ARCController():
	def __init__(self,dt):
		self.dt_ = dt
		# desired target
		self.desired_accel_x_ = 0
		self.desired_accel_y_ = 0
		
		self.pos_target_x_ = 0
		self.pos_target_y_ = 0
		self.pos_target_z_ = 0
		self.vel_target_z_ = 0

		self.ang_ef_target_z_ = 0
		self.rate_ef_target_z_ = 0

		####################################### z controller #######################################
		# error
		self.z_eq_dot_old = 0

		# position target filter
		self.pos_target_z_alt_filtered_ = 0
		self.pos_target_z_alt_filtered_old_ = 0
		self.vel_target_z_alt_filtered_ = 0
		tau_z = 1.5			# time constant
		self.alpha_z_target = dt/(tau_z+dt)
		tau_z_dot = 0.05			# time constant
		self.alpha_z_dot_target = dt/(tau_z_dot+dt)
		
		# system parameters
		self.K_Fz = 0.0165	# Fz = K_Fz*(u_z - V_s) from force mapping
		self.V_s = 2.5115
		self.mass = 0.012

		# parameter estimate
		self.theta_hat_z = np.zeros([3,1])
		self.theta_hat_z[0,0] = self.mass
		self.theta_hat_z[1,0] = 1				# constant and not updated
		self.theta_hat_z[2,0] = 0.0001			# disturbance in z direction in the unit of Newton

		self.theta_hat_z_min = np.zeros([3,1])
		self.theta_hat_z_min[0,0] = 0.011			# 11 gram
		self.theta_hat_z_min[1,0] = 1				# not update
		self.theta_hat_z_min[2,0] = -0.049		# 5 gram = 0.049 N

		self.theta_hat_z_max = np.zeros([3,1])
		self.theta_hat_z_max[0,0] = 0.018			# 11 gram
		self.theta_hat_z_max[1,0] = 1				# not update
		self.theta_hat_z_max[2,0] = 0.049			# 5 gram = 0.049 N

		self.h_z = (0.018-0.012) + (0.098) + 0.01	#18-12 gram for mass estimation + +-5gram for disturbance + 0.01N(~1gram) for uncertainty
		self.h_z = np.sum(self.theta_hat_z_max-self.theta_hat_z_min) + 0.01	#18-12 gram for mass estimation + +-5gram for disturbance + 0.01N(~1gram) for uncertainty
		# print('h_z = %.4f' % self.h_z, end="\n\r")
		
		# regressor
		self.phi_z = np.zeros([3,1])
		self.phi_z[0,0] = -9.8
		self.phi_z[1,0] = -self.K_Fz*self.V_s
		self.phi_z[2,0] = 1

		# gains
		self.k_1_z = 1.5#*3								#
		self.k_s1_z = 0.10#*3
		self.epsilon_z = 0.12
		self.Gamma_z = np.eye(3)
		
		# gain tuning
		k_eq_z = 1			# controls I, large k_eq will result in oscilating theta and oscilating performance
		h_M_z = self.h_z
		self.epsilon_z = 1/(4*k_eq_z-self.k_1_z)*h_M_z*2
		#print('epsilon_z = %.4f' % self.epsilon_z, end="\n\r")
		#self.epsilon_z = 0.115
		zeta_z = 0.707
		k_i_z = k_eq_z**2/(4*zeta_z**2)
		#print('k_i_z = %.4f' % k_i_z, end="\n\r")
		W_z = np.diag([0.029,0,0.098])			# diag{max(|theta_max+theta_min|, theta_max-theta_min)}
		s_phi_z = np.asscalar(self.phi_z.transpose().dot(np.square(W_z)).dot(self.phi_z))
		#print('s_phi_z = %.4f' % s_phi_z, end="\n\r")
		gamma_z = k_i_z/s_phi_z
		gamma_z = 5.5343
		#print('gamma_z = %.4f' % gamma_z, end="\n\r")
		self.Gamma_z = gamma_z*np.square(W_z)
		self.Gamma_z[2,2] = self.Gamma_z[2,2]
		#print('Gamma_z=', end="\n\r")
		#print(self.Gamma_z, end="\n\r")

		####################################### xy controller #######################################

		# position target filter
		self.pos_target_x_filtered_ = 0
		self.pos_target_x_filtered_old_ = 0
		self.pos_target_x_filtered_old_2_ = 0
		self.vel_target_x_filtered_ = 0;
		self.acc_target_x_ = 0;
		self.acc_target_x_filtered_ = 0;
		self.jer_target_x_ = 0;
		self.jer_target_x_filtered_ = 0;

		self.pos_target_y_filtered_ = 0
		self.pos_target_y_filtered_old_ = 0
		self.pos_target_y_filtered_old_2_ = 0
		self.vel_target_y_filtered_ = 0;
		self.acc_target_y_ = 0;
		self.acc_target_y_filtered_ = 0;
		self.jer_target_y_ = 0;
		self.jer_target_y_filtered_ = 0;
		
		self.pos_target_z_filtered_ = 0
		self.pos_target_z_filtered_old_ = 0
		self.pos_target_z_filtered_old_2_ = 0
		self.vel_target_z_filtered_ = 0;
		self.acc_target_z_ = 0;
		self.acc_target_z_filtered_ = 0;
		self.jer_target_z_ = 0;
		self.jer_target_z_filtered_ = 0;

		ts_xy = 0.02			# settling time of xy
		zeta_xy = 1			# damping ratio of xy
		omega_n_xy = 5/(zeta_xy*ts_xy)		# natural frequency of xy
		self.a_2_xy_target = -1/(1 + 2*zeta_xy*omega_n_xy*dt + omega_n_xy**2*dt**2)
		self.b_1_xy_target = omega_n_xy**2*dt**2/(1 + 2*zeta_xy*omega_n_xy*dt + omega_n_xy**2*dt**2)
		self.a_1_xy_target = 1 - self.b_1_xy_target - self.a_2_xy_target
		tau_xy = 0.002			# time constant
		self.alpha_xy_target = dt/(tau_xy+dt)

		# system parameters
		self.tau_x = -0.2788e-3		# roll torque (Nm)
		self.K_Tx = 0.9068e-3		# (Nm/V)
		self.tau_y = 0.1009e-3		# pitch torque (Nm)
		self.K_Ty = 0.3199e-3		# (Nm/V)
		self.tau_z = 0.099e-3		# yaw torque (Nm)
		self.K_Tz = 1.8401e-3		# (Nm/splitcycle)
		self.I_x = 5000e-9			# kgm^2
		self.I_y = 3500e-9			# kgm^2
		self.I_z = 1800e-9			# kgm^2

		# parameter estimate
		self.theta_hat_xy = np.zeros([9,1])
		self.theta_hat_xy[0,0] = self.tau_x		# roll torque offsed
		self.theta_hat_xy[1,0] = self.tau_y		# pitch torque offsed
		self.theta_hat_xy[2,0] = self.tau_z		# yaw torque offsed
		self.theta_hat_xy[3,0] = 1				# constant and not updated
		self.theta_hat_xy[4,0] = 1				# constant and not updated
		self.theta_hat_xy[5,0] = 1				# constant and not updated
		self.theta_hat_xy[6,0] = 0			# disturbance in roll direction (1e-4Nm = 0.1Nmm)
		self.theta_hat_xy[7,0] = 0			# disturbance in pitch direction (1e-4Nm = 0.1Nmm)
		self.theta_hat_xy[8,0] = 0			# disturbance in yaw direction (1e-4Nm = 0.1Nmm)

		self.theta_hat_xy_min = np.zeros([9,1])
		self.theta_hat_xy_min[0,0] = -0.7e-3		# -0.7Nmm = -0.5V
		self.theta_hat_xy_min[1,0] = -0.25e-3		# -0.25Nmm = -1V
		self.theta_hat_xy_min[2,0] = -0.1e-3		# -0.1Nmm = -0.1
		self.theta_hat_xy_min[3,0] = 1				# constant and not updated
		self.theta_hat_xy_min[4,0] = 1				# constant and not updated
		self.theta_hat_xy_min[5,0] = 1				# constant and not updated
		self.theta_hat_xy_min[6,0] = -0.5e-3			# disturbance in roll direction (0.5Nmm)
		self.theta_hat_xy_min[7,0] = -0.25e-3			# disturbance in pitch direction (0.25Nmm)
		self.theta_hat_xy_min[8,0] = -0.02e-3			# disturbance in yaw direction (0.02Nmm)

		self.theta_hat_xy_max = np.zeros([9,1])
		self.theta_hat_xy_max[0,0] = 0.7e-3			# 0.6Nmm = 1V
		self.theta_hat_xy_max[1,0] = 0.25e-3		# 0.175Nmm = 0V
		self.theta_hat_xy_max[2,0] = 0.1e-3			# 0.1Nmm = 0
		self.theta_hat_xy_max[3,0] = 1				# constant and not updated
		self.theta_hat_xy_max[4,0] = 1				# constant and not updated
		self.theta_hat_xy_max[5,0] = 1				# constant and not updated
		self.theta_hat_xy_max[6,0] = 0.5e-3			# disturbance in roll direction (0.5Nmm)
		self.theta_hat_xy_max[7,0] = 0.25e-3		# disturbance in pitch direction (0.25Nmm)
		self.theta_hat_xy_max[8,0] = 0.02e-3			# disturbance in yaw direction (0.02Nmm)

		# regressor
		self.phi_xy = np.concatenate((np.eye(3),np.zeros([3,3]),np.eye(3)),axis = 0)	# 9x3 matrix

		self.h_xy = np.matmul(self.phi_xy.transpose(),(self.theta_hat_xy_max-self.theta_hat_xy_min)) + 0.2e-3	# 3x9 * 9x1 = 3x1   +0.2Nmm for uncertainty
		# print('h_xy =', end="\n\r")
		# print(self.h_xy, end="\n\r")

		self.omega_x_eq_old_ = 0
		self.omega_y_eq_old_ = 0
		self.omega_z_eq_old_ = 0


		# gains
		self.lambda_1 = 2
		self.lambda_1_pitch = 20
		self.lambda_1_roll = 10

		self.lambda_2 = 4
		self.lambda_2_pitch = 40
		self.lambda_2_roll = 40

		self.lambda_3 = 10
		self.lambda_3_pitch = 120#20
		self.lambda_3_roll = 100
		self.lambda_psi = 400

		self.k_s1_xy = np.array([0.7*self.K_Tx, 0.9*self.K_Ty, 0.001*self.K_Tz])
		self.Gamma_xy = np.eye(9)

		# gain tuning
		k_eq_x = 0.05#0.05-0.08			# controls I, large k_eq will result in oscilating theta and oscilating performance
		k_eq_y = 0.05#0.05-0.08			# controls I, large k_eq will result in oscilating theta and oscilating performance
		k_eq_yaw = 0.001
		h_M_xy = self.h_xy
		#self.epsilon_xy = 1/(4*k_eq_xy-self.lambda_3)*h_M_xy*2				# 3x1, not sure if correct, epsilon_xy mainly consists of the uncertainty term in h_xy
		self.epsilon_xy = np.array([[0.1], [0.1], [0.1]])
		# print('epsilon_xy =', end="\n\r")
		# print(self.epsilon_xy, end="\n\r")

		zeta_xy = 0.707
		k_i_x = k_eq_x**2/(4*zeta_xy**2)
		k_i_y = k_eq_y**2/(4*zeta_xy**2)
		k_i_yaw = k_eq_yaw**2/(4*zeta_xy**2)
		# print('k_i_x = %.4f' % k_i_x, end="\n\r")
		# print('k_i_y = %.4f' % k_i_y, end="\n\r")
		# print('k_i_yaw = %.4f' % k_i_yaw, end="\n\r")
		W_xy = np.diag([1.4e-3, 0.5e-3, 0.3e-3, 0, 0, 0, 1e-3, 0.5e-3, 0.04e-3])			# diag{max(|theta_max+theta_min|, theta_max-theta_min)}
		s_phi_xy = self.phi_xy.transpose().dot(np.square(W_xy)).dot(self.phi_xy)			# 3x9 * 9x9 * 9x3 = 3x3
		# print('s_phi_xy=', end="\n\r")
		# print(s_phi_xy, end="\n\r")
		gamma_xy = k_i_x*np.linalg.inv(s_phi_xy)
		gamma_y = k_i_y*np.linalg.inv(s_phi_xy)
		gamma_yaw = k_i_yaw*np.linalg.inv(s_phi_xy)
		gamma_xy[1,1] = gamma_y[1,1]
		gamma_xy[2,2] = gamma_yaw[2,2]
		#gamma_z = 5.5343
		# print('gamma_xy=', end="\n\r")
		# print(gamma_xy, end="\n\r")
		temp0 = np.concatenate((gamma_xy, np.zeros([3,6])), axis = 1)
		temp2 = np.concatenate((np.zeros([3,6]), gamma_xy), axis = 1)
		gamma_xy_99 = np.concatenate((temp0, np.zeros([3,9]), temp2),axis = 0)
		self.Gamma_xy = gamma_xy_99.dot(np.square(W_xy))
		# print('Gamma_xy=', end="\n\r")
		# print(self.Gamma_xy, end="\n\r")


		# control voltage limit
		self.differential_voltage_max_ = 2
		self.mean_voltage_max_ = 2.5
		self.split_cycle_max_ = 0.15
		self.hover_voltage_ = 9.3
		self.voltage_amplitude_max_ = 18

		self.voltage_amplitude_ = 12
		self.differential_voltage_ = 0
		self.mean_voltage_ = 0
		self.split_cycle_ = 0

		# state filter
		RC = 1/(2*np.pi*20)
		self.alpha = dt/(RC+dt)
		RC = 1/(2*np.pi*20)
		self.alpha_yaw = dt/(RC+dt)
		RC = 1/(2*np.pi*2)
		self.alpha_xyz = dt/(RC+dt)

		self.pos_current_x_ = 0
		self.pos_current_y_ = 0
		self.vel_current_x_ = 0
		self.vel_current_y_ = 0
		self.roll_angle_ = 0
		self.pitch_angle_ = 0
		self.yaw_angle_ = 0
		self.gyro_x_ = 0
		self.gyro_y_ = 0
		self.gyro_z_ = 0
		self.altitude_ = 0
		self.velocity_z_ = 0
		self.acceleration_z_ = 0
		self.raw_velocity_z_old = 0


	def get_action(self, observation):
		self.sensor_read(observation)
		self.controller_run()

		action = np.zeros([4],dtype=np.float64)

		action[0] = self.voltage_amplitude_
		action[1] = self.differential_voltage_
		action[2] = self.mean_voltage_
		action[3] = self.split_cycle_

		return action

	def controller_run(self):
		self.z_control()
		self.xy_control()

	def sensor_read(self, observation):
		self.vel_current_x_old_ = self.vel_current_x_
		self.vel_current_y_old_ = self.vel_current_y_

		#updat raw observation
		raw_pos_current_x_ = observation[9]
		raw_pos_current_y_ = observation[10]
		raw_vel_current_x_ = observation[12]
		raw_vel_current_y_ = observation[13]

		R = observation[0:9]
		[raw_roll_angle_, raw_pitch_angle_, raw_yaw_angle_] = self.rotation_to_euler_angle(R.reshape(3,3))

		raw_gyro_x_ = observation[15]
		raw_gyro_y_ = observation[16]
		raw_gyro_z_ = observation[17]
		raw_altitude_ = observation[11]
		raw_velocity_z_ = observation[14]
		raw_acceleration_z_ = (raw_velocity_z_ - self.raw_velocity_z_old)/self.dt_
		self.raw_velocity_z_old = raw_velocity_z_

		# filter with low pass
		self.pos_current_x_ = self.pos_current_x_*(1-self.alpha_xyz) + raw_pos_current_x_*self.alpha_xyz
		self.pos_current_y_ = self.pos_current_y_*(1-self.alpha_xyz) + raw_pos_current_y_*self.alpha_xyz
		self.vel_current_x_ = self.vel_current_x_*(1-self.alpha_xyz) + raw_vel_current_x_*self.alpha_xyz
		self.vel_current_y_ = self.vel_current_y_*(1-self.alpha_xyz) + raw_vel_current_y_*self.alpha_xyz
		self.roll_angle_ = self.roll_angle_*(1-self.alpha) + raw_roll_angle_*self.alpha
		self.pitch_angle_ = self.pitch_angle_*(1-self.alpha) + raw_pitch_angle_*self.alpha
		self.yaw_angle_ = self.yaw_angle_*(1-self.alpha_yaw) + raw_yaw_angle_*self.alpha_yaw
		self.gyro_x_ = self.gyro_x_*(1-self.alpha) + raw_gyro_x_*self.alpha
		self.gyro_y_ = self.gyro_y_*(1-self.alpha) + raw_gyro_y_*self.alpha
		self.gyro_z_ = self.gyro_z_*(1-self.alpha_yaw) + raw_gyro_z_*self.alpha_yaw
		self.altitude_ = self.altitude_*(1-self.alpha_xyz) + raw_altitude_*self.alpha_xyz
		self.velocity_z_ = self.velocity_z_*(1-self.alpha_xyz) + raw_velocity_z_*self.alpha_xyz
		self.acceleration_z_ = self.acceleration_z_*(1-self.alpha_xyz) + raw_acceleration_z_*self.alpha_xyz

		self.sin_roll_ = np.sin(self.roll_angle_)
		self.cos_roll_ = np.cos(self.roll_angle_)
		self.sin_pitch_ = np.sin(self.pitch_angle_)
		self.cos_pitch_ = np.cos(self.pitch_angle_)
		self.sin_yaw_ = np.sin(self.yaw_angle_)
		self.cos_yaw_ = np.cos(self.yaw_angle_)

		# derivatives
		self.acc_current_x_ = (self.vel_current_x_ - self.vel_current_x_old_)/self.dt_
		self.acc_current_y_ = (self.vel_current_y_ - self.vel_current_y_old_)/self.dt_
		

	def xy_control(self):
		############# implement element wise for easy porting to ARM(STM32)

		# filter position target with second order filter
		# generate velocity, acceleration, jerk target
		# x
		self.pos_target_x_filtered_old_2_ = self.pos_target_x_filtered_old_
		self.pos_target_x_filtered_old_ = self.pos_target_x_filtered_
		self.vel_target_x_filtered_old_ = self.vel_target_x_filtered_
		self.acc_target_x_filtered_old_ = self.acc_target_x_filtered_

		self.pos_target_x_filtered_ = self.a_1_xy_target*self.pos_target_x_filtered_old_ + self.a_2_xy_target*self.pos_target_x_filtered_old_2_ + self.b_1_xy_target*self.pos_target_x_;
		self.vel_target_x_filtered_ = (self.pos_target_x_filtered_ - self.pos_target_x_filtered_old_)/self.dt_
		self.acc_target_x_ = (self.vel_target_x_filtered_ - self.vel_target_x_filtered_old_)/self.dt_
		self.acc_target_x_filtered_ = self.acc_target_x_filtered_ * (1-self.alpha_xy_target) + self.acc_target_x_*self.alpha_xy_target
		self.jer_target_x_ = (self.acc_target_x_filtered_ - self.acc_target_x_filtered_old_)/self.dt_
		self.jer_target_x_filtered_ = self.jer_target_x_filtered_ * (1-self.alpha_xy_target) + self.jer_target_x_*self.alpha_xy_target
		# print('pos_target_x_filtered_ = %.6f, vel_target_x_filtered_ = %.6f, acc_target_x_filtered_ = %.6f' % (self.pos_target_x_filtered_, self.vel_target_x_filtered_, self.acc_target_x_filtered_), end="\n\r")


		# y
		self.pos_target_y_filtered_old_2_ = self.pos_target_y_filtered_old_
		self.pos_target_y_filtered_old_ = self.pos_target_y_filtered_
		self.vel_target_y_filtered_old_ = self.vel_target_y_filtered_
		self.acc_target_y_filtered_old_ = self.acc_target_y_filtered_

		self.pos_target_y_filtered_ = self.a_1_xy_target*self.pos_target_y_filtered_old_ + self.a_2_xy_target*self.pos_target_y_filtered_old_2_ + self.b_1_xy_target*self.pos_target_y_;
		self.vel_target_y_filtered_ = (self.pos_target_y_filtered_ - self.pos_target_y_filtered_old_)/self.dt_
		self.acc_target_y_ = (self.vel_target_y_filtered_ - self.vel_target_y_filtered_old_)/self.dt_
		self.acc_target_y_filtered_ = self.acc_target_y_filtered_ * (1-self.alpha_xy_target) + self.acc_target_y_*self.alpha_xy_target
		self.jer_target_y_ = (self.acc_target_y_filtered_ - self.acc_target_y_filtered_old_)/self.dt_
		self.jer_target_y_filtered_ = self.jer_target_y_filtered_ * (1-self.alpha_xy_target) + self.jer_target_y_*self.alpha_xy_target

		# z
		self.pos_target_z_filtered_old_2_ = self.pos_target_z_filtered_old_
		self.pos_target_z_filtered_old_ = self.pos_target_z_filtered_
		self.vel_target_z_filtered_old_ = self.vel_target_z_filtered_
		self.acc_target_z_filtered_old_ = self.acc_target_z_filtered_

		self.pos_target_z_filtered_ = self.a_1_xy_target*self.pos_target_z_filtered_old_ + self.a_2_xy_target*self.pos_target_z_filtered_old_2_ + self.b_1_xy_target*self.pos_target_z_;
		self.vel_target_z_filtered_ = (self.pos_target_z_filtered_ - self.pos_target_z_filtered_old_)/self.dt_
		self.acc_target_z_ = (self.vel_target_z_filtered_ - self.vel_target_z_filtered_old_)/self.dt_
		self.acc_target_z_filtered_ = self.acc_target_z_filtered_ * (1-self.alpha_xy_target) + self.acc_target_z_*self.alpha_xy_target
		self.jer_target_z_ = (self.acc_target_z_filtered_ - self.acc_target_z_filtered_old_)/self.dt_
		self.jer_target_z_filtered_ = self.jer_target_z_filtered_ * (1-self.alpha_xy_target) + self.jer_target_z_*self.alpha_xy_target

		# error
		e_x = self.pos_current_x_ - self.pos_target_x_filtered_
		e_x_dot = self.vel_current_x_ - self.vel_target_x_filtered_
		e_x_ddot = self.acc_current_x_ - self.acc_target_x_filtered_

		e_pitch = self.pitch_angle_ - 0

		e_y = self.pos_current_y_ - self.pos_target_y_filtered_
		e_y_dot = self.vel_current_y_ - self.vel_target_y_filtered_
		e_y_ddot = self.acc_current_y_ - self.acc_target_y_filtered_

		e_roll = self.roll_angle_ - 0

		# print('e_y = %.6f, e_y_dot = %.6f, e_y_ddot = %.6f' % (e_y, e_y_dot, e_y_ddot), end="\n\r")
		# print('e_x = %.6f, e_x_dot = %.6f, e_x_ddot = %.6f' % (e_x, e_x_dot, e_x_ddot), end="\n\r")


		e_z = self.altitude_ - self.pos_target_z_filtered_
		e_z_dot = self.velocity_z_ - self.vel_target_z_filtered_
		e_z_ddot = self.acceleration_z_ - self.acc_target_z_filtered_
		#print('e_z = %.6f, e_z_dot = %.6f, e_z_ddot = %.6f' % (e_z, e_z_dot, e_z_ddot), end="\n\r")

		e_psi = self.yaw_angle_ - self.ang_ef_target_z_

		x_tdot_eq = self.jer_target_x_filtered_ - self.lambda_1_pitch*e_x_ddot - self.lambda_2_pitch*e_x_dot  - self.lambda_3_pitch*e_x
		y_tdot_eq = self.jer_target_y_filtered_ - self.lambda_1_roll*e_y_ddot - self.lambda_2_roll*e_y_dot  - self.lambda_3_roll*e_y
		# print('y_tdot_eq = %.4f' % y_tdot_eq, end="\n\r")
		# print('x_tdot_eq = %.4f' % x_tdot_eq, end="\n\r")

		z_tdot_eq = self.jer_target_z_filtered_ - self.lambda_1*e_z_ddot - self.lambda_2*e_z_dot  - self.lambda_3*e_z
		# print('z_tdot_eq = %.4f' % z_tdot_eq, end="\n\r")

		# sliding surface (try use real mass first)
		F_z = self.K_Fz*(self.voltage_amplitude_ - self.V_s)		#0.2-0.1N => 20-10 gram
		#print('F_z = %.4f' % F_z, end="\n\r")
		# i
		R_11 = self.cos_yaw_*self.cos_pitch_
		R_21 = self.sin_yaw_*self.cos_pitch_
		R_31 = -self.sin_pitch_
		# print('i: R_11 = %.6f, R_21 = %.6f, R_31 = %.6f' % (R_11, R_21, R_31), end="\n\r")

		# j
		R_12 = self.cos_yaw_*self.sin_pitch_*self.cos_roll_ - self.sin_yaw_*self.cos_roll_
		R_22 = self.sin_yaw_*self.sin_pitch_*self.sin_roll_ + self.cos_yaw_*self.cos_roll_
		R_32 = self.cos_pitch_*self.sin_roll_
		# print('j: R_12 = %.6f, R_22 = %.6f, R_32 = %.6f' % (R_12, R_22, R_32), end="\n\r")

		# print('mass/F_z = %.4f' % (self.mass/F_z), end="\n\r")

		omega_x_eq = self.mass/F_z*(x_tdot_eq*R_12 + y_tdot_eq*R_22 + z_tdot_eq*R_32)			# ~ y_tdot_eq*mass/Fz (mass/Fz~=0.1)
		# print('omega_x_eq = %.4f' % omega_x_eq, end="\n\r")
		omega_y_eq = self.mass/F_z*(x_tdot_eq*R_11 + y_tdot_eq*R_21 + z_tdot_eq*R_31)
		# print('omega_y_eq = %.4f' % omega_y_eq, end="\n\r")
		omega_z_eq = self.rate_ef_target_z_ - self.lambda_psi*e_psi
		#print('omega_z_eq = %.4f' % omega_z_eq, end="\n\r")

		p_x = self.gyro_x_ + omega_x_eq# + 8*e_roll			# ~ y_tdot_eq*mass/Fz
		p_y = self.gyro_y_ - omega_y_eq# + 10*e_pitch
		p_z = self.gyro_z_ - omega_z_eq
		# print('gyro_x_ = %.4f' % self.gyro_x_, end="\n\r")
		# print('gyro_y_ = %.4f' % self.gyro_y_, end="\n\r")
		# print('p_x = %.4f' % p_x, end="\n\r")
		# print('p_y = %.4f' % p_y, end="\n\r")
		# print('p_z = %.4f' % p_z, end="\n\r")
		p = np.zeros([3,1])
		p[0,0] = p_x
		p[1,0] = p_y
		p[2,0] = p_z
		# print('p =', end="\n\r")
		# print(p, end="\n\r")

		omega_x_eq_dot = (omega_x_eq - self.omega_x_eq_old_)/self.dt_
		omega_y_eq_dot = (omega_y_eq - self.omega_y_eq_old_)/self.dt_
		omega_z_eq_dot = (omega_z_eq - self.omega_z_eq_old_)/self.dt_
		# print('omega_x_eq_dot = %.6f, omega_y_eq_dot = %.6f, omega_z_eq_dot = %.6f' % (omega_x_eq_dot, omega_y_eq_dot, omega_z_eq_dot), end="\n\r")


		self.omega_x_eq_old_ = omega_x_eq
		self.omega_y_eq_old_ = omega_y_eq
		self.omega_z_eq_old_ = omega_z_eq

		# regressor only update the middle 3x3
		self.phi_xy[3,0] = -(self.I_z - self.I_y)*self.gyro_y_*self.gyro_z_ + self.I_x*omega_x_eq_dot
		self.phi_xy[4,1] = -(self.I_z - self.I_x)*self.gyro_x_*self.gyro_z_ + self.I_y*(-omega_y_eq_dot)
		self.phi_xy[5,2] = -(self.I_y - self.I_x)*self.gyro_x_*self.gyro_y_ + self.I_z*(-omega_z_eq_dot)
		# print('phi_xy_x = %.6f, phi_xy_y = %.6f, phi_xy_z = %.6f' % (self.phi_xy[3,0], self.phi_xy[4,1], self.phi_xy[5,2]), end="\n\r")


		# control input
		self.phi_xy[3,0] = 0
		self.phi_xy[4,1] = 0
		self.phi_xy[5,2] = 0
		u_a_xy = -self.phi_xy.transpose().dot(self.theta_hat_xy)	#3x1
		# print('tau_x = %.4f' % self.theta_hat_xy[0,0], end="\n\r")
		# print('middle term = %.4f' % self.phi_xy[3,0], end="\n\r")
		# print('d_x = %.4f' % self.theta_hat_xy[6,0], end="\n\r")
		# print('u_a_x = %.4f' % u_a_xy[0,0], end="\n\r")
		# print('tau_y = %.4f' % self.theta_hat_xy[1,0], end="\n\r")
		# print('middle term = %.4f' % self.phi_xy[4,1], end="\n\r")
		# print('d_y = %.4f' % self.theta_hat_xy[7,0], end="\n\r")
		# print('u_a_y = %.4f' % u_a_xy[1,0], end="\n\r")
		# print('tau_z = %.4f' % self.theta_hat_xy[2,0], end="\n\r")
		# print('middle term = %.4f' % self.phi_xy[5,2], end="\n\r")
		# print('d_z = %.4f' % self.theta_hat_xy[8,0], end="\n\r")
		# print('u_a_z = %.4f' % u_a_xy[2,0], end="\n\r")
		# print('self.theta_hat_xy', end="\n\r")
		# print(self.theta_hat_xy, end="\n\r")
		#print('u_a_x = %.6f, u_a_y = %.6f, u_a_z = %.6f' % (u_a[0,0], u_a[1,0], u_a[2,0]), end="\n\r")
		u_s1_xy = -np.diag(self.k_s1_xy).dot(p)		# 3x3 * 3x1 = 3x1
		#print('u_s1_x = %.6f, u_s1_y = %.6f, u_s1_z = %.6f' % (u_s1[0,0], u_s1[1,0], u_s1[2,0]), end="\n\r")
		u_s2_xy = -np.linalg.inv(4*np.diag(np.squeeze(self.epsilon_xy))).dot(np.diag(np.squeeze(self.h_xy))).dot(np.diag(np.squeeze(p))).dot(self.h_xy)
		#print('u_s2_x = %.6f, u_s2_y = %.6f, u_s2_z = %.6f' % (u_s2[0,0], u_s2[1,0], u_s2[2,0]), end="\n\r")


		# parameter adaptation
		theta_hat_xy_dot = self.Gamma_xy.dot(self.phi_xy).dot(p)
		# print('theta_hat_xy_dot', end="\n\r")
		# print(theta_hat_xy_dot, end="\n\r")
		theta_hat_xy_dot = self.nl_projection(theta_hat_xy_dot, self.theta_hat_xy, self.theta_hat_xy_min, self.theta_hat_xy_max)

		self.theta_hat_xy = self.theta_hat_xy + theta_hat_xy_dot*self.dt_
		#print('m_hat = %.6f, 1_hat = %.6f, d_hat = %.6f' % (self.theta_hat_z[0,0], self.theta_hat_z[1,0], self.theta_hat_z[2,0]), end="\n\r")
		self.theta_hat_xy = np.clip(self.theta_hat_xy, self.theta_hat_xy_min, self.theta_hat_xy_max)
		# print('tau_x_hat = %.6f, tau_y_hat = %.6f, tau_z_hat = %.6f' % (self.theta_hat_xy[0,0], self.theta_hat_xy[1,0], self.theta_hat_xy[2,0]), end="\n\r")
		# print('dx_hat = %.6f, d_y_hat = %.6f, d_z_hat = %.6f' % (self.theta_hat_xy[6,0], self.theta_hat_xy[7,0], self.theta_hat_xy[8,0]), end="\n\r")

		# output voltage
		Kuuz_xy = u_a_xy + u_s1_xy + u_s2_xy
		Ku_xy = np.diag([self.K_Tx, self.K_Ty, self.K_Tz])
		voltage_out_xy = np.linalg.inv(Ku_xy).dot(Kuuz_xy)
		# print('Ku-1', end="\n\r")
		# print(np.linalg.inv(Ku), end="\n\r")
		# print('voltage_out =', end="\n\r")
		# print(voltage_out, end="\n\r")

		# print('u_a_x(V) = %.6f, u_a_y(V) = %.6f, u_a_z(V) = %.6f' % (u_a_xy[0,0]/self.K_Tx, u_a_xy[1,0]/self.K_Ty, u_a_xy[2,0]/self.K_Tz), end="\n\r")
		# print('u_s1_x(V) = %.6f, u_s1_y(V) = %.6f, u_s1_z(V) = %.6f' % (u_s1_xy[0,0]/self.K_Tx, u_s1_xy[1,0]/self.K_Ty, u_s1_xy[2,0]/self.K_Tz), end="\n\r")
		# print('u_s2_x(V) = %.6f, u_s2_y(V) = %.6f, u_s2_z(V) = %.6f' % (u_s2_xy[0,0]/self.K_Tx, u_s2_xy[1,0]/self.K_Ty, u_s2_xy[2,0]/self.K_Tz), end="\n\r")

		# print('In u_s1_x(V): -(k_s1_x*omega_x) %.6f - (k_s1_x*mass/F_z*(x_tdot_eq*R_12)) %.6f -  (k_s1_x*mass/F_z*(y_tdot_eq*R_22)) %.6f - (k_s1_x*mass/F_z*(z_tdot_eq*R_32)) %.6f' % (-self.k_s1_xy[0]*self.gyro_x_/self.K_Tx, -self.k_s1_xy[0]*self.mass/F_z*x_tdot_eq*R_12/self.K_Tx, -self.k_s1_xy[0]*self.mass/F_z*y_tdot_eq*R_22/self.K_Tx, -self.k_s1_xy[0]*self.mass/F_z*z_tdot_eq*R_32/self.K_Tx), end="\n\r")
		# print('In u_s1_y(V): -(k_s1_y*omega_y) %.6f + (k_s1_y*mass/F_z*(x_tdot_eq*R_11)) %.6f +  (k_s1_y*mass/F_z*(y_tdot_eq*R_21)) %.6f + (k_s1_y*mass/F_z*(z_tdot_eq*R_31)) %.6f' % (-self.k_s1_xy[1]*self.gyro_y_/self.K_Ty, self.k_s1_xy[1]*self.mass/F_z*x_tdot_eq*R_11/self.K_Ty, self.k_s1_xy[1]*self.mass/F_z*y_tdot_eq*R_21/self.K_Ty, self.k_s1_xy[1]*self.mass/F_z*z_tdot_eq*R_31/self.K_Ty), end="\n\r")


		self.differential_voltage_ = np.clip(voltage_out_xy[0,0],  -self.differential_voltage_max_, self.differential_voltage_max_)
		self.mean_voltage_ = np.clip(voltage_out_xy[1,0],  -self.mean_voltage_max_, self.mean_voltage_max_)
		self.split_cycle_ = np.clip(voltage_out_xy[2,0],  -self.split_cycle_max_, self.split_cycle_max_)
		# print('differential_voltage_ = %.6f, mean_voltage_ = %.6f, split_cycle_ = %.6f' % (self.differential_voltage_, self.mean_voltage_, self.split_cycle_), end="\n\r")	
		

	def z_control(self):
		# filtered position target with first order filter
		# generate velocity targetz
		self.pos_target_z_alt_filtered_old_ = self.pos_target_z_alt_filtered_
		self.pos_target_z_alt_filtered_ = self.pos_target_z_alt_filtered_*(1-self.alpha_z_target) + self.pos_target_z_*self.alpha_z_target
		self.vel_target_z_ = (self.pos_target_z_alt_filtered_ - self.pos_target_z_alt_filtered_old_)/self.dt_
		self.vel_target_z_alt_filtered_ = self.vel_target_z_alt_filtered_*(1-self.alpha_z_dot_target) + self.vel_target_z_*self.alpha_z_dot_target

		# errors
		e_z = self.altitude_ - self.pos_target_z_alt_filtered_
		#print('e_z = %.4f' % e_z, end="\n\r")

		e_z_dot = self.velocity_z_ - self.vel_target_z_alt_filtered_
		#print('e_z_dot = %.4f' % e_z_dot, end="\n\r")

		z_eq_dot = self.vel_target_z_alt_filtered_ - self.k_1_z*e_z
		#print('z_r_dot = %.4f' % self.vel_target_z_alt_filtered_, end="\n\r")

		#print('z_eq_dot = %.4f' % z_eq_dot, end="\n\r")

		z_eq_ddot = (z_eq_dot - self.z_eq_dot_old)/self.dt_
		#print('z_eq_ddot = %.4f' % z_eq_ddot, end="\n\r")
		self.z_eq_dot_old = z_eq_dot
		
		# sliding surface
		p_z = self.velocity_z_ - z_eq_dot
		#print('p_z = %.4f' % p_z, end="\n\r")
		# rotation matrix
		R_33 = self.cos_pitch_*self.cos_roll_

		# regressor (DARC)
		self.phi_z[0] = -9.81 - z_eq_ddot
		self.phi_z[1] = -R_33*self.K_Fz*self.V_s
		self.phi_z[2] = 1
		#print('phi_z=', end="\n\r")
		#print(self.phi_z, end="\n\r")

		# control input
		u_a = -np.matmul(self.phi_z.transpose(), self.theta_hat_z)
		u_s1 = -self.k_s1_z*p_z
		u_s2 = -1/(4*self.epsilon_z)*self.h_z**2*p_z
		#print('m_hat = %.6f, 1_hat = %.6f, d_hat = %.6f' % (self.theta_hat_z[0,0], self.theta_hat_z[1,0], self.theta_hat_z[2,0]), end="\n\r")

		# parameter adaptation
		theta_hat_z_dot = np.matmul(self.Gamma_z, self.phi_z)*p_z
		theta_hat_z_dot = self.nl_projection(theta_hat_z_dot, self.theta_hat_z, self.theta_hat_z_min, self.theta_hat_z_max)
		#print('m_hat_dot = %.6f, 1_hat_dot = %.6f, d_hat_dot = %.6f' % (theta_hat_z_dot[0,0], theta_hat_z_dot[1,0], theta_hat_z_dot[2,0]), end="\n\r")
		self.theta_hat_z = self.theta_hat_z + theta_hat_z_dot*self.dt_
		#print('m_hat = %.6f, 1_hat = %.6f, d_hat = %.6f' % (self.theta_hat_z[0,0], self.theta_hat_z[1,0], self.theta_hat_z[2,0]), end="\n\r")
		self.theta_hat_z = np.clip(self.theta_hat_z, self.theta_hat_z_min, self.theta_hat_z_max)
		#print('m_hat = %.6f, 1_hat = %.6f, d_hat = %.6f' % (self.theta_hat_z[0,0], self.theta_hat_z[1,0], self.theta_hat_z[2,0]), end="\n\r")

		# output voltage
		Kuuz = u_a + u_s1 + u_s2
		Ku = R_33*self.K_Fz
		voltage_out = Kuuz/Ku
		#print('u_a = %.4f, u_s1 = %.4f, u_s2 = %.4f' % (u_a/Ku, u_s1/Ku, u_s2/Ku), end="\n\r")

		self.voltage_amplitude_ = np.clip(voltage_out, 0, self.voltage_amplitude_max_)

	def nl_projection(self, theta_dot, theta_hat, theta_min, theta_max):
		condi = np.logical_or(np.logical_and(theta_hat>=theta_max,theta_dot>0),np.logical_and(theta_hat<=theta_min,theta_dot<0))
		theta_dot[condi] = 0
		return theta_dot

	def rotation_to_euler_angle(self,R):
		roll = np.arctan2(R[2,1],R[2,2])
		pitch = np.arcsin(-R[2, 0])
		yaw = np.arctan2(R[1,0],R[0,0])
		return roll, pitch, yaw

