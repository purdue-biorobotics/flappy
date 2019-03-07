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

class PIDController():
	def __init__(self,dt):
		self.dt_ = dt
		self.desired_accel_x_ = 0
		self.desired_accel_y_ = 0
		
		self.pos_target_x_ = 0
		self.pos_target_y_ = 0
		self.pos_target_z_ = 0.0

		self.ang_ef_target_z_ = 0

		# control here
		# xy postion to velocity
		self.p_pos_xy_Kp_ = 7	#1m error to 0.1m/s

		# z postion to velocity
		self.p_pos_z_Kp_ = 5
		# z velocity to acceleration
		# self.p_vel_z_Kp_ = 20 # need to test

		# z position PID controller
		# self.pid_pos_z_ = pid()
		# self.pid_pos_z_.old_error = 0
		# self.pid_pos_z_.integral = 0
		# self.pid_pos_z_.int_max = 5
		# self.pid_pos_z_.Kp = 10
		# self.pid_pos_z_.Ki = 5
		# self.pid_pos_z_.Kd = 8
		# self.pid_pos_z_.p = 0
		# self.pid_pos_z_.i= 0
		# self.pid_pos_z_.d = 0

		# z velocity PID controller
		self.pid_vel_z_ = pid()
		self.pid_vel_z_.old_error = 0
		self.pid_vel_z_.integral = 0
		self.pid_vel_z_.int_max = 5
		self.pid_vel_z_.Kp = 10#10
		self.pid_vel_z_.Ki = 4#4
		self.pid_vel_z_.Kd = 0.04
		self.pid_vel_z_.p = 0
		self.pid_vel_z_.i= 0
		self.pid_vel_z_.d = 0

		# z acceleration PID controller
		# self.pid_acc_z_ = pid()
		# self.pid_acc_z_.old_error = 0
		# self.pid_acc_z_.integral = 0
		# self.pid_acc_z_.int_max = 5
		# self.pid_acc_z_.Kp = 1
		# self.pid_acc_z_.Ki = 0
		# self.pid_acc_z_.Kd = 0.00001
		# self.pid_acc_z_.p = 0
		# self.pid_acc_z_.i= 0
		# self.pid_acc_z_.d = 0

		# x velocity PID controller (pitch)
		self.pid_vel_x_ = pid()
		self.pid_vel_x_.old_error = 0
		self.pid_vel_x_.integral = 0
		self.pid_vel_x_.int_max = 10
		self.pid_vel_x_.Kp = 2
		self.pid_vel_x_.Ki = 0.5
		self.pid_vel_x_.Kd = 0.01
		self.pid_vel_x_.p = 0
		self.pid_vel_x_.i= 0
		self.pid_vel_x_.d = 0
		self.acc_target_xy_max_ = 10

		# y velocity PID controller (roll)
		self.pid_vel_y_ = pid()
		self.pid_vel_y_.old_error = 0
		self.pid_vel_y_.integral = 0
		self.pid_vel_y_.int_max = 10
		self.pid_vel_y_.Kp = 1.75
		self.pid_vel_y_.Ki = 0.4
		self.pid_vel_y_.Kd = 0.002
		self.pid_vel_y_.p = 0
		self.pid_vel_y_.i= 0
		self.pid_vel_y_.d = 0

		# rpy angle to angular rate	// control how fast angle converges
		self.p_ang_roll_Kp_ = 10
		self.p_ang_pitch_Kp_ = 10
		self.p_ang_yaw_Kp_ = 5

		# roll angular rate PID controller
		self.pid_ang_roll_ = pid()
		self.pid_ang_roll_.old_error = 0
		self.pid_ang_roll_.integral = 0
		self.pid_ang_roll_.int_max = 2
		self.pid_ang_roll_.Kp = 3
		self.pid_ang_roll_.Ki = 1.5
		self.pid_ang_roll_.Kd = 0.3
		self.pid_ang_roll_.p = 0
		self.pid_ang_roll_.i= 0
		self.pid_ang_roll_.d = 0

		# pitch angular rate PID controller
		self.pid_ang_pitch_ = pid()
		self.pid_ang_pitch_.old_error = 0
		self.pid_ang_pitch_.integral = 0
		self.pid_ang_pitch_.int_max = 2
		self.pid_ang_pitch_.Kp = 5
		self.pid_ang_pitch_.Ki = 1.5
		self.pid_ang_pitch_.Kd = 0.4
		self.pid_ang_pitch_.p = 0
		self.pid_ang_pitch_.i= 0
		self.pid_ang_pitch_.d = 0

		# yaw angular rate PID controller
		self.pid_ang_yaw_ = pid()
		self.pid_ang_yaw_.old_error = 0
		self.pid_ang_yaw_.integral = 0
		self.pid_ang_yaw_.int_max = 1
		self.pid_ang_yaw_.Kp = 0.2
		self.pid_ang_yaw_.Ki = 0.33
		self.pid_ang_yaw_.Kd = 0.003
		self.pid_ang_yaw_.p = 0
		self.pid_ang_yaw_.i= 0
		self.pid_ang_yaw_.d = 0

		# control voltage limit
		self.differential_voltage_max_ = 3
		self.mean_voltage_max_ = 3.5
		self.split_cycle_max_ = 0.1
		self.hover_voltage_ = 9.3
		self.voltage_amplitude_max_ = 18

		self.frequency_ = 34
		self.voltage_amplitude_ = 12
		self.differential_voltage_ = 0
		self.mean_voltage_ = 0
		self.split_cycle_ = 0

		# working trim for flapper_sc_trim in mav_config_list.json
		# self.roll_trim_ = 0.4	#voltage
		# self.pitch_trim_ = -0.4	#voltage
		# self.yaw_trim_ = -0.04	#split cycle
		self.roll_trim_ = 0	#voltage
		self.pitch_trim_ = 0	#voltage
		self.yaw_trim_ = 0	#split cycle

		# filter
		RC = 1/(2*np.pi*20)
		self.alpha = dt/(RC+dt)
		RC = 1/(2*np.pi*20)
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
		self.add_trim()

		#print('differential_voltage_ = %.4f, mean_voltage_ = %.4f, split_cycle_ = %.4f' % (self.differential_voltage_, self.mean_voltage_, self.split_cycle_), end="\n\r")

		action_pid = np.zeros([4],dtype=np.float64)

		action_pid[0] = self.voltage_amplitude_
		action_pid[1] = self.differential_voltage_
		action_pid[2] = self.mean_voltage_
		action_pid[3] = self.split_cycle_

		return action_pid

	def controller_run(self):
		self.xy_control()
		self.attitude_control()
		self.z_control()

	def add_trim(self):
		self.differential_voltage_ = np.clip(self.differential_voltage_ + self.roll_trim_, -self.differential_voltage_max_, self.differential_voltage_max_)
		self.mean_voltage_ = np.clip(self.mean_voltage_ + self.pitch_trim_, -self.mean_voltage_max_, self.mean_voltage_max_)
		self.split_cycle_ = np.clip(self.split_cycle_ + self.yaw_trim_, -self.split_cycle_max_, self.split_cycle_max_)

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
		self.yaw_angle_ = self.yaw_angle_*(1-self.alpha) + raw_yaw_angle_*self.alpha
		self.gyro_x_ = self.gyro_x_*(1-self.alpha) + raw_gyro_x_*self.alpha
		self.gyro_y_ = self.gyro_y_*(1-self.alpha) + raw_gyro_y_*self.alpha
		self.gyro_z_ = self.gyro_z_*(1-self.alpha) + raw_gyro_z_*self.alpha
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
		# desired acceleration to velocity
		desired_vel_x = self.desired_accel_x_ * self.dt_
		desired_vel_y = self.desired_accel_y_ * self.dt_

		# update xy controller
		# desired velocity to position
		self.pos_target_x_ = self.pos_target_x_ + desired_vel_x * self.dt_
		self.pos_target_y_ = self.pos_target_y_ + desired_vel_y * self.dt_

		# position to rate
		pos_error_x = self.pos_target_x_ - self.pos_current_x_
		pos_error_y = self.pos_target_y_ - self.pos_current_y_
		vel_target_x = self.p_pos_xy_Kp_ * pos_error_x
		vel_target_y = self.p_pos_xy_Kp_ * pos_error_y

		# rate to acceleration
		vel_error_x = vel_target_x - self.vel_current_x_
		vel_error_y = vel_target_y - self.vel_current_y_

		self.pid_vel_x_ = self.update_pid(vel_error_x, self.pid_vel_x_)
		acc_target_x = np.clip(self.pid_vel_x_.p+self.pid_vel_x_.i+self.pid_vel_x_.d, -self.acc_target_xy_max_, self.acc_target_xy_max_)
		#print('x_p = %.4f, x_i = %.4f, x_d = %.4f' % (self.pid_vel_x_.p, self.pid_vel_x_.i, self.pid_vel_x_.d), end="\n\r")
		self.pid_vel_y_ = self.update_pid(vel_error_y, self.pid_vel_y_)
		acc_target_y = np.clip(self.pid_vel_y_.p+self.pid_vel_y_.i+self.pid_vel_y_.d, -self.acc_target_xy_max_, self.acc_target_xy_max_)
		#print('y_p = %.4f, y_i = %.4f, y_d = %.4f' % (self.pid_vel_y_.p, self.pid_vel_y_.i, self.pid_vel_y_.d), end="\n\r")

		# acceleration to lean angle
		acc_fwd = acc_target_x * self.cos_yaw_ + acc_target_y * self.sin_yaw_
		acc_lft = -acc_target_x * self.sin_yaw_ + acc_target_y * self.cos_yaw_

		self.pitch_target_ef_ = np.arctan(acc_fwd/9.81)
		self.roll_target_ef_ = -np.arctan(acc_lft*np.cos(self.pitch_target_ef_)/9.81)
		#print('roll target = %.4f, pitch target = %.4f' % (self.roll_target_ef_/np.pi*180, self.pitch_target_ef_/np.pi*180), end="\n\r")

	def attitude_control(self):
		# constraint roll pitch angle target
		# for debug
		#roll_target_ef_ = 0;
		#pitch_target_ef_ = 0;
		ang_ef_target_x = np.clip(self.roll_target_ef_, -45.0/180.0*np.pi, 45.0/180.0*np.pi)	#roll can be 45 deg
		ang_ef_target_y = np.clip(self.pitch_target_ef_, -30.0/180.0*np.pi, 30.0/180.0*np.pi)

		ang_ef_error_x = self.wrap_180(ang_ef_target_x - self.roll_angle_)
		ang_ef_error_y = self.wrap_180(ang_ef_target_y - self.pitch_angle_)
		ang_ef_error_z = self.wrap_180(self.ang_ef_target_z_ - self.yaw_angle_)
		ang_ef_error_z = np.clip(ang_ef_error_z, -10/180*np.pi, 10/180*np.pi)

		# convert to body frame

		ang_bf_error = self.frame_ef_to_bf(ang_ef_error_x, ang_ef_error_y, ang_ef_error_z)
		ang_bf_error_x = ang_bf_error[0]
		ang_bf_error_y = ang_bf_error[1]
		ang_bf_error_z = ang_bf_error[2]
		# ang_bf_error_z = 0.0f;
		#print('ang_ef_target_z_ = %.4f, yaw_angle_ = %.4f, ang_bf_error_z = %.4f' % (self.ang_ef_target_z_, self.yaw_angle_, ang_bf_error_z), end="\n\r")

		# roll angle to roll control
		self.pid_ang_roll_ = self.update_pid(ang_bf_error_x, self.pid_ang_roll_)
		self.differential_voltage_ = np.clip(self.pid_ang_roll_.p + self.pid_ang_roll_.i + self.pid_ang_roll_.d, -self.differential_voltage_max_, self.differential_voltage_max_)
		#print('roll_p = %.4f, roll_i = %.4f, roll_d = %.4f' % (self.pid_ang_roll_.p, self.pid_ang_roll_.i, self.pid_ang_roll_.d), end="\n\r")

		self.pid_ang_pitch_ = self.update_pid(ang_bf_error_y, self.pid_ang_pitch_)
		self.mean_voltage_ = np.clip(self.pid_ang_pitch_.p + self.pid_ang_pitch_.i + self.pid_ang_pitch_.d, -self.mean_voltage_max_, self.mean_voltage_max_)
		#print('pitch_p = %.4f, pitch_i = %.4f, pitch_d = %.4f' % (self.pid_ang_pitch_.p, self.pid_ang_pitch_.i, self.pid_ang_pitch_.d), end="\n\r")

		self.pid_ang_yaw_ = self.update_pid(ang_bf_error_z, self.pid_ang_yaw_)
		self.split_cycle_ = np.clip(self.pid_ang_yaw_.p + self.pid_ang_yaw_.i + self.pid_ang_yaw_.d, -self.split_cycle_max_, self.split_cycle_max_)
		#print('yaw_p = %.4f, yaw_i = %.4f, yaw_d = %.4f' % (self.pid_ang_yaw_.p, self.pid_ang_yaw_.i, self.pid_ang_yaw_.d), end="\n\r")

		# update rate body frame targets
		# rate_bf_target_x = self.p_ang_roll_Kp_ * ang_bf_error_x
		# rate_bf_target_y = self.p_ang_pitch_Kp_ * ang_bf_error_y
		# rate_bf_target_z = self.p_ang_yaw_Kp_ * ang_bf_error_z

		#include roll and pitch rate required to account for precession of the desired attitude about the body frame yaw axes
		#rate_bf_target_x = rate_bf_target_x + ang_bf_error_y* gyro_z_;
		#rate_bf_target_y = rate_bf_target_y - ang_bf_error_x* gyro_z_;

		# run rate controller
		# self.rate_bf_to_roll(rate_bf_target_x)
		# self.rate_bf_to_pitch(rate_bf_target_y)
		# self.rate_bf_to_yaw(rate_bf_target_z)

	def wrap_180(self, angle):
		if (angle > 3*np.pi or angle < -3*np.pi):
			angle = np.fmod(angle,2*np.pi)
		if (angle > np.pi):
			angle = angle - 2*np.pi
		if (angle < - np.pi):
			angle = angle + 2*np.pi
		return angle;

	def rate_bf_to_roll(self, rate_target):
		rate_error = rate_target - self.gyro_x_
		self.pid_rate_roll_ = self.update_pid(rate_error, self.pid_rate_roll_)
		self.differential_voltage_ = np.clip(self.pid_rate_roll_.p + self.pid_rate_roll_.i + self.pid_rate_roll_.d, -self.differential_voltage_max_, self.differential_voltage_max_)

	def rate_bf_to_pitch(self, rate_target):
		rate_error = rate_target - self.gyro_y_
		self.pid_rate_pitch_ = self.update_pid(rate_error, self.pid_rate_pitch_)
		self.mean_voltage_ = np.clip(self.pid_rate_pitch_.p + self.pid_rate_pitch_.i + self.pid_rate_pitch_.d, -self.mean_voltage_max_, self.mean_voltage_max_)

	def rate_bf_to_yaw(self, rate_target):
		rate_error = rate_target - self.gyro_z_
		self.pid_rate_yaw_ = self.update_pid(rate_error, self.pid_rate_yaw_)
		self.split_cycle_ = np.clip(self.pid_rate_yaw_.p + self.pid_rate_yaw_.i + self.pid_rate_yaw_.d, -self.split_cycle_max_, self.split_cycle_max_)
		
	def update_pid(self, error, pid_target, gyro = 0):
		integral = pid_target.integral + error*self.dt_
		
		if (integral > pid_target.int_max):
			integral = pid_target.int_max
		elif (integral < -pid_target.int_max):
			integral = -pid_target.int_max

		if gyro == 0:
			derivative = (error - pid_target.old_error)/self.dt_
		else:
			derivative = gyro
		pid_target.p = error*pid_target.Kp
		pid_target.i = integral*pid_target.Ki
		pid_target.d = derivative*pid_target.Kd
		
		# update all fields
		pid_target.old_error = error
		pid_target.integral = integral

		return pid_target

	def z_control(self):
		# position to rate
		pos_error_z = self.pos_target_z_ - self.altitude_
		#pos_error_z = np.clip(pos_error_z, -0.2, 0.2)
		# self.pid_pos_z_ = self.update_pid(pos_error_z, self.pid_pos_z_)
		# voltage_out = self.pid_pos_z_.p + self.pid_pos_z_.i + self.pid_pos_z_.d
		# print('z_p = %.4f, z_i = %.4f, z_d = %.4f' % (self.pid_pos_z_.p, self.pid_pos_z_.i, self.pid_pos_z_.d), end="\n\r")
		vel_target_z = pos_error_z * self.p_pos_z_Kp_

		# rate to acceleration
		vel_error_z = vel_target_z - self.velocity_z_

		self.pid_vel_z_ = self.update_pid(vel_error_z, self.pid_vel_z_)
		voltage_out = self.pid_vel_z_.p + self.pid_vel_z_.i + self.pid_vel_z_.d
		#print('z_p = %.4f, z_i = %.4f, z_d = %.4f' % (self.pid_vel_z_.p, self.pid_vel_z_.i, self.pid_vel_z_.d), end="\n\r")
		# acc_target_z = vel_error_z * self.p_vel_z_Kp_

		# # acceleration to throttle
		# acc_error_z = acc_target_z - self.acceleration_z_

		# self.pid_acc_z_ = self.update_pid(acc_error_z, self.pid_acc_z_)

		# voltage_out = self.pid_acc_z_.p + self.pid_acc_z_.i + self.pid_acc_z_.d
		voltage_out = voltage_out/(self.cos_pitch_*self.cos_roll_)

		self.voltage_amplitude_ = np.clip(voltage_out, 0, self.voltage_amplitude_max_ - self.hover_voltage_)
		self.voltage_amplitude_ += self.hover_voltage_
		#print('voltage_amplitude_ = %.4f' % self.voltage_amplitude_, end="\n\r")

	def frame_ef_to_bf(self, ef_x, ef_y, ef_z):
		bf = np.zeros([3],dtype=np.float64)

		bf[0] = ef_x - self.sin_pitch_*ef_z
		bf[1] = self.cos_roll_*ef_y + self.sin_roll_*self.cos_pitch_*ef_z
		bf[2] = -self.sin_roll_*ef_y + self.cos_pitch_*self.cos_roll_*ef_z

		return bf

	def rotation_to_euler_angle(self,R):
		roll = np.arctan2(R[2,1],R[2,2])
		pitch = np.arcsin(-R[2, 0])
		yaw = np.arctan2(R[1,0],R[0,0])
		return roll, pitch, yaw

