import numpy as np

class Sensor_IMU_Vicon():
	def __init__(self, dt_imu, dt_vicon, states):
		self.dt_imu_ = dt_imu
		self.dt_vicon_ = dt_vicon

		vicon_delay_time_ = 0.030
		self.vicon_roll_noise_ = 0.005	# 0.00873 = 0.5 degree
		self.vicon_pitch_noise_ = 0.005
		self.vicon_yaw_noise_ = 0.005
		self.vicon_x_noise_ = 1e-4	# 0.5mm;
		self.vicon_y_noise_ = 1e-4
		self.vicon_z_noise_ = 1e-4
		self.IMU_ax_noise_ = 0.5	# 1m/s^2
		self.IMU_ay_noise_ = 0.5
		self.IMU_az_noise_ = 0.5

		self.IMU_gx_noise_ = 3/180*np.pi	# 0.00873 = 0.5 deg/s
		self.IMU_gy_noise_ = 3/180*np.pi
		self.IMU_gz_noise_ = 3/180*np.pi
		self.IMU_mx_noise_ = 50
		self.IMU_my_noise_ = 50
		self.IMU_mz_noise_ = 50

		self.delay_vicon_step_ = np.ceil(vicon_delay_time_/dt_vicon);

		self.dt_vicon_actual_ = dt_vicon
		self.dt_IMU_actual_ = dt_imu
		self.reset(states)


	def reset(self,states):
		self.IMU_gx_noise_mean_ =  np.random.normal(0,0.02) #0.03
		self.IMU_gy_noise_mean_ = np.random.normal(0,0.03) #-0.035	# -0.035 = -1.988deg/s this non zero mean is the drift effect
		self.IMU_gz_noise_mean_ = np.random.normal(0,0.015) #0.02
		self.last_vicon_update_time_ = 0
		self.next_vicon_update_time_ = 0
		self.last_IMU_update_time_ = 0
		self.next_IMU_update_time_ = 0
		self.vicon_roll_cache_ = np.zeros([self.delay_vicon_step_.astype(int)],dtype=np.float64) + states['body_positions'][0,0]
		self.vicon_pitch_cache_ = np.zeros([self.delay_vicon_step_.astype(int)],dtype=np.float64) + states['body_positions'][1,0]
		self.vicon_yaw_cache_ = np.zeros([self.delay_vicon_step_.astype(int)],dtype=np.float64) + states['body_positions'][2,0]
		self.vicon_x_cache_ = np.zeros([self.delay_vicon_step_.astype(int)],dtype=np.float64) + states['body_positions'][3,0]
		self.vicon_y_cache_ = np.zeros([self.delay_vicon_step_.astype(int)],dtype=np.float64) + states['body_positions'][4,0]
		self.vicon_z_cache_ = np.zeros([self.delay_vicon_step_.astype(int)],dtype=np.float64) + states['body_positions'][5,0]
		self.vicon_cache_pointer = 0
		self.vicon_update_flag_ = 0
		self.vicon_x_old_ = 0
		self.vicon_y_old_ = 0
		self.vicon_z_old_ = 0

	def update_raw_data(self, time, states):
		self.update_Vicon(time, states)
		self.update_IMU(time, states)

	def update_Vicon(self, time, states):
		if time >= self.next_vicon_update_time_:
			self.dt_vicon_actual_ = time - self.last_vicon_update_time_

			roll_reading = states['body_positions'][0,0] + np.random.normal(0,self.vicon_roll_noise_)
			pitch_reading = states['body_positions'][1,0] + np.random.normal(0,self.vicon_pitch_noise_)
			yaw_reading = states['body_positions'][2,0] + np.random.normal(0,self.vicon_yaw_noise_)
			x_reading = states['body_positions'][3,0] + np.random.normal(0,self.vicon_x_noise_)
			y_reading = states['body_positions'][4,0] + np.random.normal(0,self.vicon_y_noise_)
			z_reading = states['body_positions'][5,0] + np.random.normal(0,self.vicon_z_noise_)
			# roll_reading = states['body_positions'][0,0]
			# pitch_reading = states['body_positions'][1,0]
			# yaw_reading = states['body_positions'][2,0]
			# x_reading = states['body_positions'][3,0]
			# y_reading = states['body_positions'][4,0]
			# z_reading = states['body_positions'][5,0]

			# cache reading
			self.vicon_roll_cache_[self.vicon_cache_pointer] = roll_reading
			self.vicon_pitch_cache_[self.vicon_cache_pointer] = pitch_reading
			self.vicon_yaw_cache_[self.vicon_cache_pointer] = yaw_reading
			self.vicon_x_cache_[self.vicon_cache_pointer] = x_reading
			self.vicon_y_cache_[self.vicon_cache_pointer] = y_reading
			self.vicon_z_cache_[self.vicon_cache_pointer] = z_reading

			self.vicon_read_pointer = self.vicon_cache_pointer + 1
			if self.vicon_read_pointer >= self.delay_vicon_step_:
				self.vicon_read_pointer = 0

			# output delayed reading
			self.vicon_roll_ = self.vicon_roll_cache_[self.vicon_read_pointer]
			self.vicon_pitch_ = self.vicon_pitch_cache_[self.vicon_read_pointer]
			self.vicon_yaw_ = self.vicon_yaw_cache_[self.vicon_read_pointer]
			self.vicon_x_ = self.vicon_x_cache_[self.vicon_read_pointer]
			self.vicon_y_ = self.vicon_y_cache_[self.vicon_read_pointer]
			self.vicon_z_ = self.vicon_z_cache_[self.vicon_read_pointer]

			self.vicon_x_dot_ = (self.vicon_x_ - self.vicon_x_old_)/self.dt_vicon_
			self.vicon_y_dot_ = (self.vicon_y_ - self.vicon_y_old_)/self.dt_vicon_
			self.vicon_z_dot_ = (self.vicon_z_ - self.vicon_z_old_)/self.dt_vicon_

			self.vicon_x_old_ = self.vicon_x_
			self.vicon_y_old_ = self.vicon_y_
			self.vicon_z_old_ = self.vicon_z_

			self.vicon_cache_pointer = self.vicon_cache_pointer + 1
			if self.vicon_cache_pointer >= self.delay_vicon_step_:
				self.vicon_cache_pointer = 0

			self.last_vicon_update_time_ = time
			self.next_vicon_update_time_ = self.next_vicon_update_time_ + self.dt_vicon_

			if(time<self.dt_vicon_):
				self.vicon_update_flag_ = 0
			else:
				self.vicon_update_flag_ = 1
			#print('Vicon updated!', end="\n\r")

	def update_IMU(self, time, states):
		if time >= self.next_IMU_update_time_:
			self.dt_IMU_actual_ = time - self.last_IMU_update_time_

			# self.IMU_ax_ = states['body_accelerations'][3,0]
			# self.IMU_ay_ = states['body_accelerations'][4,0]
			# self.IMU_az_ = states['body_accelerations'][5,0]
			self.IMU_ax_ = states['body_accelerations'][3,0] + np.random.normal(0,self.IMU_ax_noise_)
			self.IMU_ay_ = states['body_accelerations'][4,0] + np.random.normal(0,self.IMU_ay_noise_)
			self.IMU_az_ = states['body_accelerations'][5,0] + np.random.normal(0,self.IMU_az_noise_)
			# self.IMU_gx_ = states['body_velocities'][0,0]
			# self.IMU_gy_ = states['body_velocities'][1,0]
			# self.IMU_gz_ = states['body_velocities'][2,0]
			self.IMU_gx_ = states['body_velocities'][0,0] + np.random.normal(self.IMU_gx_noise_mean_,self.IMU_gx_noise_)
			self.IMU_gy_ = states['body_velocities'][1,0] + np.random.normal(self.IMU_gy_noise_mean_,self.IMU_gy_noise_)
			self.IMU_gz_ = states['body_velocities'][2,0] + np.random.normal(self.IMU_gz_noise_mean_,self.IMU_gz_noise_)
			self.IMU_mx_ = np.random.normal(0,self.IMU_mx_noise_)
			self.IMU_my_ = np.random.normal(0,self.IMU_my_noise_)
			self.IMU_mz_ = np.random.normal(0,self.IMU_mz_noise_)

			self.last_IMU_update_time_ = time
			self.next_IMU_update_time_ = self.next_IMU_update_time_ + self.dt_imu_
			#print('IMU updated!', end="\n\r")



