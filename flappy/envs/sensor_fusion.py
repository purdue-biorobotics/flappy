import numpy as np
import math

class SensorFusion():
	def __init__(self, dt_s, states):
		self.dt_s_ = dt_s

		k_I_x_ = 5300e-9
		k_I_y_ = 4900e-9
		k_I_z_ = 3700e-9
		self.I1_ = (k_I_y_-k_I_z_)/k_I_x_
		self.I2_ = (k_I_x_-k_I_z_)/k_I_y_
		self.I3_ = (k_I_x_-k_I_y_)/k_I_z_
		self.EKF_cache_size_ = 50

		# # in STM32 code the below exact filter is used
		# RC = 1/(2*np.pi*50)
		# self.alpha = 0.0005/(RC+0.0005)
		# # equivlent to
		# RC = 1/(2*np.pi*12.5)
		# self.alpha = dt_s/(RC+dt_s)

		RC = 1/(2*np.pi*5)
		self.alpha = dt_s/(RC+dt_s)

		RC = 1/(2*np.pi*136)
		self.alpha_gyro = dt_s/(RC+dt_s)

		self.reset(states)

	def reset(self, states):
		roll_angle_ = states['body_positions'][0,0]
		pitch_angle_ = states['body_positions'][1,0]
		yaw_angle_ = states['body_positions'][2,0]
		pos_x_ = states['body_positions'][3,0]
		pos_y_ = states['body_positions'][4,0]
		pos_z_ = states['body_positions'][5,0]


		self.x_pri_ = np.zeros([7,1],dtype=np.float64)
		self.x_po_ = np.zeros([7,1],dtype=np.float64)
		self.gyro_ = np.zeros([3,1],dtype=np.float64)

		[self.x_pri_[0][0], self.x_pri_[1][0], self.x_pri_[2][0], self.x_pri_[3][0]] = self.euler_angle_to_quaternion(roll_angle_, pitch_angle_, yaw_angle_)
		[self.x_po_[0][0], self.x_po_[1][0], self.x_po_[2][0], self.x_po_[3][0]] = self.euler_angle_to_quaternion(roll_angle_, pitch_angle_, yaw_angle_)


		self.x_po_cache_ = np.zeros([7,self.EKF_cache_size_],dtype=np.float64)
		self.gyro_cache_ = np.zeros([3,self.EKF_cache_size_],dtype=np.float64)
		self.ind_now = 0
		self.ind_prev = 0
		self.ind_prev_next = 0

		self.out_roll_ = roll_angle_
		self.out_pitch_ = pitch_angle_
		self.out_yaw_ = yaw_angle_
		self.out_yaw_old_ = yaw_angle_

		self.vicon_roll_ = roll_angle_
		self.vicon_pitch_ = pitch_angle_
		self.vicon_yaw_ = yaw_angle_
		self.vicon_x_ = pos_x_
		self.vicon_y_ = pos_y_
		self.vicon_z_ = pos_z_

		self.IMU_ax_ = 0
		self.IMU_ay_ = 0
		self.IMU_az_ = 0
		self.IMU_gx_ = 0
		self.IMU_gy_ = 0
		self.IMU_gz_ = 0
		self.IMU_mx_ = 0
		self.IMU_my_ = 0
		self.IMU_mz_ = 0

		self.out_x_dot_ = 0
		self.out_y_dot_ = 0
		self.out_z_dot_ = 0

		self.IMU_gx_dot_ = 0
		self.IMU_gy_dot_ = 0
		self.IMU_gz_dot_ = 0

		self.out_gx_ = 0
		self.out_gy_ = 0
		self.out_gz_ = 0

		self.out_x_dot_old_ = 0
		self.out_y_dot_old_ = 0
		self.out_z_dot_old_ = 0

		self.vicon_x_old_ = pos_x_
		self.vicon_y_old_ = pos_y_
		self.vicon_z_old_ = pos_z_

	def run(self, sensor, time):
		self.update_sensors(sensor)
		self.EKF2_run(time, sensor, 16)

		# self.out_roll_ = np.arctan2((2.0*(self.x_po_[0][0]*self.x_po_[1][0] + self.x_po_[2][0]*self.x_po_[3][0])), (1.0-2.0*(self.x_po_[1][0]*self.x_po_[1][0] + self.x_po_[2][0]*self.x_po_[2][0])))
		# self.out_pitch_ = np.arcsin(2.0*(self.x_po_[0][0]*self.x_po_[2][0] - self.x_po_[1][0]*self.x_po_[3][0]))
		# self.out_yaw_ = np.arctan2((2.0*(self.x_po_[1][0]*self.x_po_[2][0] + self.x_po_[0][0]*self.x_po_[3][0])), (1.0-2.0*(self.x_po_[2][0]*self.x_po_[2][0] + self.x_po_[3][0]*self.x_po_[3][0])))
		[self.out_roll_, self.out_pitch_, self.out_yaw_] = self.quaternion_to_euler_angle(self.x_po_[0][0], self.x_po_[1][0], self.x_po_[2][0], self.x_po_[3][0])

		self.out_x_dot_ = (self.vicon_x_ - self.vicon_x_old_)/self.dt_s_
		self.out_y_dot_ = (self.vicon_y_ - self.vicon_y_old_)/self.dt_s_
		self.out_z_dot_ = (self.vicon_z_ - self.vicon_z_old_)/self.dt_s_
		
		self.out_x_dot_ = self.out_x_dot_ * self.alpha + self.out_x_dot_old_*(1-self.alpha)
		self.out_y_dot_ = self.out_y_dot_ * self.alpha + self.out_y_dot_old_*(1-self.alpha)
		self.out_z_dot_ = self.out_z_dot_ * self.alpha + self.out_z_dot_old_*(1-self.alpha)

		self.out_gx_ = self.IMU_gx_ * self.alpha_gyro + self.out_gx_ * (1-self.alpha_gyro)
		self.out_gy_ = self.IMU_gy_ * self.alpha_gyro + self.out_gy_ * (1-self.alpha_gyro)
		self.out_gz_ = self.IMU_gz_ * self.alpha_gyro + self.out_gz_ * (1-self.alpha_gyro)

		self.out_x_dot_old_ = self.out_x_dot_
		self.out_y_dot_old_ = self.out_y_dot_
		self.out_z_dot_old_ = self.out_z_dot_

		self.vicon_x_old_ = self.vicon_x_
		self.vicon_y_old_ = self.vicon_y_
		self.vicon_z_old_ = self.vicon_z_

		# # wrap yaw
		if(np.fabs(self.out_yaw_-self.out_yaw_old_)>3.49):
			if(self.out_yaw_<self.out_yaw_old_):
				self.out_yaw_ = self.out_yaw_+ 6.28319
			else:
				self.out_yaw_ = self.out_yaw_-6.28319
			
		
		self.out_yaw_old_ = self.out_yaw_

	def quaternion_to_euler_angle(self,w, x, y, z):
	
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		X = math.atan2(t0, t1)
		
		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		Y = math.asin(t2)
		
		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		Z = math.atan2(t3, t4)
	
		return X, Y, Z
	def euler_angle_to_quaternion(self, roll, pitch, yaw):
		w = np.cos(roll/2.0)*np.cos(pitch/2.0)*np.cos(yaw/2.0) + np.sin(roll/2.0)*np.sin(pitch/2.0)*np.sin(yaw/2.0)
		x = np.sin(roll/2.0)*np.cos(pitch/2.0)*np.cos(yaw/2.0) - np.cos(roll/2.0)*np.sin(pitch/2.0)*np.sin(yaw/2.0)
		y = np.cos(roll/2.0)*np.sin(pitch/2.0)*np.cos(yaw/2.0) + np.sin(roll/2.0)*np.cos(pitch/2.0)*np.sin(yaw/2.0)
		z = np.cos(roll/2.0)*np.cos(pitch/2.0)*np.sin(yaw/2.0) - np.sin(roll/2.0)*np.sin(pitch/2.0)*np.cos(yaw/2.0)

		return w, x, y, z

	def update_sensors(self, sensor):
		self.vicon_roll_ = sensor.vicon_roll_
		self.vicon_pitch_ = sensor.vicon_pitch_
		self.vicon_yaw_ = sensor.vicon_yaw_
		self.vicon_x_ = sensor.vicon_x_
		self.vicon_y_ = sensor.vicon_y_
		self.vicon_z_ = sensor.vicon_z_

		self.IMU_ax_ = sensor.IMU_ax_
		self.IMU_ay_ = sensor.IMU_ay_
		self.IMU_az_ = sensor.IMU_az_
		self.IMU_gx_ = sensor.IMU_gx_
		self.IMU_gy_ = sensor.IMU_gy_
		self.IMU_gz_ = sensor.IMU_gz_
		self.IMU_mx_ = sensor.IMU_mx_
		self.IMU_my_ = sensor.IMU_my_
		self.IMU_mz_ = sensor.IMU_mz_

	def EKF2_run(self, time, sensor, vicon_delay_step_):
		if sensor.vicon_update_flag_ == 0:		#No vicon come in// only do priori estimation			
			# update priori estimates using measurement instead of states
			self.gyro_[0][0] = self.IMU_gx_;
			self.gyro_[1][0] = self.IMU_gy_;
			self.gyro_[2][0] = self.IMU_gz_;
			self.x_po_[4][0] = self.gyro_[0][0];
			self.x_po_[5][0] = self.gyro_[1][0];
			self.x_po_[6][0] = self.gyro_[2][0];
			
			self.update_prediction()
			# update posterior covariance
			# put priori estimate in posterior estimate for output
			self.x_po_ = self.x_pri_
		else:
			self.ind_prev = self.ind_now - vicon_delay_step_
			if self.ind_prev < 0:
				j = self.ind_prev+self.EKF_cache_size_
			else:
				j = self.ind_prev

			self.x_po_ = self.x_po_cache_[:,[j]]

			self.ind_prev_next = self.ind_prev + 1

			for j in range(self.ind_prev_next, self.ind_now+2):
				if j<0:
					i = j + self.EKF_cache_size_
				else:
					i = j

				if j == self.ind_prev_next:
					self.gyro_ = self.gyro_cache_[:,[i]]
					roll_ref_ = self.vicon_roll_
					pit_ref_ = self.vicon_pitch_
					yaw_ref_ = self.vicon_yaw_

					# s_phi_2 = np.sin(roll_ref_/2.0)
					# c_phi_2 = np.cos(roll_ref_/2.0)
					# s_theta_2 = np.sin(pit_ref_/2.0)
					# c_theta_2 = np.cos(pit_ref_/2.0)
					# s_psi_2 = np.sin(yaw_ref_/2.0)
					# c_psi_2 = np.cos(yaw_ref_/2.0)
					# self.x_po_[0][0] = c_phi_2*c_theta_2*c_psi_2 + s_phi_2*s_theta_2*s_psi_2
					# self.x_po_[1][0] = -c_phi_2*s_theta_2*s_psi_2 + c_theta_2*c_psi_2*s_phi_2
					# self.x_po_[2][0] = c_phi_2*c_psi_2*s_theta_2 + s_phi_2*c_theta_2*s_psi_2
					# self.x_po_[3][0] = c_phi_2*c_theta_2*s_psi_2 - s_phi_2*c_psi_2*s_theta_2

					# self.x_po_[0][0] = np.cos(roll_ref_/2.0)*np.cos(pit_ref_/2.0)*np.cos(yaw_ref_/2.0) + np.sin(roll_ref_/2.0)*np.sin(pit_ref_/2.0)*np.sin(yaw_ref_/2.0)
					# self.x_po_[1][0] = np.sin(roll_ref_/2.0)*np.cos(pit_ref_/2.0)*np.cos(yaw_ref_/2.0) - np.cos(roll_ref_/2.0)*np.sin(pit_ref_/2.0)*np.sin(yaw_ref_/2.0)
					# self.x_po_[2][0] = np.cos(roll_ref_/2.0)*np.sin(pit_ref_/2.0)*np.cos(yaw_ref_/2.0) + np.sin(roll_ref_/2.0)*np.cos(pit_ref_/2.0)*np.sin(yaw_ref_/2.0)
					# self.x_po_[3][0] = np.cos(roll_ref_/2.0)*np.cos(pit_ref_/2.0)*np.sin(yaw_ref_/2.0) - np.sin(roll_ref_/2.0)*np.sin(pit_ref_/2.0)*np.cos(yaw_ref_/2.0)
					[ self.x_po_[0][0], self.x_po_[1][0], self.x_po_[2][0], self.x_po_[3][0]] = self.euler_angle_to_quaternion(roll_ref_, pit_ref_, yaw_ref_)
					self.x_po_[4][0] = self.gyro_[0][0];
					self.x_po_[5][0] = self.gyro_[1][0];
					self.x_po_[6][0] = self.gyro_[2][0];

					self.update_prediction()
					self.x_po_ = self.x_pri_
				else:
					if j == self.ind_now+1:
						self.gyro_[0][0] = self.IMU_gx_
						self.gyro_[1][0] = self.IMU_gy_
						self.gyro_[2][0] = self.IMU_gz_
					else:
						self.gyro_ = self.gyro_cache_[:,[i]]

					self.x_po_[4][0] = self.gyro_[0][0]
					self.x_po_[5][0] = self.gyro_[1][0]
					self.x_po_[6][0] = self.gyro_[2][0]

					self.update_prediction()
					self.x_po_ = self.x_pri_

				if j!=self.ind_now+1:
					self.x_po_cache_[:,[i]] = self.x_po_

			sensor.vicon_update_flag_ = 0

		self.ind_now = self.ind_now+1
		if self.ind_now > self.EKF_cache_size_-1:
			self.ind_now = self.ind_now -  self.EKF_cache_size_

		self.x_po_cache_[:,[self.ind_now]] = self.x_po_
		self.gyro_cache_[:,[self.ind_now]] = self.gyro_


	def update_prediction(self):
		self.x_pri_[0][0] = self.x_po_[0][0] - 0.5*self.x_po_[1][0]*self.x_po_[4][0]*self.dt_s_ - 0.5*self.x_po_[2][0]*self.x_po_[5][0]*self.dt_s_ - 0.5*self.x_po_[3][0]*self.x_po_[6][0]*self.dt_s_;
		self.x_pri_[1][0] = self.x_po_[1][0] + 0.5*self.x_po_[0][0]*self.x_po_[4][0]*self.dt_s_ - 0.5*self.x_po_[3][0]*self.x_po_[5][0]*self.dt_s_ + 0.5*self.x_po_[2][0]*self.x_po_[6][0]*self.dt_s_;
		self.x_pri_[2][0] = self.x_po_[2][0] + 0.5*self.x_po_[3][0]*self.x_po_[4][0]*self.dt_s_ + 0.5*self.x_po_[0][0]*self.x_po_[5][0]*self.dt_s_ - 0.5*self.x_po_[1][0]*self.x_po_[6][0]*self.dt_s_;
		self.x_pri_[3][0] = self.x_po_[3][0] - 0.5*self.x_po_[2][0]*self.x_po_[4][0]*self.dt_s_ + 0.5*self.x_po_[1][0]*self.x_po_[5][0]*self.dt_s_ + 0.5*self.x_po_[0][0]*self.x_po_[6][0]*self.dt_s_;
		# angular velocity
		self.x_pri_[4][0] = self.x_po_[4][0] + (self.I1_*self.x_po_[5][0]*self.x_po_[6][0])*self.dt_s_
		self.x_pri_[5][0] = self.x_po_[5][0] + (self.I2_*self.x_po_[4][0]*self.x_po_[6][0])*self.dt_s_
		self.x_pri_[6][0] = self.x_po_[6][0] + (self.I3_*self.x_po_[4][0]*self.x_po_[5][0])*self.dt_s_
