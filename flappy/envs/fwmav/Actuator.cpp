/*************************  FWMAV Simulation  *************************
* Version 0.3
* Fan Fei		Jan 2018
* FWMAV simulation with dual motor driven robotic flapper
* PID controller using split cycle mechanism
***********************************************************************
*/

#include "Actuator.hpp"

FWMAV::Actuator::Actuator(double variable_resistance, double* stroke_velocity, double* stroke_acceleration):
		k_resistance_(12.4),
		k_torque_constant_(1.75E-3),
		k_gear_ratio_(10),
		k_mechanical_efficiency_(0.85),
		k_friction_coefficient_(2E-5),
		k_damping_coefficient_(9.74E-9),
		k_inertia_(7.03e-10)
{
	variable_resistance_ = variable_resistance;
	stroke_velocity_ = stroke_velocity;
	stroke_acceleration_ = stroke_acceleration;
	doNothing();
}


FWMAV::Actuator::~Actuator()
{

}

void FWMAV::Actuator::doNothing()
{
	output_torque_ = 0;
}

void FWMAV::Actuator::updateDriverVoltage(double voltage)
{
	voltage_ = voltage;
}

void FWMAV::Actuator::UpdateTorque()
{
	psi_dot_ = *stroke_velocity_;
	psi_ddot_ = *stroke_acceleration_;
	motor_vel_ = psi_dot_*k_gear_ratio_;
	motor_accel_ = psi_ddot_*k_gear_ratio_;
	if (psi_dot_>0)
		sign_ = 1;
	else if (psi_dot_<0)
		sign_ = -1;
	else
		sign_ = 0;
	
	if (variable_resistance_ == 0)
		resistance_ = k_resistance_;
	else
		resistance_ = variable_resistance_;

	back_EMF_ = k_torque_constant_*motor_vel_;
	current_ = (voltage_-back_EMF_)/resistance_;

	inertia_torque_ = k_inertia_*motor_accel_;
	damping_torque_ = k_damping_coefficient_*motor_vel_;
	friction_torque_ = k_friction_coefficient_*sign_;
	magnetic_torque_ = k_torque_constant_*current_;

	motor_torque_ = magnetic_torque_-inertia_torque_-damping_torque_-friction_torque_;

	output_torque_ = motor_torque_*k_gear_ratio_*k_mechanical_efficiency_;
	//std::cout << "inertia_torque_=" << inertia_torque_ << " damping_torque_=" << damping_torque_ << " friction_torque_=" << friction_torque_ << " magnetic_torque_=" << magnetic_torque_ << " motor_torque_=" << motor_torque_ << " output_torque_=" << output_torque_ << std::endl;
}

double FWMAV::Actuator::GetTorque()
{
	return output_torque_;
}

// for debugging

double FWMAV::Actuator::GetMotorTorque()
{
	return motor_torque_;
}

double FWMAV::Actuator::GetMagTorque()
{
	return magnetic_torque_;
}

double FWMAV::Actuator::GetInerTorque()
{
	return inertia_torque_;
}

double FWMAV::Actuator::GetDampTorque()
{
	return damping_torque_;
}

double FWMAV::Actuator::GetFricTorque()
{
	return friction_torque_;
}

double FWMAV::Actuator::GetBEMF()
{
	return back_EMF_;
}

double FWMAV::Actuator::GetCurrent()
{
	return current_;
}

/*
I_a = (V-K_a psi_dot)/(R_a)

T_motor = T_load/(eta*N_g)
*/
