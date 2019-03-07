/*************************  FWMAV Simulation  *************************
* Version 0.3.2
* Fan Fei		Jan 2018
* Flapping wing quasi-steady aerodynamics
***********************************************************************
*/

#include "Wing.hpp"

FWMAV::Wing::Wing(int wing_index,
				  double wing_length,
				  double mean_chord,
				  double r33,
				  double r22,
				  double r11,
				  double r00,
				  double z_cp2,
				  double z_cp1,
				  double z_cp0,
				  double z_rd,
				  double shoulder_width,
				  double stroke_plane_offset)
{
	air_density_ = 1.18009482370369;
	wing_length_ = wing_length;
	mean_chord_ =mean_chord;
	r33_ = r33;
	r22_ = r22;
	r11_ = r11;
	r00_ = r00;
	z_cp2_ = z_cp2;
	z_cp1_ = z_cp1;
	z_cp0_ = z_cp0;
	z_rd_ = z_rd;
	sign_ = pow(-1,wing_index);

	// body_velocity_roll_ = body_velocity_roll;
	// body_velocity_pitch_ = body_velocity_pitch;
	// body_velocity_yaw_ = body_velocity_yaw;
	// body_velocity_x_ = body_velocity_x;
	// body_velocity_y_ = body_velocity_y;
	// body_velocity_z_ = body_velocity_z;
	// stroke_angle_ = stroke_angle;
	// stroke_velocity_ = stroke_velocity;
	// rotate_angle_ = rotate_angle;
	// rotate_velocity_ = rotate_velocity;

	R_w_ = wing_length_;
	d_0_ = shoulder_width;
	d_s_ = stroke_plane_offset;
	r_cp_ = R_w_*r33_/r22_;
	doNothing();
}


FWMAV::Wing::~Wing()
{
}

void FWMAV::Wing::doNothing()
{
	span_wise_center_of_pressure_ = 0;
	cord_wise_center_of_pressure_ = 0;
	normal_force_ = 0;
	aero_moment_ = 0;
	rotational_damping_moment_ = 0;
}

void FWMAV::Wing::UpdateAeroForce()
{
	// UpdateStates();

	UpdateVelocityCoeff();
	UpdateAoA();
	UpdateCN();
	UpdateCenterOfPressure();
	UpdateVelocitySquaredCoeff();

	normal_force_ = 0.5*air_density_*mean_chord_*C_N_*(a_U2_*pow(R_w_,3)*r22_ + a_U1_*pow(R_w_,2)*r11_ + a_U0_*R_w_*r00_);
	
	aero_moment_ = -0.5*air_density_*pow(mean_chord_,2)*C_N_*d_cp_*(a_U2_*pow(R_w_,3)*z_cp2_ + a_U1_*pow(R_w_,2)*z_cp1_ + a_U0_*R_w_*z_cp0_);
	if (normal_force_!=0)
		cord_wise_center_of_pressure_ = -aero_moment_/normal_force_;
	else
		cord_wise_center_of_pressure_ = 0;
	
	span_wise_center_of_pressure_ = r_cp_;
	
	rotational_damping_moment_ = -0.125*air_density_*fabs(theta_dot_)*theta_dot_*5.0*R_w_*pow(mean_chord_,4)*z_rd_;
	/*
	if (sign_ == 1)
		std::cout << "Left F_N=" << normal_force_ << " Left M_aero=" << aero_moment_ << " Left M_rd=" << rotational_damping_moment_ << " Left d_cp=" << d_cp_ << std::endl;
	else
		std::cout << "Right F_N=" << normal_force_ << " Right M_aero=" << aero_moment_ << " Right M_rd=" << rotational_damping_moment_ << " Right d_cp=" << d_cp_ << std::endl;
	*/
}

void FWMAV::Wing::UpdateStates(	double body_velocity_roll,
								double body_velocity_pitch,
								double body_velocity_yaw,
								double body_velocity_x,
								double body_velocity_y,
								double body_velocity_z,
								double stroke_plane_angle,
								double stroke_plane_velocity,
								double stroke_angle,
								double stroke_velocity,
								double deviation_angle,
								double deviation_velocity,
								double rotate_angle,
								double rotate_velocity)
{
	// get current body states
	
	// body velocities
	u_ = body_velocity_x;
	v_ = body_velocity_y;
	w_ = body_velocity_z;
	p_ = body_velocity_roll;
	q_ = body_velocity_pitch;
	r_ = body_velocity_yaw;

	// stroke plance
	Phi_ = stroke_plane_angle;
	Phi_dot_ = stroke_plane_velocity;

	// stroke
	psi_ = stroke_angle;
	psi_dot_ = stroke_velocity;

	// deviation
	phi_ = deviation_angle;
	phi_dot_ = deviation_velocity;

	// rotation	
	theta_ = rotate_angle;
	theta_dot_ = rotate_velocity;
	
	// trig pre calculation
	S_Phi_ = sin(Phi_);
	C_Phi_ = cos(Phi_);
	S_psi_ = sin(psi_);
	C_psi_ = cos(psi_);
	S_phi_ = sin(phi_);
	C_phi_ = cos(phi_);

	/*
	if (sign_ == 1)
		std::cout << "u =" << u_ << " v =" << v_ << " w =" << w_ << " p =" << p_ << " q =" << q_ << " r =" << r_ << std::endl;
	if (sign_ == 1)
		std::cout << "Left psi =" << psi_*180/M_PI << " Left theta =" << theta_*180/M_PI << " Left psi_dot =" << psi_dot_*180/M_PI << " Left theta_dot =" << theta_dot_*180/M_PI << std::endl;
	else
		std::cout << "Right psi =" << psi_*180/M_PI << " Right theta =" << theta_*180/M_PI << " Right psi_dot =" << psi_dot_*180/M_PI << " Right theta_dot =" << theta_dot_*180/M_PI << std::endl;
	*/
}

void FWMAV::Wing::UpdateVelocityCoeff()
{
	//std::cout << "psi_dot_ =" << psi_dot_ << std::endl;
	
	U_o1_ = sign_*(p_*C_psi_*C_Phi_ - Phi_dot_*S_psi_)
			+ (q_*S_psi_ + r_*C_psi_*S_Phi_ + phi_dot_);
	U_o0_ = sign_*(-(u_ + q_*d_s_)*C_phi_*S_Phi_ - r_*d_0_*S_phi_*S_psi_*C_Phi_ - (v_ - p_*d_s_)*S_phi_*C_psi_ + w_*S_phi_*S_psi_*S_Phi_ + p_*d_0_*C_phi_*C_Phi_) 
			+ ((u_ + q_*d_s_)*S_phi_*S_psi_*C_Phi_ + r_*d_0_*C_phi_*S_Phi_ + w_*C_phi_*C_Phi_ + p_*d_0_*S_phi_*S_psi_*S_Phi_);
	U_i1_ = sign_*(p_*S_phi_*S_psi_*C_Phi_ + r_*C_phi_*C_Phi_ + Phi_dot_*S_phi_*C_psi_)
			+ (-p_*C_phi_*S_Phi_ - q_*S_phi_*C_psi_ + r_*S_phi_*S_psi_*S_Phi_ + psi_dot_*C_phi_);
	U_i0_ = sign_*(r_*d_0_*C_psi_*C_Phi_ - (v_ - p_*d_s_)*S_psi_ - w_*C_psi_*S_Phi_)
			+(-(u_ + q_*d_s_)*C_psi_*C_Phi_ - p_*d_0_*C_psi_*S_Phi_);

	/* old model
	U_o1_ = sign_*p_*cos(psi_) + q_*sin(psi_);
	U_o0_ = sign_*p_*d_0_ + w_;
	U_i1_ = psi_dot_ + sign_*r_;
	U_i0_ = sign_*d_0_*r_*cos(psi_) + (-u_*cos(psi_)-sign_*v_*sin(psi_)) + (-q_*d_s_);
	*/
	
	/*
	if (sign_ == 1)
		std::cout << "Left U_o1 =" << U_o1_ << " Left U_o0 =" << U_o0_ << " Left U_i1 =" << U_i1_ << " Left U_i0 =" << U_i0_ << std::endl;
	else
		std::cout << "Right U_o1 =" << U_o1_ << " Right U_o0 =" << U_o0_ << " Right U_i1 =" << U_i1_ << " Right U_i0 =" << U_i0_ << std::endl;
	*/
}

/*
void FWMAV::Wing::UpdateAoACoeff()
{
	delta_alpha_prime_ = U_o1_/U_i1_ + U_o0_/U_i0_/R_w_;
	scaler_ = atan(delta_alpha_prime_)/delta_alpha_prime_;
	delta_alpha_0_ = U_o1_/U_i1_*scaler_;
	delta_alpha__1_ = U_o0_/U_i1_*scaler_;
	delta_alpha__1_ = 0;
	//approximation
	double U_i_ = U_i1_*R_w_ + U_i0_;
	alpha_0_ = theta_ + double(sgn(U_i_))*M_PI/2;
	//std::cout << "sgn(U_i_) =" << sgn(U_i_) << " sgn(psi_dot_)=" << sgn(psi_dot_) << std::endl;
	std::cout << "alpha_0_=" << alpha_0_ << " delta_alpha_0_=" << delta_alpha_0_ << " delta_alpha__1_=" << delta_alpha__1_ << std::endl;
}
*/

void FWMAV::Wing::UpdateAoA()
{
	// AoA correction
	double U_i_ = U_i1_*r_cp_ + U_i0_;
	if (U_i_ != 0)
		delta_alpha_ = atan((U_o1_*r_cp_ + U_o0_)/(U_i1_*r_cp_ + U_i0_));
	else
		delta_alpha_ = 0;

	// geometric AoA
	alpha_0_ = theta_ + double(sgn(U_i_))*M_PI/2;

	alpha_ = alpha_0_ - delta_alpha_;
	/*
	if (sign_ == 1)
		std::cout << "Left delta_alpha=" << delta_alpha_ << " Left U_i=" << U_i_ << " Left alpha_0=" << alpha_0_ << std::endl;
	else
		std::cout << "Right delta_alpha=" << delta_alpha_ << " Right U_i=" << U_i_ << " Right alpha_0=" << alpha_0_ << std::endl;
	*/
}



void FWMAV::Wing::UpdateVelocitySquaredCoeff()
{
	a_U2_ = U_i1_*U_i1_ + U_o1_*U_o1_;
	a_U1_ = 2*U_i1_*U_i0_ + 2*U_o1_*U_o0_;
	a_U0_ = U_i0_*U_i0_ + U_o0_*U_o0_;
	/*
	if (sign_ == 1)
		std::cout << "Left a_U2=" << a_U2_ << " Left a_U1=" << a_U1_ << " Left a_U0=" << a_U0_ << std::endl;
	else
		std::cout << "Right a_U2=" << a_U2_ << " Right a_U1=" << a_U1_ << " Right a_U0=" << a_U0_ << std::endl;
	*/
}
/*
void FWMAV::Wing::UpdateForceCoeff()
{
	K_N2_ = a_U2_*C_N0_;
	K_N1_ = a_U1_*C_N0_ + a_U2_*C_N_1_;
	K_N0_ = a_U0_*C_N0_ + a_U1_*C_N_1_;
	K_N_1_ = a_U0_*C_N_1_;
	std::cout << "K_N2_ =" << K_N2_ << " K_N1_ =" << K_N1_ << " K_N0_ =" << K_N0_ << " K_N_1_ =" << K_N_1_ << std::endl;

}
*/

void FWMAV::Wing::UpdateCN()
{
	C_N_ = C_N(alpha_);
	//std::cout << "C_N0_=" << C_N0_ << " C_N_1_=" << C_N_1_ << std::endl;
}

void FWMAV::Wing::UpdateCenterOfPressure()
{
	d_cp_ = d_cp(alpha_);
}

/*
void FWMAV::Wing::UpdateMomentCoeff()
{
	K_aero2_ = K_N2_*d_cp0_;
	K_aero1_ = K_N2_*d_cp_1_ + K_N1_*d_cp0_;
	K_aero0_ = K_N1_*d_cp_1_ + K_N0_*d_cp0_;
	K_aero_1_ = K_N0_*d_cp_1_ + K_N_1_*d_cp0_;
	K_aero_2_ = K_N_1_*d_cp_1_;
}
*/

// math functions

template <typename T> int FWMAV::Wing::sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

double FWMAV::Wing::C_N(double alpha)
{
	return 1.8*sin(2*alpha)*cos(alpha) + 1.95*sin(alpha) - 1.5*cos(2*alpha)*sin(alpha);
}

/*
double FWMAV::Wing::dC_N(double alpha)
{
	return cos(alpha)*(0.3*pow(sin(alpha),2) + 2.1*pow(cos(alpha),2) + 1.95);
}
*/

double FWMAV::Wing::d_cp(double alpha)
{
	return 0.46 - 0.332*cos(alpha) - 0.037*cos(3*alpha) - 0.013*cos(5*alpha);
}

/*
double FWMAV::Wing::dd_cp(double alpha)
{
	return 0.332*sin(alpha) + 0.111*sin(3*alpha) + 0.065*sin(5*alpha);
}
*/

// grabber functions
double FWMAV::Wing::GetSpanCoP()
{
	return span_wise_center_of_pressure_;
}

double FWMAV::Wing::GetChordCoP()
{
	return cord_wise_center_of_pressure_;
}

double FWMAV::Wing::GetNormalForce()
{
	return normal_force_;
}

double FWMAV::Wing::GetMoment()
{
	double total_moment = aero_moment_+rotational_damping_moment_;
	return rotational_damping_moment_;
}

double FWMAV::Wing::GetM_aero()
{
	return aero_moment_;
}

double FWMAV::Wing::GetM_rd()
{
	return rotational_damping_moment_;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// tester functions
double FWMAV::Wing::GetStroke()
{
	// return *stroke_angle_;
	return 0;
}

double FWMAV::Wing::GetAoA()
{
	return alpha_;
}