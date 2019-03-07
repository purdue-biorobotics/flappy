/*************************  FWMAV Simulation  *************************
* Version 0.3.2
* Fan Fei		Jan 2018
* Flapping wing quasi-steady aerodynamics
***********************************************************************
*/

#include <math.h>

namespace FWMAV{
class Wing {
public:
	Wing(int wing_index,
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
				  double stroke_plane_offset);
	virtual ~Wing();

	void doNothing();

	void UpdateAeroForce();

	void UpdateStates(	double body_velocity_roll,
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
						double rotate_velocity);
	void UpdateVelocityCoeff();
	void UpdateAoA();
	void UpdateCN();
	void UpdateVelocitySquaredCoeff();
	void UpdateCenterOfPressure();
	//void UpdateMomentCoeff();

	// grabber functions
	double GetSpanCoP();
	double GetChordCoP();
	double GetNormalForce();
	double GetMoment();
	double GetM_aero();
	double GetM_rd();

	// math functions
	double C_N(double alpha);	//CN
	//double dC_N(double alpha);	//partial CN / partial alpha
	double d_cp(double alpha);	//Dickenson center of pressure Fourier approximation
	//double dd_cp(double alpha);	
	template <typename T> int sgn(T val);

	//tester function
	double GetStroke();
	double GetAoA();

protected:

	// wing type and velocity pointers
	int sign_;

	// double* body_velocity_roll_;
	// double* body_velocity_pitch_;
	// double* body_velocity_yaw_;
	// double* body_velocity_x_;
	// double* body_velocity_y_;
	// double* body_velocity_z_;
	// double* stroke_angle_;
	// double* stroke_velocity_;
	// double* rotate_angle_;
	// double* rotate_velocity_;

	// flapper geometry
	double R_w_;	// wing length
	double d_0_;	// half shoulder
	double d_s_;	// stroke plane offset

	// wing geometry constants
	double air_density_;
	double wing_length_;
	double mean_chord_;
	double r33_;
	double r22_;
	double r11_;
	double r00_;
	double z_cp2_;
	double z_cp1_;
	double z_cp0_;
	double z_rd_;

	// body velocities
	double u_;	// x
	double v_;	// y
	double w_;	// z
	double p_;	// roll
	double q_;	// pitch
	double r_;	// yaw

	// wing angle and velocities
	double Phi_;		// stroke plane angle
	double Phi_dot_;
	double psi_;		// stroke angle
	double psi_dot_;
	double phi_;		// deviation angle
	double phi_dot_;	
	double theta_;		// wing rotation angle
	double theta_dot_;

	// wing angle trig
	double S_Phi_;
	double C_Phi_;
	double S_psi_;
	double C_psi_;
	double S_phi_;
	double C_phi_;

	// velocity coefficients
	double U_o1_;
	double U_o0_;
	double U_i1_;
	double U_i0_;
	
	// AoA
	double delta_alpha_;
	double alpha_0_;
	double alpha_;

	// C_N
	double C_N_;

	// velocity squared coefficients
	double a_U2_;
	double a_U1_;
	double a_U0_;

	// F_N coefficients
	double K_N2_;
	double K_N1_;
	double K_N0_;
	double K_N_1_;

	// center of pressure
	double d_cp_;
	double r_cp_;

	// moment coefficients
	double K_aero2_;
	double K_aero1_;
	double K_aero0_;
	double K_aero_1_;
	double K_aero_2_;

	// total force and moments
	double span_wise_center_of_pressure_;
	double cord_wise_center_of_pressure_;
	double normal_force_;
	double aero_moment_;
	double rotational_damping_moment_;
	
};
}