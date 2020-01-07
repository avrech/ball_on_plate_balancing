/*
  Ball_On_Plate.h - Library for Ball_On_Plate Project.
  Created by Avrech Ben-David 29.3.2017
  Released into the public domain.
*/

#ifndef Ball_On_Plate_h
#define Ball_On_Plate_h

// the #include statment and code go here...
#include "Arduino.h"
#include "Servo.h"
// alias:
#define sqr(x) ((x)*(x))
//constants


#define ON 1
#define OFF 0
#define MOTORS_NUM 6
#define ARM_LEN 15 // motor's arm. [mm]
#define ROD_LEN 162 // motor's rod [mm]
// Translation vector = [0;0;H0].
#define MAX_CTRL 15 // [degree]
#define ANG_FIR_LEN 7
class Touchpad
{
  public:
    Touchpad(int pins[4], float offset[2], float scaling_factor[2], int d_th, int max_sampling);
    void read_ball_position(float position[2]);
    void init(void); // read first position of the ball
  private:
    int _pins[4];
	int _x_offset, _y_offset;
	float _x_scaling, _y_scaling;
	float _prev_sample[2] = {0};
	int _d_th, _max_sampling;
	//int _d[2]; // samples derivative
	//int _dd[2]; // second derivative.
};

class StewartPlatform
{
  public:
    StewartPlatform(	const int motor_pins[6],
							const float motor_ang_offset[6],
							const float b[6][2], 
							const float p[6][2], 
							const float T[3], 
							const float beta[6]);
							
    void set_motors_angles(float motors_angles[6]);
	void init_motors(void);
	void get_motors_angles(float roll, float pitch, float alpha_vec[6]);
	void set_search_err_th(float th){_search_err_th = th;}
	void set_search_max_iter(float max_iter){_search_max_iter = max_iter;}
  private:
	int _motor_pins[6];
	int _motor_ang_offset[6];
	float _b[6][2]; // motors spindles
    float _p[6][2]; // rod plate joints
	float _cos_beta[6]; 
	float _sin_beta[6]; 
	float _T[3]; // translation vector
	Servo _myservo[6];
	const float RAD2DEG = 180/PI;
	const float DEG2RAD = PI/180;
	float _search_err_th = 0.05;
	float _search_max_iter = 10;
	
};

class Pid
{
	public:
		Pid(float T);
		void set_pid_coef(float Kp,float Ki, float Kd);
		void set_pid_soft_coef(float softKp,float softKi, float softKd);
		void set_lead_coef(float a, float b, float c){_A = a; _B = b; _C = c;}
		void set_ud_filter_coef(float tau){_tau = tau;}
		void set_th(float radius_th, float velocity_th){_radius_th = radius_th; _velocity_th = velocity_th;}
		void record_e(int en){_record_e = en;}
		void record_output(int en){_record_output = en;}
		void enable_adaptive(int en){_adaptive = en;}
		void enable_dead_zone(int en){_dead_zone = en;}
		void enable_lead(int en){_lead = en;}
		void enable_filter(int en){_filter = en;}
		void get_ctrl(float ref_position[2],float ball_position[2],float plate_angle_cmd[2]);
		
	private:
		float _T = 0.015; // loop period in seconds.
		int _adaptive = OFF;
		int _dead_zone = OFF;
		int _lead = OFF;
		int _filter = OFF;
		float _prev_ball_position[2] = {0};
		float _prev_pid_ctrl_vec[2] = {0};
		float _prev_lead_ctrl_vec[2] = {0};
		/* Lead compensation:
		 *                  1+(C/B)z^-1
		 *  lead_ctrl = B * -----------
		 *                  1 + A*z^-1
		 */
		float _A=0.2,_B=1,_C=0.1;
		
		float _tau = 0.5; // ud filter parameter: filter equation: TAUf*d/dt{CO*} + CO* = CO   
		float _ud[2] = {0};
		float _ui[2] = {0};
		float _up[2] = {0};
		float _pid_ctrl[2];
		// PID coefficients:
		float _Kp = -0.8; // [rad/cm]
		float _KiT = -0.01 * _T; // [rad/cm]
		float _KdT = -0.03 / _T; // [rad/cm]
		float _softKp = -0.5;
		float _softKiT = -0.01*_T;
		float _softKdT = -0.01/_T;
		float _radius_th = 0;
		float _velocity_th = 0;
		int _record_e = 0;
		int _record_output = 0;
};

/* Butterworth filter:
 * out[n] = a1*out[n-1] + b1*in[n-1] + b0*in[n]
 * coefficients values from matlab.
 */
class Butterworth1 
{
	public: 
		Butterworth1(float a1, float b[2]);
		void filter2d(float position_in[2],float position_out[2]);
	private:
		float _a1 = -0.5095; // denumerator 
		float _b[2] = {0.2452, 0.2452}; // numerator
		float _filter2d_in[2];
		float _filter2d_out[2];
		
};

class Butterworth2
{
	public: 
		Butterworth2(float a[3], float b[3]);
		void filter2d(float position_in[2],float position_out[2]);
	private:
		float _a[3];
		float _b[3];
		float _filter2d_in[2][2];
		float _filter2d_out[2][2];
		
};

#endif