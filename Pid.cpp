/* 	Class: 			TouchScreen
	Author: 		Avrech Ben-David
	Description: 	This class is part of Ball On Plate project.
	Functionality: 	calculate motors angles from plate angles,
					set motors angles.
*/
#include "Ball_On_Plate.h"
Pid::Pid(float T):_T(T){}
void Pid::set_pid_coef(float Kp,float Ki, float Kd){
	_Kp = Kp;
	_KiT = Ki*_T;
	_KdT = Kd/_T;	
}
void Pid::set_pid_soft_coef(float softKp,float softKi, float softKd){
	_softKp = softKp;
	_softKiT = softKi*_T;
	_softKdT = softKd/_T;	
}
		


// inputs: ball_position[cm], ref_position[cm], plate_angle_cmd[rad]
void Pid::get_ctrl(float ref_position[2],float ball_position[2],float plate_angle_cmd[2]){
    int axis_idx,plate_angle_idx;
    float e;
    float ud;
    float lead_ctrl;
    float kp,kit,kdt; // select the pid coefficients.
    float velocity[2] = {abs(ball_position[0] - _prev_ball_position[0])/_T,
						abs(ball_position[1] - _prev_ball_position[1])/_T};
	float radius[2] = {abs(ref_position[0]-ball_position[0]),abs(ref_position[1]-ball_position[1])};
    // axis_idx=0 for x, axis_idx=1 for y:
     
	for (axis_idx=0; axis_idx<2; axis_idx++) 
	{
		plate_angle_idx = (axis_idx==0) ? 1 : 0;
      
		if(_dead_zone = ON && radius[axis_idx]< _radius_th && velocity[axis_idx] < _velocity_th)
		{
			plate_angle_cmd[axis_idx] = _pid_ctrl[axis_idx];
		}
		else
		{
			// adaptive select pid coeff:
			if(_adaptive == ON && radius[axis_idx] < 1 && velocity[axis_idx] < 1) {kp = _softKp; kit = _softKiT; kdt = _softKdT;}
			else {kp = _Kp; kit = _KiT; kdt = _KdT;}
		  
			// PID:
			// error computation:
			e = ref_position[axis_idx]-ball_position[axis_idx];             
			if(_record_e) Serial.println(e);
			// proportional:
			_up[axis_idx] = kp * e;                              
		  
			// integral:
			_ui[axis_idx] = _ui[axis_idx] + kit * e;
		  
			// differential:
			ud = kdt * (ball_position[axis_idx]-_prev_ball_position[axis_idx]);
		  
			// Dead Width - disable ud:
			//if(_dead_zone == ON && abs(ball_position[axis_idx]) < 1) ud = 0;
		  
			// Filter ud: first order IIR.
			if(_filter == ON) _ud[axis_idx] = _ud[axis_idx] + _tau*(ud-_ud[axis_idx]);
			else _ud[axis_idx] = ud;
		  
			// PID output:
			_pid_ctrl[axis_idx] = _up[axis_idx] + _ui[axis_idx] - _ud[axis_idx];
		  
			// update variables for next iteration:
			_prev_ball_position[axis_idx] = ball_position[axis_idx];
		  
			// Saturate pid_ctrl:
			if(_pid_ctrl[axis_idx] > MAX_CTRL) _pid_ctrl[axis_idx] = MAX_CTRL;  
			else if(_pid_ctrl[axis_idx] < -MAX_CTRL) _pid_ctrl[axis_idx] = -MAX_CTRL;  
		  
			if (_lead == ON){
				// lead compensation:
				lead_ctrl = _A*_prev_lead_ctrl_vec[axis_idx] + _B*_pid_ctrl[axis_idx] +_C*_prev_pid_ctrl_vec[axis_idx];
				// update for next iteration:
				_prev_lead_ctrl_vec[axis_idx] = lead_ctrl;
				_prev_pid_ctrl_vec[axis_idx] = _pid_ctrl[axis_idx];
			
				if(lead_ctrl > MAX_CTRL) plate_angle_cmd[plate_angle_idx] = MAX_CTRL;  
				else if(lead_ctrl < -MAX_CTRL) plate_angle_cmd[plate_angle_idx] = -MAX_CTRL;  
				else plate_angle_cmd[plate_angle_idx] = lead_ctrl;  
			
			}
			else {
				plate_angle_cmd[plate_angle_idx] = _pid_ctrl[axis_idx];
			}
			if(_record_output)Serial.println(plate_angle_cmd[axis_idx]);
		}
   }
   
	   
}

