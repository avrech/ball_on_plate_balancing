/* 	Class: 			StewartPlatform
	Author: 		Avrech Ben-David
	Description: 	This class is part of Ball On Plate project.
	Functionality: 	calculate motors angles from plate angles,
					set motors angles.
*/
#include "Ball_On_Plate.h"
StewartPlatform::StewartPlatform(	const int motor_pins[6],
									const float motor_ang_offset[6], 
									const float b[6][2], 
									const float p[6][2], 
									const float T[3], 
									const float beta[6]): 
	// for the odd motors 0,2,4, add PI to the argument, yields factor (-1).
	_cos_beta({-cos(beta[0]),cos(beta[1]),-cos(beta[2]),cos(beta[3]),-cos(beta[4]),cos(beta[5])}),
	_sin_beta({-sin(beta[0]),sin(beta[1]),-sin(beta[2]),sin(beta[3]),-sin(beta[4]),sin(beta[5])}),
	_T({T[0],T[1],T[2]})	
{
    for(int m=0;m<6;m++){
		_motor_pins[m] = motor_pins[m];
		_motor_ang_offset[m] = motor_ang_offset[m];
		for(int j=0;j<2;j++){
			_b[m][j] = b[m][j];
			_p[m][j] = p[m][j];
		}
	}
}

void StewartPlatform::set_motors_angles(float motors_angles[6]){
	int i;
	float ang[6];
	for(i=0;i<6;i+=2) ang[i] = _motor_ang_offset[i] + (motors_angles[i]);// +fir_mtr_ang[i])/2;
	for(i=1;i<6;i+=2) ang[i] = _motor_ang_offset[i] - (motors_angles[i]);// +fir_mtr_ang[i])/2;
	for(i=0;i<6;i++) _myservo[i].write(ang[i]);
}

void StewartPlatform::init_motors(void){
	for (int i = 0; i < 6; i++) _myservo[i].attach(_motor_pins[i]);
	float init_ang[6] = {0,0,0,0,0,0};
	set_motors_angles(init_ang);
}

void StewartPlatform::get_motors_angles(float roll, float pitch, float alpha_vec[6]){
	
  float sin_roll = sin(roll*DEG2RAD); 
  float cos_roll = cos(roll*DEG2RAD); 
  float sin_pitch = sin(pitch*DEG2RAD);
  float cos_pitch = cos(pitch*DEG2RAD);
  float R[3][3] = { {cos_pitch,  sin_pitch*sin_roll, sin_pitch*cos_roll},
                    {0,          cos_roll,           -sin_roll},
                    {-sin_pitch, cos_pitch*sin_roll, cos_pitch*cos_roll}
                  };
  // q[jointIdx][axisIdx - x,y,z]:              *** note: p[i][3] == 0 for all i, R[1][0] == 0 ****
  float q[6][3] = { {_p[0][0]*R[0][0]+_p[0][1]*R[0][1]+_T[0], _p[0][1]*R[1][1]+_T[1],  _p[0][0]*R[2][0]+_p[0][1]*R[2][1]+_T[2]},
                    {_p[1][0]*R[0][0]+_p[1][1]*R[0][1]+_T[0], _p[1][1]*R[1][1]+_T[1],  _p[1][0]*R[2][0]+_p[1][1]*R[2][1]+_T[2]},
                    {_p[2][0]*R[0][0]+_p[2][1]*R[0][1]+_T[0], _p[2][1]*R[1][1]+_T[1],  _p[2][0]*R[2][0]+_p[2][1]*R[2][1]+_T[2]},
                    {_p[3][0]*R[0][0]+_p[3][1]*R[0][1]+_T[0], _p[3][1]*R[1][1]+_T[1],  _p[3][0]*R[2][0]+_p[3][1]*R[2][1]+_T[2]},
                    {_p[4][0]*R[0][0]+_p[4][1]*R[0][1]+_T[0], _p[4][1]*R[1][1]+_T[1],  _p[4][0]*R[2][0]+_p[4][1]*R[2][1]+_T[2]},
                    {_p[5][0]*R[0][0]+_p[5][1]*R[0][1]+_T[0], _p[5][1]*R[1][1]+_T[1],  _p[5][0]*R[2][0]+_p[5][1]*R[2][1]+_T[2]}
                  };
  int motor_idx, itr = 0;
  float max_angle; //  [rad].
  float min_angle; //  [rad].
  float arm_joint[3];
  float tmp_alpha, cos_tmp_alpha, leg_len, err;
  for(motor_idx = 0;motor_idx < MOTORS_NUM; motor_idx++)
  {
    // initiate loop:
    tmp_alpha = 0;
    max_angle = 1.5707; // = 90*DEG2RAD; //  [rad].
    min_angle = -1.5707; // = -90*DEG2RAD; //  [rad].
    itr = 0;
    // calculate arm-rod connection coordinates:
    cos_tmp_alpha = cos(tmp_alpha);
    arm_joint[0]  = _b[motor_idx][0] + ARM_LEN*cos_tmp_alpha*_cos_beta[motor_idx];
    arm_joint[1]  = _b[motor_idx][1] + ARM_LEN*cos_tmp_alpha*_sin_beta[motor_idx];
    arm_joint[2]  = ARM_LEN*sin(tmp_alpha);
    // calculate leg length:
    leg_len = sqrt( sqr(q[motor_idx][0]-arm_joint[0]) +
                    sqr(q[motor_idx][1]-arm_joint[1]) +
                    sqr(q[motor_idx][2]-arm_joint[2])
                  );
    // check error:
    err = leg_len - ROD_LEN; // units [mm].
    while(abs(err) > _search_err_th && itr < _search_max_iter)
    {
      if(err > 0) min_angle = tmp_alpha;
      else max_angle = tmp_alpha;
      tmp_alpha = (min_angle+max_angle)/2;
      cos_tmp_alpha = cos(tmp_alpha);
      // repeat leg_len calculus with new motor angle.
      arm_joint[0]  = _b[motor_idx][0] + ARM_LEN*cos_tmp_alpha*_cos_beta[motor_idx];
      arm_joint[1]  = _b[motor_idx][1] + ARM_LEN*cos_tmp_alpha*_sin_beta[motor_idx];
      arm_joint[2]  = ARM_LEN*sin(tmp_alpha);
      // calculate leg length:
      leg_len = sqrt( sqr(q[motor_idx][0]-arm_joint[0]) +
                      sqr(q[motor_idx][1]-arm_joint[1]) +
                      sqr(q[motor_idx][2]-arm_joint[2])
                    );
      // calculate error:
      err = leg_len - ROD_LEN; // units [mm].
      itr++;
    }
    alpha_vec[motor_idx] = tmp_alpha*RAD2DEG; 
    // repeat loop for the next motor.
  }

}
	