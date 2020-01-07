/* 	Class: 			Touchpad
	Author: 		Avrech Ben-David
	Description: 	This class is part of Ball On Plate project.
	Functionality: 	Controls the touch screen pins,
					Reads the ball position in [cm]
*/
#include "Ball_On_Plate.h"

Touchpad::Touchpad(int pins[4], float offset[2], float scaling_factor[2],int d_th, int max_sampling):
_prev_sample({0}),_d_th(d_th),_max_sampling(max_sampling)
{
	for (int i=0;i<4;i++) _pins[i] = pins[i];
	_x_offset = offset[0];
	_y_offset = offset[1];
	_x_scaling = scaling_factor[0];
	_y_scaling = scaling_factor[1];
}

// Function: read_pos - update ball_pos_vec to the cartesian coordinates of the ball in [cm]
void Touchpad::read_ball_position(float position[2]){  //In this function we read position of the ball on the touch screen.
 int temp0,temp1;
 for(unsigned int iter = _max_sampling; iter > 0; iter--)
 {
	 // configuration for reading the x value
	 pinMode(_pins[0], INPUT); pinMode(_pins[2], INPUT); 
	 pinMode(_pins[3], OUTPUT); digitalWrite(_pins[3], LOW);
	 pinMode(_pins[1], OUTPUT); digitalWrite(_pins[1], HIGH);
	 delay(2);              // let things settle
	 temp0 = analogRead(_pins[2]);
	 //Serial.println(temp);// TODO: debug.
	 
	 // configuration for reading the y value
	 pinMode(_pins[1], INPUT); pinMode(_pins[3], INPUT);
	 pinMode(_pins[2], OUTPUT); digitalWrite(_pins[2], LOW);
	 pinMode(_pins[0], OUTPUT); digitalWrite(_pins[0], HIGH);
	 delay(2);              // let things settle.
	 temp1 = analogRead(_pins[3]);
	 if (abs(temp0-_prev_sample[0]) < _d_th && abs(temp1-_prev_sample[1]) < _d_th) break;
	 
 } 
 _prev_sample[0] = temp0;
 _prev_sample[1] = temp1;
 position[0] = (temp0-_x_offset)*_x_scaling; // read the X value in [cm]
 position[1] = (temp1-_y_offset)*_y_scaling; // read the Y value in [cm]
}

void Touchpad::init(void){  //In this function we read position of the ball on the touch screen.
 int temp;
 // configuration for reading the x value
 pinMode(_pins[0], INPUT); pinMode(_pins[2], INPUT); 
 pinMode(_pins[3], OUTPUT); digitalWrite(_pins[3], LOW);
 pinMode(_pins[1], OUTPUT); digitalWrite(_pins[1], HIGH);
 delay(1);              // let things settle
 // check sample validity:
 _prev_sample[0] = analogRead(_pins[2]);
 
 // configuration for reading the y value
 pinMode(_pins[1], INPUT); pinMode(_pins[3], INPUT);
 pinMode(_pins[2], OUTPUT); digitalWrite(_pins[2], LOW);
 pinMode(_pins[0], OUTPUT); digitalWrite(_pins[0], HIGH);
 delay(1);              // let things settle.
 _prev_sample[1] = analogRead(_pins[3]);
 
}


