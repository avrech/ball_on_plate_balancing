/* 	Class: 			Butterworth1
	Author: 		Avrech Ben-David
	Description: 	This class is part of Ball On Plate project.
	Functionality: 	Controls the touch screen pins,
					Reads the ball position in [cm]
*/
#include "Ball_On_Plate.h"
Butterworth1::Butterworth1(float a1, float b[2]): 
	_a1(a1),_b({b[0],b[1]}),_filter2d_in({0}),_filter2d_out({0}){}
		
void Butterworth1::filter2d(float position_in[2],float position_out[2])
{	
	position_out[0] = _filter2d_out[0] = -_a1*_filter2d_out[0] + _b[1]*_filter2d_in[0]+_b[0]*position_in[0];
	position_out[1] = _filter2d_out[1] = -_a1*_filter2d_out[1] + _b[1]*_filter2d_in[1]+_b[0]*position_in[1];
	_filter2d_in[0] = position_in[0]; // save for next call.
	_filter2d_in[1] = position_in[1];
}
