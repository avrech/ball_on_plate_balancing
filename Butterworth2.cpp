/* 	Class: 			Butterworth1
	Author: 		Avrech Ben-David
	Description: 	This class is part of Ball On Plate project.
	Functionality: 	Controls the touch screen pins,
					Reads the ball position in [cm]
*/
#include "Ball_On_Plate.h"
Butterworth2::Butterworth2(float a[3], float b[3]): 
	_a({a[0],a[1],a[2]}),_b({b[0],b[1],b[2]}),_filter2d_in({{0},{0}}),_filter2d_out({{0},{0}}){}
		
void Butterworth2::filter2d(float position_in[2],float position_out[2])
{	
	for(int axisIdx = 0; axisIdx < 2; ++axisIdx){
		position_out[axisIdx] =	-_a[1]*_filter2d_out[axisIdx][0]-_a[2]*_filter2d_out[axisIdx][1] 
								+ _b[2]*_filter2d_in[axisIdx][1]+ _b[1]*_filter2d_in[axisIdx][0]+_b[0]*position_in[axisIdx];
		_filter2d_in[axisIdx][1] = _filter2d_in[axisIdx][0];
		_filter2d_in[axisIdx][0] = position_in[axisIdx]; // save for next call.
		_filter2d_out[axisIdx][1] = _filter2d_out[axisIdx][0];
		_filter2d_out[axisIdx][0] = position_out[axisIdx];
	}
}
