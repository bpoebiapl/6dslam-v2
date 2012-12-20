#ifndef Frame_input_H_
#define Frame_input_H_
#include <string>
#include <stdio.h>
using namespace std;
class Calibration{
	public:
	float fx;
	float fy;
	float cx;
	float cy;
	float ds;
	float scale;
};

class Frame_input{
	public:
	double depth_timestamp;
	string depth_path;
	double rgb_timestamp;
	string rgb_path;
	Calibration * calibration;
};
#endif
