#ifndef RGBDSegmentation_H_
#define RGBDSegmentation_H_
#include "../Frame_input.h"
#include "cv.h"
#include "highgui.h"
#include <opencv.hpp>
#include <stdlib.h>
#include <math.h>
#include <sys/types.h>
#include <sys/time.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct segment_pixel {
	int w;
	int h; 
	int r;
	int g;
	int b;
	float x;
	float y;
	float z;
	float edge;
	bool valid_depth;
	int segment_id;
};
using namespace Eigen;
struct segmentation_fit {
	//MyLinkedList<segment_pixel * > * segment;
	Matrix3f covMat;
	Matrix3f U;
	Vector3f S;
	Matrix3f V;
	Vector3f mean;
	int nr_valid;
	vector<int> * seg_w;
	vector<int> * seg_h;
};

class RGBDSegmentation
{
	public:
	int width;
	int height;
	//struct timeval start, end;
	//gettimeofday(&start, NULL);
	
	float ** r;
	float ** g;
	float ** b;
	
	float ** gray;

	float ** x;
	float ** y;
	float ** z;
	float ** z_valid;

	float ** norm_x;
	float ** norm_y;
	float ** norm_z;
	float ** norm_valid;
	
	float ** ii_gray;

	float ** ii_z;
	float ** ii_z_valid;

	float ** ii_norm_x;
	float ** ii_norm_y;
	float ** ii_norm_z;
	float ** ii_norm_valid;
	
	float ** depthEdge;
	float ** rgbEdge;
	float ** xedge;
	float ** yedge;
	float ** xrgbedge;
	float ** yrgbedge;
	
	int ** segmented;
	
	int smoothing_steps;
	int nr_angles;
	float *** smoothKernels;
	
	float d_scaleing;
	float centerX;
	float centerY;
	float invFocalX;
    float invFocalY;
	Calibration * calibration;
	vector<segmentation_fit * > * segment(IplImage * rgb_img,IplImage * depth_img);
	void init(IplImage * rgb_img,IplImage * depth_img);
	void extractSurfaces(vector<segmentation_fit * > * fit_segments, float ** edge, int width,int height);
	void extractSurfaces(vector<segmentation_fit * > * fit_segments, vector<int> * segment_w, vector<int> * segment_h, float ** edge, int & segment_id ,int iter, int width,int height, float min);
	segmentation_fit * fit_surface(vector<float> * seg_x, vector<float> * seg_y, vector<float> * seg_z);
	float ** getEdges();
	
	void disp_edge(string s , float** edge, int width,int height);
	void disp_xy_edge(string s , float** xedge,float** yedge, int width,int height);
	void disp_segments();
	void directional_smoothing(float** xedge, float** yedge, float** edge, int width, int height);
	RGBDSegmentation(Calibration * calib);
	~RGBDSegmentation();
};
#endif
