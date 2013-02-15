#ifndef Line_H_
#define Line_H_
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include "Point.h"
#include "Plane.h"
#include <Eigen/Core>
#include <iostream>
#include <vector>
#include <Eigen/LU>
#include <Eigen/Eigenvalues> 
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <Eigen/Core>

using namespace Eigen;

class Line
{
	public:
	int id;
	float dir_x;
	float dir_y;
	float dir_z;
	float point_x;
	float point_y;
	float point_z;
	Eigen::Vector3f dir;
	Eigen::Vector3f point_a;
	Eigen::Vector3f point_b;
	
	
	Line(float x1, float y1, float z1, float x2, float y2, float z2);
	Line(Point * a, Point * b);
	Line(Plane * a, Plane * b);
	Line();
	~Line();
	
	float distance(Point * point);
	//float distance(pcl::PointXYZRGBNormal * point);
};
#endif
