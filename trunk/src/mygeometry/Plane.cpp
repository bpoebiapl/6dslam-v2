#include "Plane.h"
#include <iostream>
#include <vector>
#include <algorithm>
//#include "cv.h"
//#include "highgui.h"
//#include <opencv.hpp>
#include <Eigen/LU>
#include <Eigen/Eigenvalues> 
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <Eigen/Core>

using namespace Eigen;
using namespace std;
int plane_counter = 0;

Plane::Plane(Point * pa, Point * pb,Point * pc)
{
	color_histogram = 0;
	id = plane_counter++;
	use_boundaries = false;
	setup(pa, pb, pc);
}

Plane::Plane(pcl::PointXYZRGBNormal * pa, pcl::PointXYZRGBNormal * pb, pcl::PointXYZRGBNormal * pc)
{
	color_histogram = 0;
	id = plane_counter++;
	use_boundaries = false;
	setup(pa, pb, pc);
}

Plane::Plane(std::vector<Point *> * points)
{
	color_histogram = 0;
	id = plane_counter++;
	use_boundaries = false;
	setup(points);
}

Plane::Plane(std::vector<pcl::PointXYZRGBNormal *> * points)
{
	color_histogram = 0;
	id = plane_counter++;
	use_boundaries = false;
	setup(points);
}

Plane::Plane()
{
	color_histogram = 0;
	id = plane_counter++;
	use_boundaries = false;
}

Plane::~Plane()
{
	if(color_histogram != 0){delete color_histogram;}
}

float ** getDataFromVector(std::vector<Point *> * points)
{
	int size = points->size();
	float ** retval = new float*[3];
	retval[0] = new float[size];
	retval[1] = new float[size];
	retval[2] = new float[size];
	
	for(int i = 0; i < size; i++)
	{
		Point * p = points->at(i);
		retval[0][i] = p->x;
		retval[1][i] = p->y;
		retval[2][i] = p->z;
	}
	return retval;
}

float ** getDataFromVector(std::vector<pcl::PointXYZRGBNormal *> * points)
{
	
	int size = points->size();
	float ** retval = new float*[3];
	retval[0] = new float[size];
	retval[1] = new float[size];
	retval[2] = new float[size];
	
	for(int i = 0; i < size; i++)
	{
		pcl::PointXYZRGBNormal * p = points->at(i);
		retval[0][i] = p->x;
		retval[1][i] = p->y;
		retval[2][i] = p->z;
	}
	return retval;
}

float * getMean(float ** data, int nr_data)
{
	float mx = 0;
	float my = 0;
	float mz = 0;
	
	for(int i = 0; i < nr_data; i++)
	{
		mx+=data[0][i];
		my+=data[1][i];
		mz+=data[2][i];
	}
	float * retval = new float[3];
	retval[0] = mx / float(nr_data);
	retval[1] = my / float(nr_data);
	retval[2] = mz / float(nr_data);
	return retval;
}

MatrixXf getCov(float ** data, int nr_data , float * mean)
{
	MatrixXf covMat(3,3);
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j <= i; j++)
		{
			float * col1 	= data[i];
			float mean1 	= mean[i];
			float * col2 	= data[j];
			float mean2 	= mean[j];
			float sum = 0;
			for(int k = 0; k < nr_data; k++)
			{
				sum+=(col1[k]-mean1)*(col2[k]-mean2);
			}
			covMat(i,j)=sum/float(nr_data-1);
			covMat(j,i)=covMat(i,j);
		}
	}
	return covMat;
}

void Plane::setup(std::vector<pcl::PointXYZRGBNormal *> * points)
{
	
	int size = points->size();
	weight = size;
	float ** data = getDataFromVector(points);
	float *  mean = getMean(data,size);
	MatrixXf covMat = getCov(data, size , mean);
	JacobiSVD<MatrixXf> svd(covMat, ComputeThinU | ComputeThinV);

	MatrixXf U = svd.matrixU();
	U = -U;	
	point_x = mean[0];
	point_y = mean[1];
	point_z = mean[2];
	mean[0] = 0;
	mean[1] = 0;
	mean[2] = 0;
	
	plane_points(0,0) 	= 		 mean[0];
	plane_points(0,1) 	= 		 mean[1];
	plane_points(0,2) 	= 		 mean[2];
	
	plane_points(1,0) 	= U(0,0)+mean[0];
	plane_points(1,1) 	= U(1,0)+mean[1];
	plane_points(1,2) 	= U(2,0)+mean[2];
	
	plane_points(2,0) 	= U(0,1)+mean[0];
	plane_points(2,1) 	= U(1,1)+mean[1];
	plane_points(2,2) 	= U(2,1)+mean[2];
	
	normal_x 			= U(0,2);
	normal_y 			= U(1,2);
	normal_z 			= U(2,2);
	
	delete[] mean;
	delete[] data[0];
	delete[] data[1];
	delete[] data[2];
	delete[] data;
	getABCDcoefs();
}

void Plane::setup(std::vector<Point *> * points)
{
	
	int size = points->size();
	for(int i = 0; i < size; i++)
	{
		p_w.push_back(points->at(i)->w);
		p_h.push_back(points->at(i)->h);
	}
	weight = size;
	float ** data = getDataFromVector(points);
	float *  mean = getMean(data,size);
	MatrixXf covMat = getCov(data, size , mean);
	JacobiSVD<MatrixXf> svd(covMat, ComputeThinU | ComputeThinV);

	MatrixXf U = svd.matrixU();
	S = svd.singularValues();
	S.normalize();
	//ROS_INFO("SVD:%f %f %f",S[0],S[1],S[2]);
	
	U = -U;	
	point_x = mean[0];
	point_y = mean[1];
	point_z = mean[2];
	mean[0] = 0;
	mean[1] = 0;
	mean[2] = 0;
	
	plane_points(0,0) 	= 		 mean[0];
	plane_points(0,1) 	= 		 mean[1];
	plane_points(0,2) 	= 		 mean[2];
	
	plane_points(1,0) 	= U(0,0)+mean[0];
	plane_points(1,1) 	= U(1,0)+mean[1];
	plane_points(1,2) 	= U(2,0)+mean[2];
	
	plane_points(2,0) 	= U(0,1)+mean[0];
	plane_points(2,1) 	= U(1,1)+mean[1];
	plane_points(2,2) 	= U(2,1)+mean[2];
	
	normal_x 			= U(0,2);
	normal_y 			= U(1,2);
	normal_z 			= U(2,2);
	float normal_normalizer = sqrt(normal_x*normal_x+normal_y*normal_y+normal_z*normal_z);
	normal_x /= normal_normalizer;
	normal_y /= normal_normalizer;
	normal_z /= normal_normalizer;
	
	delete[] mean;
	delete[] data[0];
	delete[] data[1];
	delete[] data[2];
	delete[] data;
	getABCDcoefs();
}

void Plane::getABCDcoefs()
{
	//ROS_INFO("Mean: %f %f %f",point_x,point_y,point_z);
	Matrix3f plane_points_mean;
	plane_points_mean(0,0) 	= plane_points(0,0)+point_x;
	plane_points_mean(0,1) 	= plane_points(0,1)+point_y;
	plane_points_mean(0,2) 	= plane_points(0,2)+point_z;
	
	plane_points_mean(1,0) 	= plane_points(1,0)+point_x;
	plane_points_mean(1,1) 	= plane_points(1,1)+point_y;
	plane_points_mean(1,2) 	= plane_points(1,2)+point_z;
	
	plane_points_mean(2,0) 	= plane_points(2,0)+point_x;
	plane_points_mean(2,1) 	= plane_points(2,1)+point_y;
	plane_points_mean(2,2) 	= plane_points(2,2)+point_z;
	
	Matrix3f a_det_points;
	a_det_points(0,0) 	= 1;
	a_det_points(0,1) 	= plane_points(0,1)+point_y;
	a_det_points(0,2) 	= plane_points(0,2)+point_z;
	
	a_det_points(1,0) 	= 1;
	a_det_points(1,1) 	= plane_points(1,1)+point_y;
	a_det_points(1,2) 	= plane_points(1,2)+point_z;
	
	a_det_points(2,0) 	= 1;
	a_det_points(2,1) 	= plane_points(2,1)+point_y;
	a_det_points(2,2) 	= plane_points(2,2)+point_z;
	
	Matrix3f b_det_points;
	b_det_points(0,0) 	= plane_points(0,0)+point_x;
	b_det_points(0,1) 	= 1;
	b_det_points(0,2) 	= plane_points(0,2)+point_z;
	
	b_det_points(1,0) 	= plane_points(1,0)+point_x;
	b_det_points(1,1) 	= 1;
	b_det_points(1,2) 	= plane_points(1,2)+point_z;
	
	b_det_points(2,0) 	= plane_points(2,0)+point_x;
	b_det_points(2,1) 	= 1;
	b_det_points(2,2) 	= plane_points(2,2)+point_z;
	
	Matrix3f c_det_points;
	c_det_points(0,0) 	= plane_points(0,0)+point_x;
	c_det_points(0,1) 	= plane_points(0,1)+point_y;
	c_det_points(0,2) 	= 1;
	
	c_det_points(1,0) 	= plane_points(1,0)+point_x;
	c_det_points(1,1) 	= plane_points(1,1)+point_y;
	c_det_points(1,2) 	= 1;
	
	c_det_points(2,0) 	= plane_points(2,0)+point_x;
	c_det_points(2,1) 	= plane_points(2,1)+point_y;
	c_det_points(2,2) 	= 1;
	
	float D = plane_points_mean.determinant();
	d = 100.0f;
	a = -(d/D) *a_det_points.determinant();
	b = -(d/D) *b_det_points.determinant();
	c = -(d/D) *c_det_points.determinant();
}

void Plane::setup(pcl::PointXYZRGBNormal * a, pcl::PointXYZRGBNormal * b, pcl::PointXYZRGBNormal * c)
{
	Eigen::Vector3f x1(b->x - a->x,b->y - a->y,b->z - a->z);
	Eigen::Vector3f x2(c->x - a->x,c->y - a->y,c->z - a->z);
	Eigen::Vector3f nhat_top = x1.cross(x2);
	Eigen::Vector3f nhat = nhat_top / nhat_top.norm();
	normal_x = nhat(0);
	normal_y = nhat(1);
	normal_z = nhat(2);
	point_x = a->x;
	point_y = a->y;
	point_z = a->z;
}

void Plane::setup(Point * a, Point * b,Point * c)
{
	Eigen::Vector3f x1(b->x - a->x,b->y - a->y,b->z - a->z);
	Eigen::Vector3f x2(c->x - a->x,c->y - a->y,c->z - a->z);
	Eigen::Vector3f nhat_top = x1.cross(x2);
	Eigen::Vector3f nhat = nhat_top / nhat_top.norm();
	normal_x = nhat(0);
	normal_y = nhat(1);
	normal_z = nhat(2);
	point_x = a->x;
	point_y = a->y;
	point_z = a->z;
}

inline float Plane::distance(pcl::PointXYZRGBNormal * point){
	return (normal_x*(point_x-point->x) + normal_y*(point_y-point->y) + normal_z*(point_z-point->z));
}

inline float Plane::distance(Point * point){
	return normal_x*(point_x-point->x) + normal_y*(point_y-point->y) + normal_z*(point_z-point->z);
}

void Plane::project_points(std::vector<Point *> points){
	float top = normal_x*point_x + normal_y*point_y + normal_z*point_z;
	int size = points.size();
	for(int i = 0; i < size; i++)
	{
		Point * p = points.at(i);
		float d = top/(normal_x*p->x+normal_y*p->y+normal_z*p->z);
		p->x*=d;
		p->y*=d;
		p->z*=d;
		p->pos(0) = p->x;
		p->pos(1) = p->y;
		p->pos(2) = p->z;
	}
}

bool compare_vec(std::vector<Point *> * a,std::vector<Point *> * b){return a->size()>b->size();}

std::vector< Plane *> * Plane::extract_subplanes(std::vector<Point *> * points){
	int nr_points = points->size();
	int max_width 	=  0;
	int max_height 	=  0;
	for(int i = 0; i < nr_points; i++)
	{
		Point * p = points->at(i);
		if(max_width < p->w)	{max_width = p->w;}
		if(max_height < p->h)	{max_height = p->h;}
	}
	
	Point *** pointMat = new Point**[max_width];
	for(int i = 0; i < max_width; i++)
	{
		pointMat[i] = new Point*[max_height];
		for(int j = 0; j < max_height; j++)
		{
			pointMat[i][j] = 0;
		}
	}
	
	for(int i = 0; i < nr_points; i++)
	{
		Point * p = points->at(i);
		pointMat[p->w-1][p->h-1] = p;
	}
	
	std::vector< std::vector<Point *> *> * components = new std::vector< std::vector<Point *> *>();

	for(int i = 0; i < max_width; i++)
	{
		for(int j = 0; j < max_height; j++)
		{
			if(pointMat[i][j] != 0)
			{
				std::vector<Point *>* component = new std::vector<Point *>();
				std::vector<Point *>* todo 		= new std::vector<Point *>();
				components						->push_back(component);
				todo							->push_back(pointMat[i][j]);
				pointMat[i][j] 					= 0;
				while(!todo->empty())
				{
					Point * p = todo->back();
					todo->pop_back();
					component->push_back(p);
					int w = p->w-1;
					int h = p->h-1;
					
					int a_start = w-1;
					if(a_start < 0)				{a_start = 0;}
					int a_stop = w+1;
					if(a_stop > max_width-1)	{a_stop = max_width-1;}
					
					int b_start = h-1;
					if(b_start < 0)				{b_start = 0;}
					int b_stop = h+1;
					if(b_stop > max_height-1)	{b_stop = max_height-1;}
					
					for(int a = a_start; a <= a_stop;a++)
					{
						for(int b = b_start; b <= b_stop;b++)
						{ 
							if(pointMat[a][b]!=0){
								todo->push_back(pointMat[a][b]);
								pointMat[a][b]=0;
							}
						}
					}
				}
				delete todo;
			}
		}
	}
	
	for(int i = 0; i < max_width; i++){delete pointMat[i];}
	delete pointMat;
	
	std::sort(components->begin(), components->end(), compare_vec);
	std::vector< Plane *> * sub_planes = new std::vector< Plane *>();
	
	Plane * pl = new Plane(components->at(0));
	sub_planes->push_back(pl);
    return sub_planes;
}

void Plane::merge(Plane * pl){
	float part_org = weight/(weight+pl->weight);
	float part_new = pl->weight/(weight+pl->weight);
	weight  += pl->weight;
	normal_x = normal_x*part_org + pl->normal_x*part_new;
	normal_y = normal_y*part_org + pl->normal_y*part_new;
	normal_z = normal_z*part_org + pl->normal_z*part_new;
	point_x  = point_x*part_org + pl->point_x*part_new;
	point_y  = point_y*part_org + pl->point_y*part_new;
	point_z  = point_z*part_org + pl->point_z*part_new;

	//TODO:: add more stuff
	color_histogram->mul(part_org);
	pl->color_histogram->mul(part_new);
	color_histogram->add(pl->color_histogram);
	//delete pl;
}

float Plane::angle(Plane * pl){
	return normal_x*pl->normal_x + normal_y*pl->normal_y+normal_z*pl->normal_z;
}
