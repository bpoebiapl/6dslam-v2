#include "Line.h"
int line_counter = 0;

Line::Line(Point * pa, Point * pb)
{
	id = line_counter++;
}

Line::Line(Plane * pa,Plane * pb)
{
	id = line_counter++;
	
	Eigen::Vector3f pa_norm(pa->normal_x,pa->normal_y,pa->normal_z);
	Eigen::Vector3f pa_mean(pa->point_x,pa->point_y,pa->point_z);
	Eigen::Vector3f pb_norm(pb->normal_x,pb->normal_y,pb->normal_z);
	Eigen::Vector3f pb_mean(pb->point_x,pb->point_y,pb->point_z);
	dir = pa_norm.cross(pb_norm);
	dir.normalize();
	Eigen::Vector3f orthLineDir = dir.cross(pa_norm);
	orthLineDir.normalize();
	float d = (pb_mean-pa_mean).dot(pb_norm)/(pb_norm.dot(orthLineDir));
	point_a = pa_mean+d*orthLineDir;
	point_b = point_a+dir;

	//ROS_INFO("norm1(%f,%f,%f) len: %f",pa_norm(0),pa_norm(1),pa_norm(2),pa_norm.norm());
	//ROS_INFO("mean1(%f,%f,%f) len: %f",pa_mean(0),pa_mean(1),pa_mean(2),pa_mean.norm());
	//ROS_INFO("norm2(%f,%f,%f) len: %f",pb_norm(0),pb_norm(1),pb_norm(2),pb_norm.norm());
	//ROS_INFO("mean2(%f,%f,%f) len: %f",pb_mean(0),pb_mean(1),pb_mean(2),pb_mean.norm());
	//ROS_INFO("dir(%f,%f,%f) len: %f",dir(0),dir(1),dir(2),dir.norm());
	//ROS_INFO("orthLineDir(%f,%f,%f) len: %f",orthLineDir(0),orthLineDir(1),orthLineDir(2),orthLineDir.norm());
	//ROS_INFO("d = %f",d);
	//ROS_INFO("point_a(%f,%f,%f)",point_a(0),point_a(1),point_a(2));

	dir_x = dir(0);
	dir_y = dir(1);
	dir_z = dir(2); 
	point_x = point_a(0);
	point_y = point_a(1);
	point_z = point_a(2);
}

Line::Line(){id = line_counter++;}
Line::~Line(){}

float Line::distance(Point * p){return ((p->pos - point_a).cross(p->pos - point_b)).norm();}
