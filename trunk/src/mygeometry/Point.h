#ifndef Point_H_
#define Point_H_
#include <Eigen/Core>
class KeyPoint;
class Point
{

	public:
	KeyPoint * keypoint;
	float x;
	float y;
	float z;
	int w;
	int h;
	Eigen::Vector3f pos;
	Point(float x_, float y_, float z_, int w_,int h_);
	Point();
	~Point();
	void print();
	float distance(Point * point);
};
#endif
