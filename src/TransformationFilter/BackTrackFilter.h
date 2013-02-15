#ifndef BackTrackFilter_H_
#define BackTrackFilter_H_
#include "TransformationFilter.h"

using namespace std;

class BackTrackFilter: public TransformationFilter
{
	public:
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		string name;
		float distance_threshold;
		int nr_iter;
		bool fast;
		BackTrackFilter();
		~BackTrackFilter();
		Transformation * filterTransformation(Transformation * input);
		void print();
		void setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view);
		bool jcTest(Point * src_a, Point * src_b, Point * src_c, Point * dst_a, Point * dst_b, Point * dst_c);
};

#include "BackTrackFilter.h"
#endif
