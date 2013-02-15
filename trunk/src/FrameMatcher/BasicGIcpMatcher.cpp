#include "BasicGIcpMatcher.h"

using namespace std;

BasicGIcpMatcher::BasicGIcpMatcher()
{
	name = "BasicGIcpMatcher";
	printf("new BasicGIcpMatcher\n");
	nr_iters = 25;
	correspondenceDistance = 0.008;
	rejectionThreshold = 0.004;
	//exit(0);
}

BasicGIcpMatcher::~BasicGIcpMatcher()
{
	printf("delete BasicGIcpMatcher\n");
}

void BasicGIcpMatcher::update(){}

Transformation * BasicGIcpMatcher::getTransformation(RGBDFrame * src, RGBDFrame * dst)
{
	struct timeval start, end;
	gettimeofday(&start, NULL);
	
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaximumIterations(nr_iters);
	icp.setMaxCorrespondenceDistance (correspondenceDistance); 
	icp.setTransformationEpsilon (1e-6); 
	icp.setEuclideanFitnessEpsilon(1e-8);
	icp.setRANSACOutlierRejectionThreshold(rejectionThreshold);
	//icp.setRANSACIterations(10);
	
	icp.setInputCloud (src->xyz_);
	icp.setInputTarget(dst->xyz_);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	
	Transformation * transformation = new Transformation();
	transformation->src = src;
	transformation->dst = dst;
	transformation->transformationMatrix = icp.getFinalTransformation();
	transformation->weight = 1/icp.getFitnessScore();
	
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("gicp cost: %f\n",time);
	return transformation;
}
