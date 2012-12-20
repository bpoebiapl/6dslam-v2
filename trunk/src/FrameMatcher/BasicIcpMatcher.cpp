#include "BasicIcpMatcher.h"

using namespace std;

BasicIcpMatcher::BasicIcpMatcher()
{
	name = "BasicIcpMatcher";
	printf("new BasicIcpMatcher\n");
	nr_iters = 25;
	correspondenceDistance = 0.008;
	rejectionThreshold = 0.004;
	//exit(0);
}

BasicIcpMatcher::~BasicIcpMatcher()
{
	printf("delete BasicIcpMatcher\n");
}

void BasicIcpMatcher::update(){
	//icp.setMaximumIterations(nr_iters);
	//icp.setMaxCorrespondenceDistance (correspondenceDistance); 
	//icp.setTransformationEpsilon (1e-8); 
	//icp.setEuclideanFitnessEpsilon (1);
	//icp.setRANSACOutlierRejectionThreshold(rejectionThreshold);
	//icp.setRANSACIterations(10);
}

Transformation * BasicIcpMatcher::getTransformation(RGBDFrame * src, RGBDFrame * dst)
{
	pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
	icp.setMaximumIterations(nr_iters);
	icp.setMaxCorrespondenceDistance (correspondenceDistance); 
	icp.setTransformationEpsilon (1e-4); 
	icp.setEuclideanFitnessEpsilon(1e-6);
	icp.setRANSACOutlierRejectionThreshold(rejectionThreshold);
	//icp.setRANSACIterations(10);
	
	icp.setInputCloud (src->cloud);
	icp.setInputTarget(dst->cloud);
	pcl::PointCloud<pcl::PointXYZRGBNormal> Final;
	icp.align(Final);
	Transformation * transformation = new Transformation();
	transformation->src = src;
	transformation->dst = dst;
	transformation->transformationMatrix = icp.getFinalTransformation();
	printf("has converged: %i \n",icp.hasConverged());
	printf("score: %f\n",icp.getFitnessScore());
	transformation->weight = 1/icp.getFitnessScore();
	return transformation;
}
