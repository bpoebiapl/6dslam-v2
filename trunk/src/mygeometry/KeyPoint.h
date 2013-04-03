#ifndef KeyPoint_H_
#define KeyPoint_H_
#include "FeatureDescriptor.h"
#include <Eigen/Core>
#include <algorithm>
#include <vector>
#include "Point.h"

class KeyPoint {
	public:
		int index_number;
		FeatureDescriptor * descriptor;
		Point * point;
		bool valid;
		int r;
		int g;
		int b;
		float stabilety;
		vector<float> cluster_distances;
		vector< pair <int , float > > cluster_distance_pairs;
		int cluster;
		vector< int > interesting_clusters;
		KeyPoint();
		~KeyPoint();
		void print();
		void sortDistances();
};


#endif
