#ifndef IntegerHistogramFeatureDescriptor_H_
#define IntegerHistogramFeatureDescriptor_H_
#include "FeatureDescriptor.h"
using namespace std;
class IntegerHistogramFeatureDescriptor : public FeatureDescriptor
{
	public:
	int * descriptor;
	int length;
	
	IntegerHistogramFeatureDescriptor();
	IntegerHistogramFeatureDescriptor(string path);
	IntegerHistogramFeatureDescriptor(int * feature_descriptor, int feature_length);
	double distance(IntegerHistogramFeatureDescriptor * other_descriptor);
	void print();
	void store(string path);
	IntegerHistogramFeatureDescriptor * clone();
	void update(vector<FeatureDescriptor * > * input);
	~IntegerHistogramFeatureDescriptor();
};
#endif
