#ifndef FPFHFeatureDescriptor_H_
#define FPFHFeatureDescriptor_H_
#include "FeatureDescriptor.h"
using namespace std;
class FPFHFeatureDescriptor : public FeatureDescriptor
{
	public:
	pcl::FPFHSignature33 feature;
	FPFHFeatureDescriptor();
	double distance(FPFHFeatureDescriptor * other_descriptor);
	void print();
	FPFHFeatureDescriptor * clone();
	~FPFHFeatureDescriptor();
};

inline ostream& operator << (ostream& os, const FPFHFeatureDescriptor& p){return os;}
#endif
