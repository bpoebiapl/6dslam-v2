#ifndef SurfFeatureDescriptor128_H_
#define SurfFeatureDescriptor128_H_
#include "FeatureDescriptor.h"
using namespace std;
class SurfFeatureDescriptor128 : public FeatureDescriptor
{
	public:
	float stabilety;
	int laplacian;
	
	float * descriptor;
	unsigned int descriptor_length;
	
	SurfFeatureDescriptor128();
	SurfFeatureDescriptor128(float * feature_descriptor);
	SurfFeatureDescriptor128(float * feature_descriptor, int feature_laplacian);
	double distance(SurfFeatureDescriptor128 * other_descriptor);
	SurfFeatureDescriptor128 * clone();
	void print();
	~SurfFeatureDescriptor128();
};

inline ostream& operator << (ostream& os, const SurfFeatureDescriptor128& p)
{
	for (int i = 0; i < 128; ++i)
		os << (i == 0 ? "(" : "") << p.descriptor[i] << (i < 127 ? ", " : ")");
	return (os);
}
#endif
