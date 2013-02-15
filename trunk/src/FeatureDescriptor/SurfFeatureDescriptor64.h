#ifndef SurfFeatureDescriptor64_H_
#define SurfFeatureDescriptor64_H_
#include "FeatureDescriptor.h"
using namespace std;
class SurfFeatureDescriptor64 : public FeatureDescriptor
{
	public:
	float stabilety;
	int laplacian;
	
	float * descriptor;
	unsigned int descriptor_length;
	
	SurfFeatureDescriptor64();
	SurfFeatureDescriptor64(float * feature_descriptor);
	SurfFeatureDescriptor64(float * feature_descriptor, int feature_laplacian);
	SurfFeatureDescriptor64(string path);
	double distance(SurfFeatureDescriptor64 * other_descriptor);
	void print();
	void store(string path);
	void update(vector<FeatureDescriptor * > * input);
	SurfFeatureDescriptor64 * clone();
	~SurfFeatureDescriptor64();
};

inline ostream& operator << (ostream& os, const SurfFeatureDescriptor64& p)
{
	for (int i = 0; i < 64; ++i)
		os << (i == 0 ? "(" : "") << p.descriptor[i] << (i < 63 ? ", " : ")");
	return (os);
}
#endif
