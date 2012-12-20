#ifndef RGBFeatureDescriptor_H_
#define RGBFeatureDescriptor_H_
#include "FeatureDescriptor.h"
using namespace std;
class RGBFeatureDescriptor : public FeatureDescriptor
{
	public:
		RGBFeatureDescriptor(float r, float g, float b);
};
#endif
