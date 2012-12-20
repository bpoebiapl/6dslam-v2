#include "FPFHFeatureDescriptor.h"

FPFHFeatureDescriptor::FPFHFeatureDescriptor(){};

FPFHFeatureDescriptor::~FPFHFeatureDescriptor(){};

void FPFHFeatureDescriptor::print(){}

inline FPFHFeatureDescriptor * FPFHFeatureDescriptor::clone(){
	return new FPFHFeatureDescriptor();
}

double FPFHFeatureDescriptor::distance(FPFHFeatureDescriptor * other_descriptor)
{
	float * disc 	= other_descriptor->feature.histogram;
	float * descriptor 	= feature.histogram;
	
	float tmp0 		= descriptor[0] - disc[0];
	float tmp1 		= descriptor[1] - disc[1];
	float tmp2 		= descriptor[2] - disc[2];
	float tmp3 		= descriptor[3] - disc[3];
	float tmp4 		= descriptor[4] - disc[4];
	float tmp5 		= descriptor[5] - disc[5];
	float tmp6 		= descriptor[6] - disc[6];
	float tmp7 		= descriptor[7] - disc[7];
	float tmp8 		= descriptor[8] - disc[8];
	float tmp9 		= descriptor[9] - disc[9];
		
	float tmp10 	= descriptor[10] - disc[10];
	float tmp11 	= descriptor[11] - disc[11];
	float tmp12 	= descriptor[12] - disc[12];
	float tmp13 	= descriptor[13] - disc[13];
	float tmp14 	= descriptor[14] - disc[14];
	float tmp15 	= descriptor[15] - disc[15];
	float tmp16 	= descriptor[16] - disc[16];
	float tmp17 	= descriptor[17] - disc[17];
	float tmp18 	= descriptor[18] - disc[18];
	float tmp19 	= descriptor[19] - disc[19];
			
	float tmp20 	= descriptor[20] - disc[20];
	float tmp21 	= descriptor[21] - disc[21];
	float tmp22 	= descriptor[22] - disc[22];
	float tmp23 	= descriptor[23] - disc[23];
	float tmp24 	= descriptor[24] - disc[24];
	float tmp25 	= descriptor[25] - disc[25];
	float tmp26 	= descriptor[26] - disc[26];
	float tmp27 	= descriptor[27] - disc[27];
	float tmp28 	= descriptor[28] - disc[28];
	float tmp29 	= descriptor[29] - disc[29];
			
	float tmp30 	= descriptor[30] - disc[30];
	float tmp31 	= descriptor[31] - disc[31];
	float tmp32 	= descriptor[32] - disc[32];
	
	float dist 		= tmp0*tmp0 + tmp1*tmp1 + tmp2*tmp2 + tmp3*tmp3 + tmp4*tmp4 + tmp5*tmp5 + tmp6*tmp6 + tmp7*tmp7 + tmp8*tmp8 + tmp9*tmp9 + tmp10*tmp10 + tmp11*tmp11 + tmp12*tmp12 + tmp13*tmp13 + tmp14*tmp14 + tmp15*tmp15 + tmp16*tmp16 + tmp17*tmp17 + tmp18*tmp18 + tmp19*tmp19 + tmp20*tmp20 + tmp21*tmp21 + tmp22*tmp22 + tmp23*tmp23 + tmp24*tmp24 + tmp25*tmp25 + tmp26*tmp26 + tmp27*tmp27 + tmp28*tmp28 + tmp29*tmp29 + tmp30*tmp30 + tmp31*tmp31 + tmp32*tmp32;
	return (dist);
}
