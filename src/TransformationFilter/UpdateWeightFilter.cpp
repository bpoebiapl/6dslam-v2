#include "UpdateWeightFilter.h"

using namespace std;
UpdateWeightFilter::UpdateWeightFilter(){
	limit = 0.015f;
}
UpdateWeightFilter::~UpdateWeightFilter(){}
Transformation * UpdateWeightFilter::filterTransformation(Transformation * input){
	
	struct timeval start, end;
	gettimeofday(&start, NULL);
	RGBDFrame * src = input->src;
	RGBDFrame * dst = input->dst;
	
	IplImage* depth_img = cvLoadImage(src->input->depth_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	depth_data	= (unsigned short *)depth_img->imageData;
	
	Transformation * transformation = new Transformation();
	transformation->transformationMatrix = input->transformationMatrix;
	transformation->src = src;
	transformation->dst = dst;
	
	transformation->weight = 0;
	for(int i = 0; i < input->matches.size();i++){
		KeyPoint * src_kp = input->matches.at(i).first;
		KeyPoint * dst_kp = input->matches.at(i).second;
		transformation->matches.push_back(make_pair(src_kp, dst_kp));
	}

	
	float d_scaleing	= dst->input->calibration->ds/dst->input->calibration->scale;
	float centerX		= dst->input->calibration->cx;
	float centerY		= dst->input->calibration->cy;
	float invFocalX		= 1.0f/dst->input->calibration->fx;
    float invFocalY		= 1.0f/dst->input->calibration->fy;
	Eigen::Matrix4f transformationMat2 = input->transformationMatrix;
	Eigen::Matrix4f transformationMat  = input->transformationMatrix;//Eigen::Matrix4f::Identity();//
	transformationMat = transformationMat2.inverse();
	float mat00 = transformationMat(0,0);
	float mat01 = transformationMat(0,1);
	float mat02 = transformationMat(0,2);
	float mat03 = transformationMat(0,3);
	float mat10 = transformationMat(1,0);
	float mat11 = transformationMat(1,1);
	float mat12 = transformationMat(1,2);
	float mat13 = transformationMat(1,3);
	float mat20 = transformationMat(2,0);
	float mat21 = transformationMat(2,1);
	float mat22 = transformationMat(2,2);
	float mat23 = transformationMat(2,3);
	float counter_good = 0;
	float counter_bad = 0;
	float counter_total = 0;
	for(int i = 0; i < dst->validation_points.size(); i++){
		float * vp = dst->validation_points.at(i);
	
		float x_tmp = vp[0];
		float y_tmp = vp[1];
		float z_tmp = vp[2];
				
		float x = x_tmp*mat00+y_tmp*mat01+z_tmp*mat02+mat03;
		float y = x_tmp*mat10+y_tmp*mat11+z_tmp*mat12+mat13;
		float z = x_tmp*mat20+y_tmp*mat21+z_tmp*mat22+mat23;
		
		//int d_data 	= int(0.5f+z/d_scaleing);
		int w 		= int(0.5f+x/(z * invFocalX) + centerX);
		int h 		= int(0.5f+y/(z * invFocalY) + centerY);
		if(w>=0 && w < 640 && h >= 0 && h < 480){
			float z_img = int(depth_data[640*h+w])*d_scaleing;
			if(z_img > 0.01){
				float diff = z-z_img;
				if(fabs(diff) < limit)		{counter_good++;}
				else if(z+0.075f < z_img)	{counter_bad++;}
				counter_total++;
			}
		}
	}
	if(counter_total < 1000){counter_total = 1000;}
	transformation->weight = 3 + (counter_good-3*counter_bad)/counter_total;
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	//printf("update weight results: %f %f %f-> %f\n",counter_good,counter_bad,counter_total,transformation->weight);
	cvReleaseImage( &depth_img );
	//printf("UpdateWeightFilter cost: %f\n",time);

	return transformation;
}
void UpdateWeightFilter::print(){printf("%s\n",name.c_str());}
void UpdateWeightFilter::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}
