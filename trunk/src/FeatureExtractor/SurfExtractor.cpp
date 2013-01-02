#ifndef opensurf_files
#define opensurf_files
#include "OpenSurf/surf.cpp"
#include "OpenSurf/fasthessian.cpp"
#include "OpenSurf/integral.cpp"
#include "OpenSurf/ipoint.cpp"
#include "OpenSurf/utils.cpp"
#include "OpenSurf/surflib.h"
#include "OpenSurf/kmeans.h"
#endif

#include "SurfExtractor.h"
#include <ctime>
#include "SurfFeatureDescriptor64.h"
#include <algorithm>

SurfExtractor::SurfExtractor(){
	upright 		= true;
	octaves 		= 5;
	intervals 		= 5;
	init_sample 	= 2;
	thres			= 0.00001f;
}

SurfExtractor::~SurfExtractor(){}
using namespace std;
bool comparison_surf (KeyPoint * i,KeyPoint * j) { return (i->stabilety>j->stabilety); }

KeyPointSet * SurfExtractor::getKeyPointSet(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud){
	int width = input_cloud->width;
	int height = input_cloud->height;
	IplImage * rgb_img 			= cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	int index 					= 0;
	char * rgb_data 			= rgb_img->imageData;
	for(int j = 0; j < height; j++)
	{
		for(int i = 0; i < width; i++)
		{
			int ind = 3 * (j * width + i);
			int ind2 = j * width + i;
			rgb_data[ind + 0]=input_cloud->points[ind2].b; // B
			rgb_data[ind + 1]=input_cloud->points[ind2].g; // G
			rgb_data[ind + 2]=input_cloud->points[ind2].r; // R
			index++;
		}
	}

	IpVec ipts;
	surfDetDes(rgb_img, ipts, upright, octaves, intervals, init_sample, thres);
	printf(" %i\n",(int)ipts.size());
	
	KeyPointSet * keypoints = new KeyPointSet();
	for(int i = 0; i < (int)ipts.size(); i++)
	{
		
		Ipoint p = ipts.at(i);
		int w = int(p.x+0.5f);
		int h = int(p.y+0.5f);
		int ind = h * width + w;
		
		float * desc = new float[64];
		for(int j = 0; j < 64; j++){
			desc[j] = p.descriptor[j];
		}
		SurfFeatureDescriptor64 * descriptor = new SurfFeatureDescriptor64(desc, p.laplacian);
		pcl::PointXYZRGBNormal point = input_cloud->points[ind];
		KeyPoint * kp = new KeyPoint();
		kp->stabilety = p.stab;
		kp->descriptor = descriptor;
		kp->point = new Point(point.x,point.y,point.z,w,h);
		kp->r = point.r;
		kp->g = point.g;
		kp->b = point.b;
		if(point.z > 0 && !isnan(point.z)){
			kp->valid = true;
			kp->index_number = keypoints->valid_key_points.size();
			keypoints->valid_key_points.push_back(kp);
		}else{
			kp->valid = false;
			kp->index_number = keypoints->invalid_key_points.size();
			keypoints->invalid_key_points.push_back(kp);
		}
	}
	
	sort(keypoints->valid_key_points.begin(),keypoints->valid_key_points.end(),comparison_surf);
	sort(keypoints->invalid_key_points.begin(),keypoints->invalid_key_points.end(),comparison_surf);
	cvReleaseImage( &rgb_img );
	
	return keypoints;
}

KeyPointSet * SurfExtractor::getKeyPointSet(IplImage * rgb_img,IplImage * depth_img){

	float d_scaleing	= calibration->ds/calibration->scale;
	float centerX		= calibration->cx;
	float centerY		= calibration->cy;
	float invFocalX		= 1.0f/calibration->fx;
    float invFocalY		= 1.0f/calibration->fy;
    
	char * rgb_data		= rgb_img->imageData;
	unsigned short * depth_data	= (unsigned short *)depth_img->imageData;

	IpVec ipts;
	surfDetDes(rgb_img, ipts, upright, octaves, intervals, init_sample, thres);
	printf("OpenSURF found: %i interest points\n",(int)ipts.size());
	
	KeyPointSet * keypoints = new KeyPointSet();
	for(int i = 0; i < (int)ipts.size(); i++)
	{
		
		Ipoint p = ipts.at(i);
		int w = int(p.x+0.5f);
		int h = int(p.y+0.5f);
		int ind = h * 640 + w;
		//printf("%i:w,h: %i %i\n",i,w,h);
		float * desc = new float[64];
		for(int j = 0; j < 64; j++){
			desc[j] = p.descriptor[j];
		}
		SurfFeatureDescriptor64 * descriptor = new SurfFeatureDescriptor64(desc, p.laplacian);

		int r = 0;//char(rgb_data[3*ind+2]);
		int g = 0;//char(rgb_data[3*ind+1]);
		int b = 0;//char(rgb_data[3*ind+0]);
		//unsigned int d = 0;
		//unsigned short(depth_data[ind]);
		if(r < 0){r = 255+r;}
		if(g < 0){g = 255+g;}
		if(b < 0){b = 255+b;}
	
		float x = 0;
		float y = 0;
		float z = float(depth_data[ind]) * d_scaleing;

		if(z > 0){
			x = (w - centerX) * z * invFocalX;
           	y = (h - centerY) * z * invFocalY;
		}

		//printf("%i %i --> %i %i %i --> %.3f %.3f %.3f\n",w,h,r,g,b,x,y,z);
		
		KeyPoint * kp = new KeyPoint();
		kp->stabilety = p.stab;
		kp->descriptor = descriptor;
		kp->point = new Point(x,y,z,w,h);
		kp->r = r;
		kp->g = g;
		kp->b = b;
		if(z > 0 && !isnan(z)){
			kp->valid = true;
			kp->index_number = keypoints->valid_key_points.size();
			keypoints->valid_key_points.push_back(kp);
			//cvCircle(rgb_img, cvPoint(w, h), 5, cvScalar(0, 255, 0, 0), 2, 8, 0);
		}else{
			kp->valid = false;
			kp->index_number = keypoints->invalid_key_points.size();
			keypoints->invalid_key_points.push_back(kp);
			//cvCircle(rgb_img, cvPoint(w, h), 5, cvScalar(0, 0, 255, 0), 2, 8, 0);
		}
	}
	printf("pre sort\n");
	sort(keypoints->valid_key_points.begin(),keypoints->valid_key_points.end(),comparison_surf);
	sort(keypoints->invalid_key_points.begin(),keypoints->invalid_key_points.end(),comparison_surf);
	//cvReleaseImage( &rgb_img );
	
	return keypoints;
}
