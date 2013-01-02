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

#include "OwnFeatureExtractor.h"
#include <ctime>
#include "SurfFeatureDescriptor64.h"
#include <algorithm>

OwnFeatureExtractor::OwnFeatureExtractor(){
	max_kps = 15000;
	max_dist = 3;
	histogram_bins = 300;
	
	currentDistances = new float*[max_kps];
	for(int i = 0; i < max_kps; i++){currentDistances[i] = new float[max_kps];}
}

OwnFeatureExtractor::~OwnFeatureExtractor(){}
using namespace std;

bool comparison_own (KeyPoint * i,KeyPoint * j) { return (i->stabilety>j->stabilety); }

KeyPointSet * OwnFeatureExtractor::getKeyPointSet(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud){
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

	KeyPointSet * keypoints = new KeyPointSet();
	/*
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
	*/
	//sort(keypoints->valid_key_points.begin(),keypoints->valid_key_points.end(),comparison_f);
	//sort(keypoints->invalid_key_points.begin(),keypoints->invalid_key_points.end(),comparison_f);
	cvReleaseImage( &rgb_img );
	
	return keypoints;
}

KeyPointSet * OwnFeatureExtractor::getKeyPointSet(IplImage * rgb_img,IplImage * depth_img){
	struct timeval start, end;
	gettimeofday(&start, NULL);
	
	float d_scaleing	= calibration->ds/calibration->scale;
	float centerX		= calibration->cx;
	float centerY		= calibration->cy;
	float invFocalX		= 1.0f/calibration->fx;
    float invFocalY		= 1.0f/calibration->fy;
    
	char * rgb_data		= rgb_img->imageData;
	unsigned short * depth_data	= (unsigned short *)depth_img->imageData;
	

	cv::Mat img(rgb_img);
	cv::Mat d_img(depth_img);
	
	cv::Mat desc1;
	vector <cv::KeyPoint> kp1;
	
	cv::Mat gray;
	cv::cvtColor(img, gray, CV_BGR2GRAY);  
	cv::FAST(gray, kp1,50);
	if(kp1.size() < 150){cv::FAST(gray, kp1, 35);}
	if(kp1.size() < 150){cv::FAST(gray, kp1, 25);}
	
	//cv::ORB orb = cv::ORB(2000,1.2f, 8, 10, 0,2, cv::ORB::HARRIS_SCORE, 31);
	//orb(img, cv::Mat(), kp1, desc1);
	
	//cv::MSER mser = cv::MSER();
	//mser()

	KeyPointSet * keypoints = new KeyPointSet();
	for(int i = 0; i < kp1.size();i++)
	{
		//printf("%i ->%f %f -> %f %f\n",kp1.at(i).pt.x,kp1.at(i).pt.y,kp1.at(i).size,kp1.at(i).angle);
		cv::KeyPoint curr = kp1.at(i);
		
		int w = curr.pt.x+0.5;
		int h = curr.pt.y+0.5;
		//printf("%i:%f\n",i,curr.response);
		
		int ind = 640*h+w;
		int r = char(rgb_data[3*ind+2]);
		int g = char(rgb_data[3*ind+1]);
		int b = char(rgb_data[3*ind+0]);

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
		
		KeyPoint * kp = new KeyPoint();
		kp->stabilety = curr.response;
		kp->descriptor = 0;
		kp->point = new Point(x,y,z,w,h);
		kp->r = r;
		kp->g = g;
		kp->b = b;
		if(z > 0 && !isnan(z)){
			kp->valid = true;
			kp->index_number = keypoints->valid_key_points.size();
			keypoints->valid_key_points.push_back(kp);
			//cvCircle(rgb_img, cvPoint(w, h), 5, cvScalar(0, 255, 0, 0), 2, 8, 0);
		}
	}
	printf("kp1: %i, valid: %i\n",int(kp1.size()),keypoints->valid_key_points.size());
	int kps_to_do = max_kps;
	if(keypoints->valid_key_points.size() < kps_to_do){kps_to_do = keypoints->valid_key_points.size();}
	float invMaxDist = 1/max_dist;
	float ** histograms = new float*[kps_to_do];
	
	for(int i = 0; i < kps_to_do;i++){
		histograms[i] = new float[histogram_bins];
		for(int j = 0; j < histogram_bins; j++){
			histograms[i][j] = 0;
		}
		float x1 = keypoints->valid_key_points.at(i)->point->x;
		float y1 = keypoints->valid_key_points.at(i)->point->y;
		float z1 = keypoints->valid_key_points.at(i)->point->z;
		for(int j = 0; j < i; j++){
			Point * p = keypoints->valid_key_points.at(j)->point;
			float dx = p->x-x1;
			float dy = p->y-y1;
			float dz = p->z-z1;
			float bin = sqrt(dx*dx+dy*dy+dz*dz)*invMaxDist*histogram_bins;
			int bin1 = bin;
			int bin2 = bin+1;
			float part1 = bin-int(bin);
			float part2 = 1-part1;
			if(bin2 < histogram_bins){
				histograms[i][bin1]+=part1;
				histograms[i][bin2]+=part2;
				histograms[j][bin1]+=part1;
				histograms[j][bin2]+=part2;
			}
		}
	}
	
	for(int i = 0; i < kps_to_do;i++){
		for(int j = 0; j < histogram_bins; j++){histograms[i][j] /= float(kps_to_do);}
		keypoints->valid_key_points.at(i)->descriptor = new FloatHistogramFeatureDescriptor(histograms[i], histogram_bins);
		//keypoints->valid_key_points.at(i)->descriptor->print();
	}
	//printf("w:h %i %i\n",desc1.rows,desc1.cols);	

	//sort(keypoints->valid_key_points.begin(),keypoints->valid_key_points.end(),comparison_f);
	//sort(keypoints->invalid_key_points.begin(),keypoints->invalid_key_points.end(),comparison_f);
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("Own cost: %f\n",time);
	return keypoints;
}
