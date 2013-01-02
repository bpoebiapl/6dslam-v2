#include "OrbExtractor.h"
#include <ctime>
#include "SurfFeatureDescriptor64.h"
#include <algorithm>
#include "cv.h"
#include "highgui.h"
#include <opencv.hpp>

OrbExtractor::OrbExtractor(){ nr_features = 1000;}

OrbExtractor::~OrbExtractor(){}
using namespace std;

bool comparison_orb (KeyPoint * i,KeyPoint * j) { return (i->stabilety>j->stabilety); }

KeyPointSet * OrbExtractor::getKeyPointSet(IplImage * rgb_img,IplImage * depth_img){
	KeyPointSet * keypoints = new KeyPointSet();
	
	struct timeval start, end;
	gettimeofday(&start, NULL);
	
	cv::ORB orb = cv::ORB(nr_features,1.2f, 8, 5, 0,2, cv::ORB::HARRIS_SCORE, 31);
	cv::Mat img(rgb_img);
	cv::Mat d_img(depth_img);
	
	char * rgb_data		= (char *)rgb_img->imageData;
	unsigned short * depth_data	= (unsigned short *)depth_img->imageData;
	
	cv::Mat desc1;
	vector <cv::KeyPoint> kp1;
	orb(img, cv::Mat(), kp1, desc1);
	
	float d_scaleing	= calibration->ds/calibration->scale;
	float centerX		= calibration->cx;
	float centerY		= calibration->cy;
	float invFocalX		= 1.0f/calibration->fx;
    float invFocalY		= 1.0f/calibration->fy;
		
	for(int i = 0; i < kp1.size();i++)
	{
		//printf("%i ->%f %f -> %f %f\n",kp1.at(i).pt.x,kp1.at(i).pt.y,kp1.at(i).size,kp1.at(i).angle);
		cv::KeyPoint curr = kp1.at(i);
		int * desc = new int[32];
		for(int j = 0; j < 32; j++){
			desc[j] = (int)desc1.at<uchar>(i,j);
		}
		FeatureDescriptor * descriptor = new OrbFeatureDescriptor(desc);
		
		
		int w = curr.pt.x+0.5;
		int h = curr.pt.y+0.5;
		
		int ind = 640*h+w;
		int r = char(rgb_data[3*ind+2]);
		int g = char(rgb_data[3*ind+1]);
		int b = char(rgb_data[3*ind+0]);
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
		kp->stabilety = curr.response;
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
	
	sort(keypoints->valid_key_points.begin(),keypoints->valid_key_points.end(),comparison_orb);
	sort(keypoints->invalid_key_points.begin(),keypoints->invalid_key_points.end(),comparison_orb);
	
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	//if(keypoints->invalid_key_points.size() < 10){printf("too few keypoints\n");exit(0);}
	printf("Orb cost: %f\n",time);
	
	//cv::Mat output_img;
	//drawKeypoints(img, kp1, output_img);

	//cv::namedWindow("Image");
	//cv::imshow("Image", output_img);
	//cv::waitKey(100);
	
	return keypoints;
}
