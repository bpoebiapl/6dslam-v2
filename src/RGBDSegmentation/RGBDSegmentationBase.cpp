#include "RGBDSegmentationBase.h"
#include "mygeometry/mygeometry.h"
using namespace std;

RGBDSegmentationBase::RGBDSegmentationBase(){}
RGBDSegmentationBase::~RGBDSegmentationBase(){}
vector<Plane * > * RGBDSegmentationBase::segment(IplImage * rgb_img,IplImage * depth_img){
	struct timeval start, end;
	gettimeofday(&start, NULL);
	vector<Plane * > * planes = new vector<Plane * >();
	
	int width = rgb_img->width;
	int height = rgb_img->height;
	float ** x = new float*[width];
	float ** y = new float*[width];
	float ** z = new float*[width];
	int ** gray = new int*[width];
	float ** edge = new float*[width];
	int ** segment_id = new int*[width];
	for(int i = 0; i < width; i++){
		x[i] 	= new float[height];
		y[i] 	= new float[height];
		z[i] 	= new float[height];
		gray[i] = new int[height];
		edge[i] = new float[height];
		segment_id[i] = new int[height];
	}

	float d_scaleing	= calibration->ds/calibration->scale;
	float centerX		= calibration->cx;
	float centerY		= calibration->cy;
	float invFocalX	= 1.0f/calibration->fx;
    float invFocalY	= 1.0f/calibration->fy;
	
	char * rgb_data		= (char *)rgb_img->imageData;
	unsigned short * depth_data	= (unsigned short *)depth_img->imageData;
	
	for(int i = 0; i < width; i++){
		for(int j = 0; j < height; j++){
			int ind = width*j+i;
			int tmp_r = char(rgb_data[3*ind+2]);
			int tmp_g = char(rgb_data[3*ind+1]);
			int tmp_b = char(rgb_data[3*ind+0]);

			if(tmp_r < 0){tmp_r = 255+tmp_r;}
			if(tmp_g < 0){tmp_g = 255+tmp_g;}
			if(tmp_b < 0){tmp_b = 255+tmp_b;}
			
			float tmp_z = float(depth_data[ind]) * d_scaleing;
			float tmp_x = 0;
			float tmp_y = 0;

			if(tmp_z > 0){
				tmp_x = (i - centerX) * tmp_z * invFocalX;
		       	tmp_y = (j - centerY) * tmp_z * invFocalY;
			}
			//points[i][j] = new Point(tmp_x, tmp_y, tmp_z, i,j);
			x[i][j] = tmp_x;
			y[i][j] = tmp_y;
			z[i][j] = tmp_z;
			gray[i][j] = tmp_r+tmp_g+tmp_b;
			edge[i][j] = 1000000000;
			segment_id[i][j] = -1;
		}
	}

	float gray_mul = 1.0f/15000.0f;
	float z_mul = 0.01;
	for(int i = 1; i < width-1; i++){
		for(int j = 1; j < height-1; j++){
			float zw_edge = (z[i-1][j+1]+2*z[i][j+1]+z[i+1][j+1])-(z[i-1][j-1]+2*z[i][j-1]+z[i+1][j-1]);
			float zh_edge = (z[i+1][j+1]+2*z[i+1][j]+z[i+1][j-1])-(z[i-1][j+1]+2*z[i-1][j]+z[i-1][j-1]);
			
			int gw_edge = (gray[i-1][j+1]+2*gray[i][j+1]+gray[i+1][j+1])-(gray[i-1][j-1]+2*gray[i][j-1]+gray[i+1][j-1]);
			int gh_edge = (gray[i+1][j+1]+2*gray[i+1][j]+gray[i+1][j-1])-(gray[i-1][j+1]+2*gray[i-1][j]+gray[i-1][j-1]);
			
			edge[i][j] = float(gw_edge*gw_edge+gh_edge*gh_edge)*gray_mul + (zw_edge*zw_edge+zh_edge*zh_edge)*z_mul;
		}
	}
	
	int step = 4;
	int size = 5;
	IplImage * img_clone;
	for(int runs = 0; runs < 10; runs++){
		float best_score = 0;
		float best_score_scale = 0;
		Plane * best_plane = 0;
	
		int counter = 0;
		for(int i = step; i <= width-step; i+=step){
			for(int j = step; j <= height-step; j+=step){
				if(z[i][j]!=0){
					counter++;
					vector<float > * seg_x = new vector<float >();
					vector<float > * seg_y = new vector<float >();
					vector<float > * seg_z = new vector<float >();
			
					seg_x->push_back(x[i][j]);
					seg_y->push_back(y[i][j]);
					seg_z->push_back(z[i][j]);
			
					int interval = 5;
					float thresh = 4;
					int combinations = 1;
					float score = 1;
					for(int ik = -combinations; ik <= combinations; ik++){
						for(int jk = -combinations; jk <= combinations; jk++){
							if((ik!=0)||(jk!=0)){
								int k = 0;
								float sum = 0;
								while(true){
									k++;
									int w = i+ik*k;int h=j+jk*k;
									if(w > interval && w < width-interval && h > interval && h < height-interval){
										sum 		+= edge[w][h];
										int w_internal = w-ik*interval;
										int h_internal = h-jk*interval;
										if(k > interval){sum -= edge[w_internal][h_internal];}
										if(sum >= thresh || z[w][h]==0){
											seg_x->push_back(x[w_internal][h_internal]);
											seg_y->push_back(y[w_internal][h_internal]);
											seg_z->push_back(z[w_internal][h_internal]);
											break;
										}
									}else{break;}
								}
								score*=k;
							}
						}
					}
					if(score > best_score){
						Plane * p = new Plane(seg_x,seg_y,seg_z,0,0);
						float score_scale = score / (p->weight+0.000000001);
						if(best_score_scale < score_scale){
							best_score = score;
							best_score_scale = score_scale;
							best_plane = p;
						}else{delete p;}
					}else{
						delete seg_x;
						delete seg_y;
						delete seg_z;
					}
				}
			}
		}
		if(best_plane == 0){break;}
		float threshold_now = 0.015f;
		int inliers = 1;
		int total_datapoints = 0;
		int inliers_last = 0;
	
	
		float last_normal_x = 0;
		float last_normal_y = 0;
		float last_normal_z = 0;
		float last_point_x = 0;
		float last_point_y = 0;
		float last_point_z = 0;
	
		float normal_x = best_plane->normal_x;
		float normal_y = best_plane->normal_y;
		float normal_z = best_plane->normal_z;
		float point_x = best_plane->point_x;
		float point_y = best_plane->point_y;
		float point_z = best_plane->point_z;
	
		for(int improvement = 0; improvement < 30 /*&& !(normal_x == last_normal_x && normal_y == last_normal_y && normal_z == last_normal_z && point_x == last_point_x && point_y == last_point_y && point_z == last_point_z)*/; improvement++){
			printf("improvement: %i\n",improvement);
			img_clone = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
			cvCopy( rgb_img, img_clone, NULL );
			char * cdata 			= (char *)(img_clone->imageData);

			last_normal_x = normal_x;
			last_normal_y = normal_y;
			last_normal_z = normal_z;
			last_point_x = point_x;
			last_point_y = point_y;
			last_point_z = point_z;
	
			vector<float > * s_x = new vector<float >();
			vector<float > * s_y = new vector<float >();
			vector<float > * s_z = new vector<float >();

			vector<float > d_vec;
			inliers_last = inliers;
			inliers = 0;
			total_datapoints = 0;

			for(int i = 0; i < width-0; i+=1){
				for(int j = 0; j < height-0; j+=1){
					if( z[i][j] != 0){
						total_datapoints++;
						float d = fabs(normal_x*(point_x-x[i][j]) + normal_y*(point_y-y[i][j]) + normal_z*(point_z-z[i][j]));
						if(d < threshold_now){
							inliers++;
							d_vec.push_back(d);
							s_x->push_back(x[i][j]);
							s_y->push_back(y[i][j]);
							s_z->push_back(z[i][j]);
							int ind = width*j+i;
							cdata[3*ind+0] = 255;
							cdata[3*ind+1] = 0;
							cdata[3*ind+2] = 255;
						}
					}
				}
			}
			
			float std_div = 0;
			for(int i = 0; i < d_vec.size(); i++){std_div += d_vec.at(i)*d_vec.at(i);}
			std_div = sqrt(std_div/float(d_vec.size()));

			Plane * best_plane_update = new Plane(s_x,s_y,s_z,0,0);
			delete s_x;
			delete s_y;
			delete s_z;
			normal_x = best_plane_update->normal_x;
			normal_y = best_plane_update->normal_y;
			if(best_plane_update->normal_z > 0){best_plane_update->normal_z*=-1;}
			normal_z = best_plane_update->normal_z;
			point_x = best_plane_update->point_x;
			point_y = best_plane_update->point_y;
			point_z = best_plane_update->point_z;
			delete best_plane;
			best_plane = best_plane_update;
			
			cvNamedWindow("segments", CV_WINDOW_AUTOSIZE );
			cvShowImage("segments", img_clone);
			cvWaitKey(0);
			cvReleaseImage( &img_clone );
		}
		if(inliers < 3){
			delete best_plane;
			break;
		}
		normal_x = best_plane->normal_x;
		normal_y = best_plane->normal_y;
		normal_z = best_plane->normal_z;
		point_x = best_plane->point_x;
		point_y = best_plane->point_y;
		point_z = best_plane->point_z;
		
		for(int i = 0; i < width-0; i+=1){
			for(int j = 0; j < height-0; j+=1){
				if(z[i][j] != 0 && fabs(normal_x*(point_x-x[i][j]) + normal_y*(point_y-y[i][j]) + normal_z*(point_z-z[i][j])) < 2*threshold_now){z[i][j] = 0;}
			}
		}
	
		printf("threshold_now %f && %i > %f && %i > %f\n", threshold_now,inliers,0.2f*total_datapoints,inliers,0.02f*width*height);
		if(threshold_now < 0.2 && inliers > 0.2f*total_datapoints && inliers > 0.02f*width*height){printf("good plane...\n");planes->push_back(best_plane);}
		if(total_datapoints < 0.02f*float(width*height)){break;}
	}
	for(int i = 0; i < width; i++){
		delete[] x[i];
		delete[] y[i];
		delete[] z[i];
		delete[] gray[i];
		delete[] edge[i];
		delete[] segment_id[i];
	}
	delete[] x;
	delete[] y;
	delete[] z;
	delete[] gray;
	delete[] edge;
	delete[] segment_id;
	
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("Segment cost: %f\n",time);
	return planes;
}
