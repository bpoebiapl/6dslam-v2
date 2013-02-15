#include "RGBDSegmentationBase.h"
#include "mygeometry/mygeometry.h"
using namespace std;

RGBDSegmentationBase::RGBDSegmentationBase(){}
RGBDSegmentationBase::~RGBDSegmentationBase(){}
vector<Plane * > * RGBDSegmentationBase::segment(IplImage * rgb_img,IplImage * depth_img){

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

	struct timeval start, end;
	gettimeofday(&start, NULL);
	vector<Plane * > * planes = new vector<Plane * >();
	
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
	for(int i = 1; i < width-1; i++){
		for(int j = 1; j < height-1; j++){
			float zw_edge = 0;
			//if(z[i-1][j+1]  != 0 && z[i][j+1]  != 0 && z[i+1][j+1] != 0 && z[i-1][j-1] != 0 && z[i][j-1] != 0 && z[i+1][j-1] != 0){
				zw_edge = (z[i-1][j+1]+2*z[i][j+1]+z[i+1][j+1])-(z[i-1][j-1]+2*z[i][j-1]+z[i+1][j-1]);
			//}
			float zh_edge = 0;
			//if(z[i-1][j+1] != 0 &&z[i][j+1] != 0 &&z[i+1][j+1] && z[i-1][j-1] != 0 &&z[i][j-1] != 0 &&z[i+1][j-1]){
				zh_edge = (z[i+1][j+1]+2*z[i+1][j]+z[i+1][j-1])-(z[i-1][j+1]+2*z[i-1][j]+z[i-1][j-1]);
			//}
			
			int gw_edge = (gray[i-1][j+1]+2*gray[i][j+1]+gray[i+1][j+1])-(gray[i-1][j-1]+2*gray[i][j-1]+gray[i+1][j-1]);
			int gh_edge = (gray[i+1][j+1]+2*gray[i+1][j]+gray[i+1][j-1])-(gray[i-1][j+1]+2*gray[i-1][j]+gray[i-1][j-1]);
			
			edge[i][j] = float(gw_edge*gw_edge+gh_edge*gh_edge)*gray_mul + (zw_edge*zw_edge+zh_edge*zh_edge)*0.01;
		}
	}


	int current_todo = 0;
	int max_todo = 0;
	int * w_todo = new int[width*height];
	int * h_todo = new int[width*height];
	vector<vector<int > * > * segs_w = new vector<vector<int > * >();
	vector<vector<int > * > * segs_h = new vector<vector<int > * >();
	
	int current_id = 0;
	int step = 5;
	for(float scale = 256; scale >= 4; scale /= 2){
		int size = step*scale;
		for(int i = int(step*scale); i < width-int(step*scale); i+=int(step*scale)){
			for(int j = int(step*scale); j < height-int(step*scale); j+=int(step*scale)){
				vector<float > * s_x = new vector<float >();
				vector<float > * s_y = new vector<float >();
				vector<float > * s_z = new vector<float >();
			
				for(int ii = -step; ii <= step; ii++){
					for(int jj = -step; jj <= step; jj++){
						int iiw = i+ii*int(scale);
						int jjh = j+jj*int(scale);
						float z_tmp = z[iiw][jjh];
						if(z_tmp > 0.1f){
							s_x->push_back(x[iiw][jjh]);
							s_y->push_back(y[iiw][jjh]);
							s_z->push_back(z_tmp);
						}
					}
				}
			
				Plane * pl = new Plane(s_x,s_y,s_z,0,0);
				int c = s_x->size();
				delete s_x;
				delete s_y;
				delete s_z;

				if(pl->weight < 0.00001 && (c * 3 > step*step)){
					
					printf("pl->weight: %f\n",pl->weight); 
					for(int iter = 0; iter < 7; iter++){
						float normal_x = pl->normal_x;
						float normal_y = pl->normal_y;
						float normal_z = pl->normal_z;
						float point_x = pl->point_x;
						float point_y = pl->point_y;
						float point_z = pl->point_z;
						
						delete pl;
						s_x = new vector<float >();
						s_y = new vector<float >();
						s_z = new vector<float >();
						int counter = 0;
						for(int ii = 0; ii < width; ii++){
							for(int jj = 0; jj < height; jj++){
								if(z[ii][jj] != 0 && fabs(normal_x*(point_x-x[ii][jj]) + normal_y*(point_y-y[ii][jj]) + normal_z*(point_z-z[ii][jj])) < 0.005){
									s_x->push_back(x[ii][jj]);
									s_y->push_back(y[ii][jj]);
									s_z->push_back(z[ii][jj]);
									counter++;
								}
							}
						}
						pl = new Plane(s_x,s_y,s_z,0,0);
						delete s_x;
						delete s_y;
						delete s_z;
					}
					
					/*
					IplImage * img_clone = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
					cvCopy( rgb_img, img_clone, NULL );
					cvRectangle(img_clone,cvPoint(i-size, j-size),cvPoint(i+size, j+size),cvScalar(255, 0, 0, 0),1, 8, 0);
					char * data 			= (char *)(img_clone->imageData);
					*/
					float normal_x = pl->normal_x;
					float normal_y = pl->normal_y;
					float normal_z = pl->normal_z;
					float point_x = pl->point_x;
					float point_y = pl->point_y;
					float point_z = pl->point_z;
					for(int ii = 0; ii < width; ii++){
						for(int jj = 0; jj < height; jj++){
							if(z[ii][jj] != 0 && fabs(normal_x*(point_x-x[ii][jj]) + normal_y*(point_y-y[ii][jj]) + normal_z*(point_z-z[ii][jj])) < 0.015){
								z[ii][jj] = 0;
								/*
								data[(jj * width + ii)*3+0] = 255;
								data[(jj * width + ii)*3+1] = 0;
								data[(jj * width + ii)*3+2] = 255;
								*/
							}
						}
					}
					/*
					cvNamedWindow("segments", CV_WINDOW_AUTOSIZE );
					cvShowImage("segments", img_clone);
					cvWaitKey(0);
					cvReleaseImage( &img_clone );
					*/
					delete pl;
					
				}
			}
		}
	}
/*	
	
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("Segment cost: %f\n",time);

	float* tmp_red = new float[height*width];
	float* tmp_green = new float[height*width];
	float* tmp_blue = new float[height*width];
	for(int i = 0; i < height*width; i ++)
	{
		tmp_red[i] = float(rand()%10000)/10000.0f;
		tmp_green[i] = float(rand()%10000)/10000.0f;
		tmp_blue[i] = float(rand()%10000)/10000.0f;
		
	}
	IplImage * img 			= cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
	float * data 			= (float *)(img->imageData);
	for(int i = 0; i < width; i++)
	{
		for(int j = 0; j < height; j++)
		{
			int seg = segment_id[i][j]+1;
			data[(j * width + i)*3+0] = tmp_blue[seg];//segment[i][j]+1];
			data[(j * width + i)*3+1] = tmp_green[seg];//segment[i][j]+1];
			data[(j * width + i)*3+2] = tmp_red[seg];//segment[i][j]+1];
		}
	}
	delete[] tmp_red;
	delete[] tmp_green;
	delete[] tmp_blue;
	cvNamedWindow("segments", CV_WINDOW_AUTOSIZE );
	cvShowImage("segments", img);
	cvWaitKey(0);
	//cvReleaseImage( &img );
	*/
/*
	gettimeofday(&start, NULL);
	int step = 5;
	int size = 2*height;
	
	while(true){
		float best_score = 0;
		float best_score_scale = 0;
		Plane * best_plane = 0;
		int counter = 0;
		for(int i = step; i <= width-step; i+=step){
			for(int j = step; j <= height-step; j+=step){
			
				//IplImage * img_clone;
				//img_clone = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
				//cvCopy( rgb_img, img_clone, NULL );
				//cvRectangle(img_clone,cvPoint(i-size, j-size),cvPoint(i+size, j+size),cvScalar(255, 0, 0, 0),1, 8, 0);
				//char * cdata 			= (char *)(img_clone->imageData);
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
								if(w > 0 && w < width && h > 0 && h < height){
									sum 		+= edge[w][h];
									if(k > interval){sum -= edge[w-ik*interval][h-jk*interval];}
									if(sum<thresh && z[w][h]!=0){
										//int ind = width*h+w;
										//cdata[3*ind+0] = 255;
										//cdata[3*ind+1] = 0;
										//cdata[3*ind+2] = 255;
										//seg_x->push_back(x[w-ik*interval][h-jk*interval]);
										//seg_y->push_back(y[w-ik*interval][h-jk*interval]);
										//seg_z->push_back(z[w-ik*interval][h-jk*interval]);
									}else{
										seg_x->push_back(x[w-ik*interval][h-jk*interval]);
										seg_y->push_back(y[w-ik*interval][h-jk*interval]);
										seg_z->push_back(z[w-ik*interval][h-jk*interval]);
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
						
						//cvNamedWindow("segments", CV_WINDOW_AUTOSIZE );
						//cvShowImage("segments", img_clone);
						//cvWaitKey(0);
						
					
					}else{delete p;}
				}else{
					delete seg_x;
					delete seg_y;
					delete seg_z;
				}
				//cvReleaseImage( &img_clone );
			}
		}
	
		//IplImage * img_clone;
		//img_clone = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
		//cvCopy( rgb_img, img_clone, NULL );
		//char * cdata 			= (char *)(img_clone->imageData);
		float normal_x = best_plane->normal_x;
		float normal_y = best_plane->normal_y;
		float normal_z = best_plane->normal_z;
		float point_x = best_plane->point_x;
		float point_y = best_plane->point_y;
		float point_z = best_plane->point_z;
	
		vector<float > * s_x = new vector<float >();
		vector<float > * s_y = new vector<float >();
		vector<float > * s_z = new vector<float >();
			

		for(int i = step; i <= width-step; i+=step){
			for(int j = step; j <= height-step; j+=step){
				if(fabs(normal_x*(point_x-x[i][j]) + normal_y*(point_y-y[i][j]) + normal_z*(point_z-z[i][j])) < 0.01){

					s_x->push_back(x[i][j]);
					s_y->push_back(y[i][j]);
					s_z->push_back(z[i][j]);
					//z[i][j] = 0;
					//cvRectangle(img_clone,cvPoint(i-step/2, j-step/2),cvPoint(i+step/2, j+step/2),cvScalar(255, 0, 0, 0),-1, 8, 0);
				}
			}
		}
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
	
		int matches = 0;
		for(int i = step; i <= width-step; i+=step){
			for(int j = step; j <= height-step; j+=step){
				if(z[i][j] != 0 && fabs(normal_x*(point_x-x[i][j]) + normal_y*(point_y-y[i][j]) + normal_z*(point_z-z[i][j])) < 0.02){
					z[i][j] = 0;
					matches++;
					//cvRectangle(img_clone,cvPoint(i-step/4, j-step/4),cvPoint(i+step/4, j+step/4),cvScalar(255, 0, 0, 0),-1, 8, 0);
				}
			}
		}
		//cvNamedWindow("segments", CV_WINDOW_AUTOSIZE );
		//cvShowImage("segments", img_clone);
		//cvWaitKey(0);
		delete best_plane;
		if( matches * 10 > counter){
			printf("good plane found\n");
			planes->push_back(best_plane_update);
		}else{
			delete best_plane_update;
			break;
		}

	}
*/
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("Segment cost: %f\n",time);
	return planes;
}
