#include "RGBDSegmentationScaleSearch.h"
#include "mygeometry/mygeometry.h"
using namespace std;

RGBDSegmentationScaleSearch::RGBDSegmentationScaleSearch(){}
RGBDSegmentationScaleSearch::~RGBDSegmentationScaleSearch(){}
vector<Plane * > * RGBDSegmentationScaleSearch::segment(IplImage * rgb_img,IplImage * depth_img){
	struct timeval start, end;
	gettimeofday(&start, NULL);
	vector<Plane * > * planes = new vector<Plane * >();
	
	int width = rgb_img->width;
	int height = rgb_img->height;
	float ** x = new float*[width];
	float ** y = new float*[width];
	float ** z = new float*[width];
	int ** segment_id = new int*[width];
	for(int i = 0; i < width; i++){
		x[i] 	= new float[height];
		y[i] 	= new float[height];
		z[i] 	= new float[height];
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
			float tmp_z = float(depth_data[ind]) * d_scaleing;
			float tmp_x = 0;
			float tmp_y = 0;

			if(tmp_z > 0){
				tmp_x = (i - centerX) * tmp_z * invFocalX;
		       	tmp_y = (j - centerY) * tmp_z * invFocalY;
			}
			x[i][j] = tmp_x;
			y[i][j] = tmp_y;
			z[i][j] = tmp_z;
			segment_id[i][j] = -1;
		}
	}
	int start_size = 128;
	int stop_size = 16;
	for(int size = start_size; size >=stop_size; size/=2){
		printf("size:%i\n",size);
		for(int w = size; w+size < width; w+=size/2){
			for(int h = size; h+size < height; h+=size/2){
				if(z[w][h] != 0){
					vector<float > * s_x = new vector<float >();
					vector<float > * s_y = new vector<float >();
					vector<float > * s_z = new vector<float >();
					vector<int > * s_w = new vector<int >();
					vector<int > * s_h = new vector<int >();

					for(int i = w-size; i <= w+size; i+=size/4){
						for(int j = h-size; j <= h+size; j+=size/4){
							if(z[i][j] != 0){
								s_x->push_back(x[i][j]);
								s_y->push_back(y[i][j]);
								s_z->push_back(z[i][j]);
								s_w->push_back(i);
								s_h->push_back(j);
							}
						}
					}
					if(s_x->size() >= 3){
						Plane * p = new Plane(s_x,s_y,s_z,s_w,s_h);

						if( p->normal_z > 0){
							p->normal_x*=-1;
							p->normal_y*=-1;
							p->normal_z*=-1;
						}
				
						bool good_plane = true;
						//printf("--------------------------------------------------------------------\n");
						int max_refinement = 15;
						vector<int> speedup_step;
						speedup_step.push_back(4);
						speedup_step.push_back(3);
						speedup_step.push_back(2);
						while(speedup_step.size()<=max_refinement){
							speedup_step.push_back(1);
						}
						float angle_change = 1;
						for(int refinement = 0; refinement <= max_refinement && good_plane && angle_change > 0.000001f; refinement++){
							//printf("p->weight: %f\n",p->weight);

							if(p->weight < 0.000100){
								IplImage * img_clone;
								bool display_here = false;
								if(display_here){
									img_clone = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
									cvCopy( rgb_img, img_clone, NULL );
								}
								
								s_x->clear();
								s_y->clear();
								s_z->clear();
								s_w->clear();
								s_h->clear();
								//printf("d%i=[",refinement);
								int current_step = speedup_step.at(refinement);
								for(int i = 0; i < width; i+=current_step){
									for(int j = 0; j < height; j+=current_step){
										if(z[i][j] != 0){
											float d = (p->distance(x[i][j],y[i][j],z[i][j]));
											//if(fabs(d) < 0.1){printf("%.4f ",d);}
											if(fabs(d) < 0.02){
												if(display_here){cvRectangle(img_clone,cvPoint(i, j),cvPoint(i, j),cvScalar(255, 0, 255, 0), 1 , 8, 0);}
												s_x->push_back(x[i][j]);
												s_y->push_back(y[i][j]);
												s_z->push_back(z[i][j]);
												s_w->push_back(i);
												s_h->push_back(j);
											}
										}
									}
								}
								//printf("];\n\n\n");
								if(display_here){
									cvNamedWindow("segments", CV_WINDOW_AUTOSIZE );
									cvShowImage("segments", img_clone);
									cvWaitKey(0);
									cvReleaseImage( &img_clone );
								}
								Plane * p_tmp = new Plane(s_x,s_y,s_z,s_w,s_h);
								if( p_tmp->normal_z > 0){
									p_tmp->normal_x*=-1;
									p_tmp->normal_y*=-1;
									p_tmp->normal_z*=-1;
								}
								angle_change = fabs(1-p->angle(p_tmp));
								delete p;
								p = p_tmp;
								if(!(s_x->size() > 5000/(current_step*current_step) && p->weight < 0.000100)){good_plane = false;delete p;}
							}else{good_plane = false;delete p;}
						}
						if(good_plane){
							IplImage * img_clone;
							bool display_here = false;
							if(display_here){
								img_clone = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
								cvCopy( rgb_img, img_clone, NULL );
							}
							for(int i = 0; i < width; i++){
								for(int j = 0; j < height; j++){
									if(z[i][j] != 0){
										if(fabs(p->distance(x[i][j],y[i][j],z[i][j])) < 0.02){
											if(display_here){cvRectangle(img_clone,cvPoint(i, j),cvPoint(i, j),cvScalar(255, 0, 255, 0), 1 , 8, 0);}
											z[i][j] = 0;
										}
									}
								}
							}
							planes->push_back(p);
							if(display_here){
								cvNamedWindow("segments", CV_WINDOW_AUTOSIZE );
								cvShowImage("segments", img_clone);
								cvWaitKey(0);
								cvReleaseImage( &img_clone );
							}
						}
					}
					delete s_x;
					delete s_y;
					delete s_z;
					delete s_w;
					delete s_h;
				}
			}
		}
	}
	for(int i = 0; i < width; i++){
		delete[] x[i];
		delete[] y[i];
		delete[] z[i];
		delete[] segment_id[i];
	}
	delete[] x;
	delete[] y;
	delete[] z;
	delete[] segment_id;
	
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("Segment cost: %f\n",time);
	return planes;
}
