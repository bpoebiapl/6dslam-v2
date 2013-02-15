#include "RGBDSegmentationTest.h"
#include "mygeometry/mygeometry.h"
using namespace std;

RGBDSegmentationTest::RGBDSegmentationTest(){}
RGBDSegmentationTest::~RGBDSegmentationTest(){}
vector<Plane * > * RGBDSegmentationTest::segment(IplImage * rgb_img,IplImage * depth_img){

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

	vector<int> edges_w;
	vector<int> edges_h;
	
	vector<Point *> edges_p;
	
	IplImage * img_clone = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	cvCopy( rgb_img, img_clone, NULL );
	char * cdata 			= (char *)(img_clone->imageData);
	

	
	float gray_mul = 1.0f/40000.0f;
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
			
			edge[i][j] = float(gw_edge*gw_edge+gh_edge*gh_edge)*gray_mul;// + (zw_edge*zw_edge+zh_edge*zh_edge)*0.01;
			if(edge[i][j]>0.75f){
				int ind = 640*j+i;
				cdata[3*ind+0] = 0;
				cdata[3*ind+1] = 255;
				cdata[3*ind+2] = 0;
			}
		}
	}
	
	int step = 1;
	cvNamedWindow("edges", CV_WINDOW_AUTOSIZE );
	cvShowImage("edges", img_clone);
	cvWaitKey(0);
	//cvReleaseImage( &img_clone );
	int ** vertical = new int*[width/step];
	int ** horisontal = new int*[width/step];
	int ** diagonal1 = new int*[width/step];
	int ** diagonal2 = new int*[width/step];
	for(int i = 0; i < width/step; i++){
		vertical[i] = new int[height/step];
		horisontal[i] = new int[height/step];
		diagonal1[i] = new int[height/step];
		diagonal2[i] = new int[height/step];
		for(int j = 0; j < height/step; j++){
			vertical[i][j] = 0;
			horisontal[i][j] = 0;
			diagonal1[i][j] = 0;
			diagonal2[i][j] = 0;
		}
	}
	
	for(int i = 0; i < width/step; i++){
		for(int j = 0; j < height/step; j++){
			int w = i*step;
			int h = j*step;
			
			int sum = -1;
			
			int current_h = h;
			int current_w = w;
			while(edge[current_w][current_h] < 0.75){
				current_h++;
				sum++;
			}
			current_h = h;
			current_w = w;
			while(edge[current_w][current_h] < 0.75){
				current_h--;
				sum++;
			}
			vertical[i][j] = sum;
			sum = -1;
			current_h = h;
			current_w = w;
			
			while(edge[current_w][current_h] < 0.75){
				current_w++;
				sum++;
			}
			current_h = h;
			current_w = w;
			while(edge[current_w][current_h] < 0.75){
				current_w--;
				sum++;
			}
			horisontal[i][j] = sum;
			sum = -1;
			
			
			current_h = h;
			current_w = w;
			while(edge[current_w][current_h] < 0.75){
				current_w++;
				current_h++;
				sum++;
			}
			current_h = h;
			current_w = w;
			while(edge[current_w][current_h] < 0.75){
				current_w--;
				current_h--;
				sum++;
			}
			diagonal1[i][j] = sum;
			sum = -1;
			
			current_h = h;
			current_w = w;
			while(edge[current_w][current_h] < 0.75){
				current_w--;
				current_h++;
				sum++;
			}
			current_h = h;
			current_w = w;
			while(edge[current_w][current_h] < 0.75){
				current_w++;
				current_h--;
				sum++;
			}
			diagonal2[i][j] = sum;
			sum = -1;
			
			double val = long(vertical[i][j]) * long(horisontal[i][j]) * long(diagonal1[i][j]) * long(diagonal2[i][j]);
			val/=1;
			
			double plotval = sqrt(sqrt(val));
			if(plotval > 255)	{plotval = 255;}
			else if(plotval < 0){plotval = 0;}
			int ind = 640*h+w;
			cdata[3*ind+0] = 0;
			cdata[3*ind+1] = 0;
			cdata[3*ind+2] = int(plotval);
		}
	}
	cvNamedWindow("edges", CV_WINDOW_AUTOSIZE );
	cvShowImage("edges", img_clone);
	cvWaitKey(0);
	
	gettimeofday(&start, NULL);
	//int step = 15;
	int size = 2*height;
	vector<Plane * > * planes = new vector<Plane * >();
	
	while(true){
		float best_score = -1;
		float best_score_scale = -1;
		Plane * best_plane = 0;
		int counter = 0;
		for(int i = step; i <= width-step; i+=step){
			for(int j = step; j <= height-step; j+=step){
				if(z[i][j] != 0){
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
						}else{delete p;}
					}else{
						delete seg_x;
						delete seg_y;
						delete seg_z;
					}
				}
			}
		}
		if(best_plane != 0){
			vector<float > * s_x;
			vector<float > * s_y;
			vector<float > * s_z;
			vector<int > * s_w;
			vector<int > * s_h;
			Plane * best_plane_update = best_plane;
			if(best_plane_update->normal_z > 0){
				best_plane_update->normal_x*=-1;
				best_plane_update->normal_y*=-1;
				best_plane_update->normal_z*=-1;
			}
			double change = 10;
			for(int iter = 0; iter < 6 && change > 0.000000001; iter++)
			{
				s_x = new vector<float >();
				s_y = new vector<float >();
				s_z = new vector<float >();
				s_w = new vector<int >();
				s_h = new vector<int >();
						
				for(int i = step; i <= width-step; i+=step){
					for(int j = step; j <= height-step; j+=step){
						if((z[i][j] != 0) && fabs(best_plane_update->distance(x[i][j],y[i][j],z[i][j])) < 0.01){
							s_x->push_back(x[i][j]);
							s_y->push_back(y[i][j]);
							s_z->push_back(z[i][j]);
							s_w->push_back(i);
							s_h->push_back(j);
						}
					}
				}

				float normx = best_plane_update->normal_x;
				float normy = best_plane_update->normal_y;
				float normz = best_plane_update->normal_z;
			
				delete best_plane_update;
				best_plane_update = new Plane(s_x,s_y,s_z,s_w,s_h);

				if(best_plane_update->normal_z > 0){
					best_plane_update->normal_x*=-1;
					best_plane_update->normal_y*=-1;
					best_plane_update->normal_z*=-1;
				}
				change = 1 - (best_plane_update->normal_x*normx + best_plane_update->normal_y*normy + best_plane_update->normal_z*normz);
				//printf("change: %.10f\n",change);
			
				delete s_x;
				delete s_y;
				delete s_z;
				delete s_w;
				delete s_h;
			}
		
			s_x = new vector<float >();
			s_y = new vector<float >();
			s_z = new vector<float >();
			s_w = new vector<int >();
			s_h = new vector<int >();
		
			int matches = 0;
			int step2 = 3;
			for(int i = step2; i <= width-step2; i+=step2){
				for(int j = step2; j <= height-step2; j+=step2){
					if(z[i][j] != 0 && fabs(best_plane_update->distance(x[i][j],y[i][j],z[i][j])) < 0.015){
					
						s_x->push_back(x[i][j]);
						s_y->push_back(y[i][j]);
						s_z->push_back(z[i][j]);
						s_w->push_back(i);
						s_h->push_back(j);
					
						z[i][j] = 0;
						matches++;
					}
				}
			}
			best_plane_update = new Plane(s_x,s_y,s_z,s_w,s_h);
			if(best_plane_update->normal_z > 0){
				best_plane_update->normal_x*=-1;
				best_plane_update->normal_y*=-1;
				best_plane_update->normal_z*=-1;
			}

			if( /*matches * 50 > counter &&*/ matches*step2*step2 > 1000){planes->push_back(best_plane_update);}
			else{
				delete best_plane_update;
				break;
			}
		}else{break;}
	}

	
	//printf("planes->size() = %i\n",planes->size());
	/*
	float st4 = float(step)/4.0;
	IplImage * img_clone2 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	cvCopy( rgb_img, img_clone2, NULL );
	char * cdata2 			= (char *)(img_clone2->imageData);
	
	for(int i = 0; i < planes->size();i++){


		Plane * plane1 = planes->at(i);
		int r,g,b;
		if(i == 0)		{r=0;g=0;b=255;}
		else if(i == 1)	{r=0;g=255;b=0;}
		else if(i == 2)	{r=255;g=0;b=0;}
		else if(i == 3)	{r=0;g=255;b=255;}
		else if(i == 4)	{r=255;g=0;b=255;}
		else if(i == 5)	{r=255;g=255;b=0;}
		else 			{r=rand()%256;g=rand()%256;b=rand()%256;}
			
		for(int k = 0; k < plane1->w_vec->size(); k++){
			int pw = plane1->w_vec->at(k);
			int ph = plane1->h_vec->at(k);
			cvRectangle(img_clone2,cvPoint(pw-st4, ph-st4),cvPoint(pw+st4, ph+st4),cvScalar(b, g, r, 0),-1, 8, 0);
		}
	}

	cvNamedWindow("planeMix", CV_WINDOW_AUTOSIZE );
	cvShowImage("planeMix", img_clone2);
	cvWaitKey(0);
	cvReleaseImage( &img_clone2 );
	*/
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("Segment cost: %f\n",time);
	
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
	
	return planes;
}
