#include "RGBDSegmentation.h"

const int size = 10;
const int offset = 0;
const int size_two = 8;
const int step = 4;
const int step_offset = 2;
const float fsize = float(size);
const float sizeSquare = float(fsize*fsize);
const float min_limit = 0.5f;
const float pi_half = 1.57079632679f;

RGBDSegmentation::RGBDSegmentation(Calibration * calib){
	calibration = calib;
	width 	= 640;
	height 	= 480;

	r 						= new float *[width];
	g 						= new float *[width];
	b 						= new float *[width];
	gray					= new float *[width];


	ii_gray					= new float *[width];

	x						= new float *[width];
	y						= new float *[width];
	z	 					= new float *[width];
	z_valid 				= new float *[width];
	
	ii_z 					= new float *[width];
	ii_z_valid 				= new float *[width];

	norm_x 					= new float *[width];
	norm_y 					= new float *[width];
	norm_z 					= new float *[width];
	norm_valid 				= new float *[width];
	
	ii_norm_x 				= new float *[width];
	ii_norm_y 				= new float *[width];
	ii_norm_z 				= new float *[width];
	ii_norm_valid			= new float *[width];
	
	depthEdge 				= new float *[width];
	rgbEdge 				= new float *[width];
	xedge 					= new float *[width];
	yedge 					= new float *[width];
	xrgbedge 				= new float *[width];
	yrgbedge 				= new float *[width];
	
	segmented 				= new int *[width];

	for(int i = 0; i < width; i++)
	{
		
		r[i]				= new float [height];
		g[i]				= new float [height];
		b[i]				= new float [height];
		gray[i]				= new float [height];

		ii_gray[i]			= new float [height];

		x[i]				= new float [height];
		y[i]				= new float [height];
		z[i]				= new float [height];
		z_valid[i]			= new float [height];

		ii_z[i]				= new float [height];
		ii_z_valid[i]		= new float [height];

		norm_x[i]			= new float [height];
		norm_y[i]			= new float [height];
		norm_z[i]			= new float [height];
		norm_valid[i]		= new float [height];

		ii_norm_x[i]		= new float [height];
		ii_norm_y[i]		= new float [height];
		ii_norm_z[i]		= new float [height];
		ii_norm_valid[i]	= new float [height];

		depthEdge[i] 		= new float [height];
		rgbEdge[i] 			= new float [height];
		
		xedge[i] 			= new float [height];
		yedge[i] 			= new float [height];
		xrgbedge[i] 		= new float [height];
		yrgbedge[i] 		= new float [height];
		
		segmented[i] 		= new int[height];
	}
	
	d_scaleing	= calibration->ds/calibration->scale;
	centerX		= calibration->cx;
	centerY		= calibration->cy;
	invFocalX	= 1.0f/calibration->fx;
    invFocalY	= 1.0f/calibration->fy;
    
    smoothing_steps = 9;
    float mid = 4;
    float smoothing_base = 5;
	nr_angles = 1800;
	smoothKernels = new float **[nr_angles];
	for(int angle_count = 0; angle_count < nr_angles; angle_count++)
	{
		float angle = pi_half*float(angle_count)/(float(nr_angles-1));
		float sigma_x = 2.5;
		float sigma_y = 1.0f;
		float sigma_x_2 = sigma_x*sigma_x;
		float sigma_y_2 = sigma_y*sigma_y;
		
		//printf("angle: %f\n",angle);
		
		float a = cos(angle)*cos(angle)/(2*sigma_x_2) 	+ sin(angle)*sin(angle)/(2*sigma_y_2);
		float b = -sin(2*angle)/(4*sigma_x_2) 			+ sin(2*angle)/(4*sigma_y_2);
		float c = sin(angle)*sin(angle)/(2*sigma_x_2) 	+ cos(angle)*cos(angle)/(2*sigma_y_2);
		smoothKernels[angle_count] = new float*[smoothing_steps];
		for(int i = 0; i < smoothing_steps; i++)
		{
			smoothKernels[angle_count][i] = new float[smoothing_steps];
			float x = smoothing_base*2.0*(float(i)-mid)/(float(smoothing_steps-1));
			for(int j = 0; j < smoothing_steps; j++)
			{
				float y = smoothing_base*2.0*(float(j)-mid)/(float(smoothing_steps-1));
				smoothKernels[angle_count][i][j] = exp(-(a*x*x + 2*b*x*y + c*y*y));//-0.025f;
			}
		}
	}
}
RGBDSegmentation::~RGBDSegmentation(){
	for(int i = 0; i < width; i++)
	{
		delete[] r[i];
		delete[] g[i];
		delete[] b[i];
		
		delete[] x[i];
		delete[] y[i];
		delete[] z[i];
		delete[] z_valid[i];
	
		delete[] norm_x[i];
		delete[] norm_y[i];
		delete[] norm_z[i];
		delete[] norm_valid[i];
		
		delete[] ii_gray[i];
		
		delete[] ii_z[i];
		delete[] ii_z_valid[i];
	
		delete[] ii_norm_x[i];
		delete[] ii_norm_y[i];
		delete[] ii_norm_z[i];
		delete[] ii_norm_valid[i];
	}
	
	delete[] r;
	delete[] g;
	delete[] b;
	
	delete[] x;
	delete[] y;
	delete[] z;
	delete[] z_valid;
	
	delete[] norm_x;
	delete[] norm_y;
	delete[] norm_z;
	delete[] norm_valid;
	
	delete[] ii_gray;
	
	delete[] ii_z;
	delete[] ii_z_valid;
	
	delete[] ii_norm_x;
	delete[] ii_norm_y;
	delete[] ii_norm_z;
	delete[] ii_norm_valid;
}

void RGBDSegmentation::disp_segments()
{
	IplImage * img 			= cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
	float * data 			= (float *)(img->imageData);
	float* tmp_red = new float[1000];
	float* tmp_green = new float[1000];
	float* tmp_blue = new float[1000];
	for(int i = 0; i < 1000; i ++)
	{
		tmp_red[i] = float(rand()%10000)/10000.0f;
		tmp_green[i] = float(rand()%10000)/10000.0f;
		tmp_blue[i] = float(rand()%10000)/10000.0f;
		
	}
	for(int i = 0; i < width; i++)
	{
		for(int j = 0; j < height; j++)
		{
			int seg = segmented[i][j]+1;
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
	cvWaitKey(100);
	cvReleaseImage( &img );
}

void RGBDSegmentation::disp_edge(string s , float** edge, int width,int height)
{
	IplImage * img 			= cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
	float * data 			= (float *)(img->imageData);
	float max = -10000000000;
	float min = 10000000000;
	for(int i = 0; i < width; i++)
	{
		for(int j = 0; j < height; j++)
		{
			if(edge[i][j] != -1){
				if(edge[i][j] < min){min = edge[i][j];}
				if(edge[i][j] > max){max = edge[i][j];}
			}
		}
	}
	min = 0;
	for(int i = 0; i < width; i++)
	{
		for(int j = 0; j < height; j++)
		{
			if(edge[i][j] != -1){
				data[(j * width + i)*3+0] = (edge[i][j]-min)/(max-min);
				data[(j * width + i)*3+1] = (edge[i][j]-min)/(max-min);
				data[(j * width + i)*3+2] = (edge[i][j]-min)/(max-min);
			}else{
				data[(j * width + i)*3+0] = 0;
				data[(j * width + i)*3+1] = 0;
				data[(j * width + i)*3+2] = 1;
			}
		}
	}
	cvNamedWindow(s.c_str(), CV_WINDOW_AUTOSIZE );
	cvShowImage(s.c_str(), img);
	cvWaitKey(0);
	cvReleaseImage( &img );
}

void RGBDSegmentation::disp_xy_edge(string s , float** xedge,float** yedge, int width,int height)
{
	IplImage * img 			= cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
	float * data 			= (float *)(img->imageData);
	float xmax = -10000000000;
	float xmin = 10000000000;
	
	float ymax = -10000000000;
	float ymin = 10000000000;
	for(int i = 0; i < width; i++)
	{
		for(int j = 0; j < height; j++)
		{
			if(xedge[i][j] != -1){
				if(xedge[i][j] < xmin){xmin = xedge[i][j];}
				if(xedge[i][j] > xmax){xmax = xedge[i][j];}
			}
			if(yedge[i][j] != -1){
				if(yedge[i][j] < ymin){ymin = yedge[i][j];}
				if(yedge[i][j] > ymax){ymax = yedge[i][j];}
			}
		}
	}
	xmin = 0;
	ymin = 0;
	for(int i = 0; i < width; i++)
	{
		for(int j = 0; j < height; j++)
		{
			data[(j * width + i)*3+0] = 1;
			data[(j * width + i)*3+1] = 0;
			data[(j * width + i)*3+2] = 0;
			if(xedge[i][j] != -1){
				data[(j * width + i)*3+0] = 0;
				data[(j * width + i)*3+1] = (xedge[i][j]-xmin)/(xmax-xmin);
			}
			
			if(yedge[i][j] != -1){
				data[(j * width + i)*3+0] = 0;
				data[(j * width + i)*3+2] = (yedge[i][j]-ymin)/(ymax-ymin);
			}
		}
	}
	cvNamedWindow(s.c_str(), CV_WINDOW_AUTOSIZE );
	cvShowImage(s.c_str(), img);
	cvWaitKey(0);
	cvReleaseImage( &img );
}

void normalizeEdges(float** edge, int width,int height)
{
	float max = -10000000000;
	float min = 10000000000;
	for(int i = 0; i < width; i++)
	{
		for(int j = 0; j < height; j++)
		{
			if(edge[i][j] != -1){
				if(edge[i][j] < min){min = edge[i][j];}
				if(edge[i][j] > max){max = edge[i][j];}
			}
		}
	}

	for(int i = 0; i < width; i++)
	{
		for(int j = 0; j < height; j++)
		{
			if(edge[i][j] != -1){
				edge[i][j] = (edge[i][j]-min)/(max-min);
			}
		}
	}
}

void mergeRGBandDepth(float** depthEdge, float** rgbEdge, int width,int height)
{
	for(int i = 0; i < width; i++)
	{
		for(int j = 0; j < height; j++)
		{
			if(depthEdge[i][j] != -1){
				depthEdge[i][j] = 0.1*depthEdge[i][j]+0.1*rgbEdge[i][j]+1*depthEdge[i][j]*rgbEdge[i][j];
			}
			
		}
	}
}

void fixDeadPixels(float** depthEdge, float** rgbEdge, int width,int height)
{
	for(int i = 0; i < width; i++)
	{
		for(int j = 0; j < height; j++)
		{
			if(depthEdge[i][j] == -1){
				depthEdge[i][j] = 1;//0.1*depthEdge[i][j]+0.1*rgbEdge[i][j]+1*depthEdge[i][j]*rgbEdge[i][j];
			}
		}
	}
}
float edgetest(float a, float b, float c, float d){
	//Eigen::Vector2f vec1(fsize,b-a); vec1.normalize();
	//Eigen::Vector2f vec2(fsize,d-c); vec2.normalize();
	//float d3 = (1-vec1.dot(vec2))/(100+fabs(a+b-c-d));
	float ab = (a-b);
	float bc = (b-c);
	float cd = (c-d);
	float abcd = fabs(ab-cd)/(0.1+fabs(a-d));
	return abcd;//(1-vec1.dot(vec2));///(50+fabs(a+b-c-d));
}

float edgetest(float * arr){

	float ab = (arr[0]-arr[1]);
	float bc = (arr[1]-arr[2]);
	float cd = (arr[2]-arr[3]);
	float normal = 100.0f*fabs(ab-cd)/(0.1+fabs(arr[0]-arr[3]));
	float depth = fabs(arr[0]-arr[3])*exp(-(fabs(ab)+fabs(cd))/30.0f);
	return normal+depth;
}

void RGBDSegmentation::directional_smoothing(float** xedge, float** yedge, float** edge, int width, int height){
	for(int w = 0; w < width; w++){
		for(int h = 0; h < height; h++){
			edge[w][h] = 0;
		}
	}

	int startVal = 1+smoothing_steps/2;
	for(int w = startVal; w < width-startVal; w++)
	{
		for(int h = startVal; h < height-startVal; h++)
		{
			if(xedge[w][h] != -1){
				float xed = xedge[w][h];
				float yed = yedge[w][h];
				float scale = sqrt(xed*xed+yed*yed);
				if(scale > 1e-3){
					float angle;
					if(xed < 1e-5){			angle = pi_half;}
					else if(yed < 1e-5){	angle = 0;}
					else{					angle = atan(yed/xed);}
					angle = pi_half-angle;
					angle *= float(nr_angles-1)/pi_half;
					//edge[w][h] += scale;
					float ** kernel = smoothKernels[int(angle)];
					
					for(int i = 0; i < smoothing_steps; i++){
						float * kernelArray = kernel[i];
						for(int j = 0; j < smoothing_steps; j++){
							edge[w+i-startVal][h+j-startVal] += kernelArray[j]*scale;
						}
					}
					
				}
			}
		}
	}
}
float** RGBDSegmentation::getEdges()
{
	ii_gray[0][0] = gray[0][0];
	
	ii_z[0][0] = z[0][0];
	ii_z_valid[0][0] = z_valid[0][0];
	
	for(int i = 1; i < width; i++){
		int i_minus =  i-1;
		ii_gray[i][0] = gray[i][0]+ii_gray[i_minus][0];
		
		ii_z[i][0] = z[i][0]+ii_z[i_minus][0];
		ii_z_valid[i][0] = z_valid[i][0]+ii_z_valid[i_minus][0];
	}
	
	float * first_ii_gray = ii_gray[0];

	float * first_ii_z = ii_z[0];
	float * first_ii_z_valid = ii_z_valid[0];
	
	for(int j = 1; j < height; j++){
		int j_minus = j-1;
		first_ii_gray[j] = gray[0][j]+first_ii_gray[j_minus];
		
		first_ii_z[j] = z[0][j]+first_ii_z[j_minus];
		first_ii_z_valid[j] = z_valid[0][j]+first_ii_z_valid[j_minus];
	}
	

	for(int i = 1; i < width; i++){
		int i_minus =  i-1;
		
		float * gray_i = gray[i];
		float * ii_gray_i = ii_gray[i];
		float * ii_gray_i_minus = ii_gray[i-1];
		
		float * z_i = z[i];
		float * ii_z_i = ii_z[i];
		float * ii_z_i_minus = ii_z[i-1];
		
		float * z_valid_i = z_valid[i];
		float * ii_z_valid_i = ii_z_valid[i];
		float * ii_z_valid_i_minus = ii_z_valid[i-1];
		
		for(int j = 1; j < height; j++){
			int j_minus = j-1;
			ii_gray_i[j] = gray_i[j]+ii_gray_i[j_minus]+ii_gray_i_minus[j]-ii_gray_i_minus[j_minus];
		
			ii_z_i[j] = z_i[j]+ii_z_i[j_minus]+ii_z_i_minus[j]-ii_z_i_minus[j_minus];
			ii_z_valid_i[j] = z_valid_i[j]+ii_z_valid_i[j_minus]+ii_z_valid_i_minus[j]-ii_z_valid_i_minus[j_minus];
		}
	}
	
	for(int i = 0; i < width; i++)
	{
		for(int j = 0; j < height; j++)
		{
			depthEdge[i][j] = 0;
			rgbEdge[i][j] = 0;
			
			xedge[i][j] = -1;
			yedge[i][j] = -1;
			xrgbedge[i][j] = -1;
			yrgbedge[i][j] = -1;
		}
	}
	
		float z_x_valid[step];
	float z_y_valid[step];
	
	float z_x_d0[step];
	float z_y_d0[step];
	float z_x_d1[step-1];
	float z_y_d1[step-1];
	
	int rgb_size = 3;
	int rgb_size_two = 2;
	for(int i = rgb_size; i < width-rgb_size; i++)
	{
		for(int j = rgb_size; j < height-rgb_size; j++)
		{
			float gray_x_n = ii_gray[i][j] - ii_gray[i][j-rgb_size_two] - ii_gray[i-rgb_size][j] + ii_gray[i-rgb_size][j-rgb_size_two];
			float gray_x_p = ii_gray[i+rgb_size][j] - ii_gray[i+rgb_size][j-rgb_size_two] - ii_gray[i][j] + ii_gray[i][j-rgb_size_two];
						
			float gray_y_n = ii_gray[i][j] - ii_gray[i-rgb_size_two][j] - ii_gray[i][j-rgb_size] + ii_gray[i-rgb_size_two][j-rgb_size];
			float gray_y_p = ii_gray[i][j+rgb_size] - ii_gray[i-rgb_size_two][j+rgb_size] - ii_gray[i][j] + ii_gray[i-rgb_size_two][j];
			
			xrgbedge[i][j] = (gray_x_p-gray_x_n)*(gray_x_p-gray_x_n);
			yrgbedge[i][j] = (gray_y_p-gray_y_n)*(gray_y_p-gray_y_n);
		}
	}

	for(int i = offset+step_offset*size; i < width-(-offset+step_offset*size); i++)
	{
		for(int j = offset+step_offset*size; j < height-(-offset+step_offset*size); j++)
		{
			int i_offset = i+offset;
			int j_offset = j+offset;
			for(int k = 0; k < step; k++)
			{
				int kt1s = (k-step_offset+1)*size;
				int kts = (k-step_offset)*size;
				z_x_valid[k] = ii_z_valid[i_offset+kt1s][j] - ii_z_valid[i_offset+kt1s][j-size_two] - ii_z_valid[i_offset+kts][j] + ii_z_valid[i_offset+kts][j-size_two];			
				z_y_valid[k] = ii_z_valid[i][j_offset+kt1s] - ii_z_valid[i-size_two][j_offset+kt1s] - ii_z_valid[i][j_offset+kts] + ii_z_valid[i-size_two][j_offset+kts];
			}
		
			bool valid = true;
		
			for(int k = 0; k < step; k++)
			{
				valid = z_x_valid[k] > min_limit && z_y_valid[k] > min_limit && valid;
			}
		
			if(valid)
			{
				for(int k = 0; k < step; k++){
					int kt1s = (k-step_offset+1)*size;
					int kts = (k-step_offset)*size;
					z_x_d0[k] = ii_z[i_offset+kt1s][j] - ii_z[i_offset+kt1s][j-size_two] - ii_z[i_offset+kts][j] + ii_z[i_offset+kts][j-size_two];			
					z_y_d0[k] = ii_z[i][j_offset+kt1s] - ii_z[i-size_two][j_offset+kt1s] - ii_z[i][j_offset+kts] + ii_z[i-size_two][j_offset+kts];
					z_x_d0[k] /= z_x_valid[k];
					z_y_d0[k] /= z_y_valid[k];
				}			
				for(int k = 0; k < step-1; k++){
					z_x_d1[k]=z_x_d0[k]-z_x_d0[k+1];
					z_y_d1[k]=z_y_d0[k]-z_y_d0[k+1];
				}

				xedge[i][j]  = edgetest(z_x_d0);//z_x_d1[1];
				yedge[i][j]  = edgetest(z_y_d0);//z_y_d1[1];
			}else{
				xedge[i][j] = -1;
				yedge[i][j] = -1;
			}
		}
	}

	directional_smoothing(xedge,yedge,depthEdge,width,height);
	directional_smoothing(xrgbedge,yrgbedge,rgbEdge,width,height);
	for(int i = 0; i < width; i++)
	{
		for(int j = 0; j < height; j++)
		{
			if(xedge[i][j] == -1){depthEdge[i][j] = -1;}
		}
	}
	normalizeEdges(depthEdge, width, height);
	normalizeEdges(rgbEdge, width, height);
	mergeRGBandDepth(depthEdge, rgbEdge, width, height);
	normalizeEdges(depthEdge, width, height);
	fixDeadPixels(depthEdge, rgbEdge, width, height);
	return rgbEdge;//depthEdge;
}

void RGBDSegmentation::init(IplImage * rgb_img,IplImage * depth_img)
{    
	float d_scaleing	= calibration->ds/calibration->scale;
	float centerX		= calibration->cx;
	float centerY		= calibration->cy;
	float invFocalX	= 1.0f/calibration->fx;
    float invFocalY	= 1.0f/calibration->fy;
	
	char * rgb_data		= (char *)rgb_img->imageData;
	unsigned short * depth_data	= (unsigned short *)depth_img->imageData;
	
	for(int h = 0; h < height; h++)
	{
		for(int w = 0; w < width; w++)
		{
			int ind = 640*h+w;
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
				tmp_x = (w - centerX) * tmp_z * invFocalX;
		       	tmp_y = (h - centerY) * tmp_z * invFocalY;
			}
			r[w][h] = float(tmp_r);
			g[w][h] = float(tmp_g);
			b[w][h] = float(tmp_b);
			gray[w][h] = float(tmp_r+tmp_g+tmp_b)*0.3333f;
			
			x[w][h] = tmp_x;
			y[w][h] = tmp_y;
			z[w][h] = tmp_z;
		}
	}
}

void RGBDSegmentation::extractSurfaces(vector<segmentation_fit * > * fit_segments, vector<int> * segment_w, vector<int> * segment_h, float ** edge, int & segment_id ,int iter, int width,int height, float min)
{
	int segment_counter	= segment_id+1;
	int offset 			= 25;
	int max_w 			= width-offset;
	int max_h 			= height-offset;
	
	for(unsigned int counter = 0; counter < segment_w->size(); counter++)
	{
		int i = segment_w->at(counter);
		int j = segment_h->at(counter);
		if(segmented[i][j] == segment_id && edge[i][j] < min){
			segmented[i][j] = segment_counter;
			vector<int>		  queue_w;
			vector<int>		  queue_h;
			vector<int>		* seg_w	= new vector<int>();
			vector<int>		* seg_h	= new vector<int>();
			vector<float>	* seg_x	= new vector<float>();
			vector<float>	* seg_y	= new vector<float>();
			vector<float>	* seg_z	= new vector<float>();
				
			queue_w.push_back(i);
			queue_h.push_back(j);
			while(!queue_w.empty()){
				int w_current = queue_w.back();
				int h_current = queue_h.back();
				//printf("w_current: %i, h_current:%i\n",w_current,h_current);
				queue_w.pop_back();
				queue_h.pop_back();
					
				seg_w->push_back(w_current);
				seg_h->push_back(h_current);
				seg_x->push_back(x[w_current][h_current]);
				seg_y->push_back(y[w_current][h_current]);
				seg_z->push_back(z[w_current][h_current]);

				int curr_min_w = w_current-1;
				if(curr_min_w < offset)		{curr_min_w = offset;}
				int curr_max_w = w_current+2;
				if(curr_max_w > max_w)	{curr_max_w = max_w;}
				
				int curr_min_h = h_current-1;
				if(curr_min_h < offset)		{curr_min_h = offset;}
				int curr_max_h = h_current+2;
				if(curr_min_h > max_h)	{curr_min_h = max_h;}
					
				for(int a = curr_min_w; a < curr_max_w; a++){
					for(int b = curr_min_h; b < curr_max_h; b++){
						if(segmented[a][b] == segment_id && edge[a][b] < min){
							segmented[a][b] = segment_counter;
							queue_w.push_back(a);
							queue_h.push_back(b);
						}
					}
				}
			}
				
			segmentation_fit * fit = fit_surface(seg_x, seg_y, seg_z);
			delete seg_x;
			delete seg_y;
			delete seg_z;
			
			if		(fit != 0 && fit->S(2)/sqrt(fit->S(0)) < 0.0002f){
				fit->seg_w = seg_w;
				fit->seg_h = seg_h;
				fit_segments->push_back(fit);
			}
			else if	(fit != 0 && min > 0.003f){
				int  sid = segment_counter;
				extractSurfaces(fit_segments,seg_w,seg_h,edge,sid,iter,width,height,min*0.5f);
				segment_counter = sid;
				delete fit;
				delete seg_w;
				delete seg_h;
			}
			else{
				if(fit == 0){delete fit;}
				delete seg_w;
				delete seg_h;
			}
			segment_counter++;
		}
	}
}

segmentation_fit * RGBDSegmentation::fit_surface(vector<float> * seg_x, vector<float> * seg_y, vector<float> * seg_z)
{
	float x_sum = 0;
	float y_sum = 0;
	float z_sum = 0;
	int nr_points = 0;
	for(unsigned int i = 0; i < seg_x->size(); i++)
	{
		float z = seg_z->at(i);
		if(z > 0 && !std::isnan(z))
		{
			nr_points++;
			x_sum+=seg_x->at(i);
			y_sum+=seg_y->at(i);
			z_sum+=z;
		}
	}
	
	if(nr_points > 1000)
	{
		x_sum/=(float)nr_points;
		y_sum/=(float)nr_points;
		z_sum/=(float)nr_points;
		
		segmentation_fit * fit = new segmentation_fit;
		fit->nr_valid = nr_points;
		fit->mean[0] = x_sum;
		fit->mean[1] = y_sum;
		fit->mean[2] = z_sum;
		
		float * x = new float[nr_points];
		float * y = new float[nr_points];
		float * z = new float[nr_points];
		std::vector<float *> data;
		data.push_back(x);
		data.push_back(y);
		data.push_back(z);
		int c = 0;
		for(unsigned int i = 0; i < seg_x->size(); i++){
			float zi = seg_z->at(i);
			if(zi > 0 && !std::isnan(zi)){
				x[c] = seg_x->at(i) - x_sum;
				y[c] = seg_y->at(i) - y_sum;
				z[c] = seg_z->at(i) - z_sum;
				c++;
			}
		}
				
		MatrixXf covMat(data.size(),data.size());

		for(unsigned int i = 0; i < data.size(); i++)
		{
			for(int unsigned j = i; j < data.size(); j++)
			{
				float * col1 	= data.at(i);
				float * col2 	= data.at(j);
				float sum = 0;
				for(int k = 0; k < nr_points; k++)
				{
					sum+=col1[k]*col2[k];
				}
				covMat(i,j)=sum/float(nr_points-1);
				covMat(j,i)=covMat(i,j);
			}
		}		
		fit->covMat = covMat;
		JacobiSVD<MatrixXf> svd(covMat, ComputeThinU | ComputeThinV);
		fit->U = svd.matrixU();
		fit->S = svd.singularValues();
		fit->V = svd.matrixV();
		//S/=S(0);
		delete[] x;
		delete[] y;
		delete[] z;
		return fit;
	}
	return 0;
}

void RGBDSegmentation::extractSurfaces(vector<segmentation_fit * > * fit_segments, float ** edge, int width,int height)
{
	int segment_id = -1;
	for(int i = 0; i < width; i++){
		for(int j = 0; j < height; j++){
			segmented[i][j] = segment_id;
		}
	}

	float min 			= 0.5f;
	int offset 			= 25;
	int max_w 			= width-offset;
	int max_h 			= height-offset;
	
	vector<int> * seg_w			= new vector<int>();
	vector<int> * seg_h			= new vector<int>();
	

	for(int i = offset; i < max_w; i++)
	{
		for(int j = offset; j < max_h; j++)
		{
			seg_w->push_back(i);
			seg_h->push_back(j);
		}
	}
	extractSurfaces(fit_segments,seg_w,seg_h,edge,segment_id,segment_id,width,height,min);
	
	delete seg_w;
	delete seg_h;
	
	for(unsigned int i = 0; i < fit_segments->size(); i++){
		//fit_segments->at(i)->seg_h = selected_segments_h->at(i);
		//fit_segments->at(i)->seg_w = selected_segments_w->at(i);
	}
};

vector<segmentation_fit * > * RGBDSegmentation::segment(IplImage * rgb_img,IplImage * depth_img)
{
	//cvNamedWindow("rgb", CV_WINDOW_AUTOSIZE );
	//cvShowImage("rgb", rgb_img);
	//cvWaitKey(0);
	
	struct timeval start, end;
	gettimeofday(&start, NULL);
	
	init(rgb_img, depth_img);
	float** edge = getEdges();
	//disp_edge("Edge", edge, width,height);
	
	vector<segmentation_fit * > * fit_segments		= new vector<segmentation_fit * >();
	
	extractSurfaces(fit_segments,edge,width,height);
	
	//for(unsigned int i = 0; i < fit_segments->size(); i++){
	//	fit_segments->at(i)->seg_h = selected_segments_h->at(i);
	//	fit_segments->at(i)->seg_w = selected_segments_w->at(i);
	//}
	
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("segtime: %f\n",time);
	
	
	//disp_segments();
	printf("nr good segments: %i\n",fit_segments->size());
	
	
	//for(unsigned int i = 0; i < fit_segments->size(); i++){delete fit_segments->at(i);}
	//delete fit_segments;
	
	//disp_xy_edge("xyEdge", xedge,yedge, width,height);
	//disp_xy_edge("rgbxyEdge", xrgbedge,yrgbedge, width,height);
	//disp_edge("Edge", depthEdge, width,height);
	
	//for(int i = 0; i < width; i++){delete edge[i];}
	//delete[] edge;
	
	//cvNamedWindow("src image", CV_WINDOW_AUTOSIZE );
	//cvShowImage("src image", rgb_img);
		
	//cvNamedWindow("dst image", CV_WINDOW_AUTOSIZE );
	//cvShowImage("dst image", depth_img);

	//cvWaitKey(0);

	return fit_segments;
}
