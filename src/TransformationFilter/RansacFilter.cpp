#include "RansacFilter.h"

using namespace std;
RansacFilter::RansacFilter(){
	distance_threshold = 0.01;
	fast = false;
	nr_iter = 100;
}
RansacFilter::~RansacFilter(){}

bool RansacFilter::jcTest(Point * src_a, Point * src_b, Point * src_c, Point * dst_a, Point * dst_b, Point * dst_c){
	//if((src_a->point->distance( == src_b) || (src_a == src_c) || (src_b == src_c)){return false;}

	float src_a_to_b = src_a->distance(src_b);
	float src_a_to_c = src_a->distance(src_c);
	float src_b_to_c = src_b->distance(src_c);
	
	float dst_a_to_b = dst_a->distance(dst_b);
	float dst_a_to_c = dst_a->distance(dst_c);
	float dst_b_to_c = dst_b->distance(dst_c);
	
	if((fabs(src_a_to_b) < 3*distance_threshold) || (fabs(src_a_to_c) < 3*distance_threshold) || (fabs(src_b_to_c) < 3*distance_threshold)){
		return false;
	}
	
	if((fabs(dst_a_to_b) < 3*distance_threshold) || (fabs(dst_a_to_c) < 3*distance_threshold) || (fabs(dst_b_to_c) < 3*distance_threshold)){
		return false;
	}
	
	if((fabs(src_a_to_b-dst_a_to_b) < distance_threshold) && (fabs(src_a_to_c-dst_a_to_c) < distance_threshold) && (fabs(src_b_to_c-dst_b_to_c) < distance_threshold)){
		return true;
	}else{
		return false;
	}
}
const bool debugg_RansacFilter = false;

Transformation * RansacFilter::filterTransformation(Transformation * input){
	//printf("RansacFilter:start\n");
	
	IplImage* img_combine;
	int width;
	int height;
	if(debugg_RansacFilter)
	{	
		IplImage* rgb_img_src 	= cvLoadImage(input->src->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
		char * data_src = (char *)rgb_img_src->imageData;
		IplImage* rgb_img_dst 	= cvLoadImage(input->dst->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
		char * data_dst = (char *)rgb_img_dst->imageData;
		
		width = rgb_img_src->width;
		height = rgb_img_src->height;
		
		img_combine = cvCreateImage(cvSize(2*width,height), IPL_DEPTH_8U, 3);
		char * data = (char *)img_combine->imageData;
		


		int index = 0;
		
		for (int j = 0; j < height; j++)
		{
			for (int i = 0; i < width; i++)
			{
				
				int ind = 3*(640*j+i);
				data[3 * (j * (2*width) + (width+i)) + 0] = data_dst[ind +0];
				data[3 * (j * (2*width) + (width+i)) + 1] = data_dst[ind +1];
				data[3 * (j * (2*width) + (width+i)) + 2] = data_dst[ind +2];

				data[3 * (j * (2*width) + (i)) + 0] = data_src[ind +2];
				data[3 * (j * (2*width) + (i)) + 1] = data_src[ind +2];
				data[3 * (j * (2*width) + (i)) + 2] = data_src[ind +2];
				
			}
		}
		for(int i = 0; i < input->src->keypoints->valid_key_points.size(); i++){
			KeyPoint * src_kp = input->src->keypoints->valid_key_points.at(i);
			cvCircle(img_combine,cvPoint(src_kp->point->w			, src_kp->point->h), 5,cvScalar(0, 255, 0, 0),2, 8, 0);
		}
		
		for(int i = 0; i < input->dst->keypoints->valid_key_points.size(); i++){
			KeyPoint * dst_kp = input->dst->keypoints->valid_key_points.at(i);
			cvCircle(img_combine,cvPoint(dst_kp->point->w + width	, dst_kp->point->h), 5,cvScalar(0, 255, 0, 0),2, 8, 0);
		}
		
		cvNamedWindow("ransac combined image", CV_WINDOW_AUTOSIZE );
		cvShowImage("ransac combined image", img_combine);
		cvWaitKey(0);
		cvReleaseImage( &rgb_img_src );
		cvReleaseImage( &rgb_img_dst );
	}
	
	struct timeval start, end;
	gettimeofday(&start, NULL);
	Transformation * transformation = new Transformation();
	transformation->transformationMatrix = input->transformationMatrix;
	transformation->src = input->src;
	transformation->dst = input->dst;
	transformation->weight = 0;
	
	if(input->matches.size() > 3){
		vector<KeyPoint * > src_keypoints = input->src->keypoints->valid_key_points;
		vector<KeyPoint * > dst_keypoints = input->dst->keypoints->valid_key_points;

		int src_nr_points = src_keypoints.size();
		int dst_nr_points = dst_keypoints.size();
	
		float * pos_src_x = new float[src_nr_points];
		float * pos_src_y = new float[src_nr_points];
		float * pos_src_z = new float[src_nr_points];
	
		float * pos_dst_x = new float[dst_nr_points];
		float * pos_dst_y = new float[dst_nr_points];
		float * pos_dst_z = new float[dst_nr_points];
	
		Eigen::Matrix4f transformationMat = Eigen::Matrix4f::Identity();
	
		for(int i = 0; i < src_nr_points;i++)
		{
			pos_src_x[i] = src_keypoints[i]->point->x;
			pos_src_y[i] = src_keypoints[i]->point->y;
			pos_src_z[i] = src_keypoints[i]->point->z;
		}
		for(int i = 0; i < dst_nr_points;i++)
		{
			pos_dst_x[i] = dst_keypoints[i]->point->x;
			pos_dst_y[i] = dst_keypoints[i]->point->y;
			pos_dst_z[i] = dst_keypoints[i]->point->z;
		}
	
		std::vector<int> * src_best = 0;
		std::vector<int> * dst_best = 0;;
		pcl::TransformationFromCorrespondences tfc;
	
		for(int it = 0; it < nr_iter;it++){
			IplImage * img_combine_clone;
			if(debugg_RansacFilter){
				img_combine_clone = cvCreateImage(cvSize(img_combine->width, img_combine->height), IPL_DEPTH_8U, 3);
				cvCopy( img_combine, img_combine_clone, NULL );
			}
		
			tfc.reset();
			bool jc = false;
			for(int i = 0; i < 500; i++){
				int a = rand()%input->matches.size();
				int b = rand()%input->matches.size();
				int c = rand()%input->matches.size();
				KeyPoint * src_a = input->matches.at(a).first;
				KeyPoint * dst_a = input->matches.at(a).second;
		
				KeyPoint * src_b = input->matches.at(b).first;
				KeyPoint * dst_b = input->matches.at(b).second;
		
				KeyPoint * src_c = input->matches.at(c).first;
				KeyPoint * dst_c = input->matches.at(c).second;
				if(jcTest(src_a->point, src_b->point, src_c->point, dst_a->point, dst_b->point, dst_c->point)){
					tfc.add(src_a->point->pos,dst_a->point->pos);
					tfc.add(src_b->point->pos,dst_b->point->pos);
					tfc.add(src_c->point->pos,dst_c->point->pos);
				
					if(debugg_RansacFilter){
						cvLine(img_combine_clone,cvPoint(dst_a->point->w  + width ,dst_a->point->h),cvPoint(src_a->point->w,src_a->point->h),cvScalar(0, 0, 255, 0),1, 8, 0);
						cvLine(img_combine_clone,cvPoint(dst_b->point->w  + width ,dst_b->point->h),cvPoint(src_b->point->w,src_b->point->h),cvScalar(0, 0, 255, 0),1, 8, 0);
						cvLine(img_combine_clone,cvPoint(dst_c->point->w  + width ,dst_c->point->h),cvPoint(src_c->point->w,src_c->point->h),cvScalar(0, 0, 255, 0),1, 8, 0);
					}
				
					jc = true;
					break;
				}
			}
			if(debugg_RansacFilter ){
				cvShowImage("ransac combined image", img_combine_clone);
				cvWaitKey(0);
			}
		
			if(!jc){continue;}
		
			transformationMat = tfc.getTransformation().matrix();
		
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
		
			std::vector<int> src_good;
			std::vector<int> dst_good;

			for(int src_j = 0; src_j < src_nr_points; src_j++)
			{
				float x = pos_src_x[src_j];
				float y = pos_src_y[src_j];
				float z = pos_src_z[src_j];
				float tmp_x = x*mat00+y*mat01+z*mat02+mat03;
				float tmp_y = x*mat10+y*mat11+z*mat12+mat13;
				float tmp_z = x*mat20+y*mat21+z*mat22+mat23;
				float min = 100;
				int closest = -1;

				for(int dst_j = 0; dst_j < dst_nr_points; dst_j++){
				
				
					float dx = pos_dst_x[dst_j] - tmp_x;
					float dy = pos_dst_y[dst_j] - tmp_y;
					float dz = pos_dst_z[dst_j] - tmp_z;
					float dist = (dx*dx + dy*dy + dz*dz);
					if(dist < min){
						min = dist;
						closest = dst_j;
					}
				}

				if(min < distance_threshold*distance_threshold){
					src_good.push_back(src_j);
					dst_good.push_back(closest);
					if(debugg_RansacFilter){
						KeyPoint * src_kp = src_keypoints.at(src_j);
						KeyPoint * dst_kp = dst_keypoints.at(closest);
						cvLine(img_combine_clone,cvPoint(dst_kp->point->w  + width ,dst_kp->point->h),cvPoint(src_kp->point->w,src_kp->point->h),cvScalar(255, 0, 255, 0),1, 8, 0);
					}
				}
			}
			//printf("src_good.size() = %i\n",src_good.size());
			if(src_good.size() > transformation->weight){
				transformation->weight = src_good.size();
				if(src_best != 0){
					delete src_best;
					delete dst_best;
				}
			
				src_best = new vector<int>();
				dst_best = new vector<int>();
			
				for(int j = 0; j < src_good.size(); j++){
					src_best->push_back(src_good.at(j));
					dst_best->push_back(dst_good.at(j));
				}
			}
			if(debugg_RansacFilter ){
				cvShowImage("ransac combined image", img_combine_clone);
				cvWaitKey(0);
				cvReleaseImage( &img_combine_clone);
			}
		}
		tfc.reset();
		if(src_best != 0){
			for(int j = 0; j < src_best->size(); j++){
				tfc.add(src_keypoints.at(src_best->at(j))->point->pos,dst_keypoints.at(dst_best->at(j))->point->pos);
				transformation->matches.push_back(make_pair (src_keypoints.at(src_best->at(j)),dst_keypoints.at(dst_best->at(j))));
			}
		
			delete src_best;
			delete dst_best;
		
			transformation->transformationMatrix = tfc.getTransformation().matrix();
		}
	
		delete[] pos_src_x;
		delete[] pos_src_y;
		delete[] pos_src_z;
	
		delete[] pos_dst_x;
		delete[] pos_dst_y;
		delete[] pos_dst_z;
	}
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("Ransac cost: %f\n",time);
	//printf("RansacFilter:end\n");
	return transformation;
}
void RansacFilter::print(){printf("%s\n",name.c_str());}
void RansacFilter::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}
