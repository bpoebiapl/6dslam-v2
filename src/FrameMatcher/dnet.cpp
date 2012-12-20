#include "dnet.h"
#include <vector>

using namespace std;

DNET::DNET(){name = "dnet";}

DNET::~DNET(){printf("delete dnet\n");}

using namespace std;
const bool debugg_DNET = true;

Transformation * DNET::getTransformation(RGBDFrame * src, RGBDFrame * dst)
{
	int max_points = 100;
	float max_feature = 0.10;
	float max_dist = 0.01;
	printf("dnet...\n");
	vector<KeyPoint * > src_keypoints;//	= src->keypoints->valid_key_points;
	int nr_loop_src = src->keypoints->valid_key_points.size();
	if(nr_loop_src > max_points){nr_loop_src = max_points;}
	int nr_loop_dst = dst->keypoints->valid_key_points.size();
	if(nr_loop_dst > max_points){nr_loop_dst = max_points;}
	
	for(int i = 0; i < nr_loop_src; i++){src_keypoints.push_back(src->keypoints->valid_key_points.at(i));}

	vector<KeyPoint * > dst_keypoints;//	= dst->keypoints->valid_key_points;
	for(int i = 0; i < nr_loop_dst; i++){dst_keypoints.push_back(dst->keypoints->valid_key_points.at(i));}
	int src_nr_points = src_keypoints.size();
	int dst_nr_points = dst_keypoints.size();
	
	IplImage* img_combine;
	int width;
	int height;
	if(debugg_DNET)
	{	
		IplImage* rgb_img_src 	= cvLoadImage(src->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
		char * data_src = (char *)rgb_img_src->imageData;
		IplImage* rgb_img_dst 	= cvLoadImage(dst->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
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

				data[3 * (j * (2*width) + (i)) + 0] = data_src[ind +0];
				data[3 * (j * (2*width) + (i)) + 1] = data_src[ind +1];
				data[3 * (j * (2*width) + (i)) + 2] = data_src[ind +2];
				
			}
		}
		
		for(int i = 0; i < dst_nr_points;i++){
			cvCircle(img_combine,cvPoint(dst_keypoints.at(i)->point->w + width	, dst_keypoints.at(i)->point->h), 5,cvScalar(0, 255, 0, 0),2, 8, 0);
		}
		for(int i = 0; i < src_nr_points;i++){
			cvCircle(img_combine,cvPoint(src_keypoints.at(i)->point->w			, src_keypoints.at(i)->point->h), 5,cvScalar(0, 255, 0, 0),2, 8, 0);
		}
		
		cvNamedWindow("combined image", CV_WINDOW_AUTOSIZE );
		cvShowImage("combined image", img_combine);
		
		cvWaitKey(0);
		cvReleaseImage( &rgb_img_src );
		cvReleaseImage( &rgb_img_dst );

	}

	if(debugg_DNET){printf("src_keypoints.size() = %i, dst_keypoints.size() = %i\n",int(src_keypoints.size()),int(dst_keypoints.size()));}


	float ** feature_distances = new float*[src_nr_points];

	vector< vector< float > > 	src_possible_matches_dist 	= vector< vector< float > >();
	vector< vector< int > > 	src_possible_matches_id 	= vector< vector< int > >();
	vector< vector< float > > 	dst_possible_matches_dist 	= vector< vector< float > >();
	vector< vector< int > > 	dst_possible_matches_id 	= vector< vector< int > >();
	for(int i = 0; i < src_nr_points;i++)
	{
		src_possible_matches_dist.push_back(vector< float > ());
		src_possible_matches_id.push_back(vector< int >());
	}
	
		for(int i = 0; i < dst_nr_points;i++)
	{
		dst_possible_matches_dist.push_back(vector< float > ());
		dst_possible_matches_id.push_back(vector< int >());
	}
	
	for(int i = 0; i < src_nr_points;i++)
	{
		feature_distances[i] = new float[dst_nr_points];
		for(int j = 0; j < dst_nr_points;j++)
		{
			FeatureDescriptor * descriptorA = src_keypoints.at(i)->descriptor;
			FeatureDescriptor * descriptorB = dst_keypoints.at(j)->descriptor;
			feature_distances[i][j] = (descriptorA->distance(descriptorB));
			if(feature_distances[i][j] < max_feature){
				src_possible_matches_dist.at(i).push_back(feature_distances[i][j]);
				src_possible_matches_id.at(i).push_back(j);
				dst_possible_matches_dist.at(j).push_back(feature_distances[i][j]);
				dst_possible_matches_id.at(j).push_back(i);
//cvLine(img_combine,cvPoint(dst_keypoints.at(j)->point->w+width,dst_keypoints.at(j)->point->h),cvPoint(src_keypoints.at(i)->point->w,src_keypoints.at(i)->point->h),cvScalar(0,0,255, 0),1,8,0);
			}
		}
	}
	//cvShowImage("combined image", img_combine);
	//cvWaitKey(0);
	
	struct timeval start, end;
	gettimeofday(&start, NULL);
	int ** counter = new int*[src_nr_points];
	for(int i = 0; i < src_nr_points; i++){
		counter[i] = new int[dst_nr_points];
		for(int j = 0; j < dst_nr_points; j++){
			counter[i][j] = 0;
		}
	}
	float ** dst_dist = new float*[dst_nr_points]; 
	for(int i = 0; i < dst_nr_points;i++){
		dst_dist[i] = new float[dst_nr_points]; 
		for(int j = i-1; j >= 0;j--){
			dst_dist[i][j] = dst_dist[j][i] = dst_keypoints.at(i)->point->distance(dst_keypoints.at(j)->point);
		}
	}
	
	int nr_good = 0;
	for(int i = 0; i < src_nr_points;i++){
		int nr_i = src_possible_matches_id.at(i).size();
		for(int j = i-1; j >= 0;j--){
			int nr_j = src_possible_matches_id.at(j).size();
			float src_distance = src_keypoints.at(i)->point->distance(src_keypoints.at(j)->point);
			if(src_distance > max_dist){
				vector<int> di = src_possible_matches_id.at(i);
				vector<int> dj = src_possible_matches_id.at(j);
				for(int ii = 0; ii < nr_i;ii++){
					int dst_1_id = di.at(ii);
					float * ddist_ii = dst_dist[dst_1_id];
					for(int jj = 0; jj < nr_j;jj++){
						int dst_2_id = dj.at(jj);
						float diff_distance = ddist_ii[dst_2_id]-src_distance;
						if(fabs(diff_distance) < max_dist){
							counter[i][dst_1_id]++;
							counter[j][dst_2_id]++;
						}
					}
				}
			}
		}
	}
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("net cost: %f\n",time);
	int * src_matches = new int[src_nr_points];
	for(int i = 0; i < src_nr_points; i++){
		int best = 0;
		int best_id = 0;
		for(int j = 0; j < dst_nr_points; j++){
			if(counter[i][j]>best){
				best = counter[i][j];
				best_id = j;
			}
		}
		if(best >= 20){
			src_matches[i] = best_id;
			printf("%i\n",best);//src_matches[i]);
		}else{src_matches[i] = -1;}
	}
	
	for(int i = 0; i < src_nr_points; i++){
		if(!(src_matches[i] == -1)){
			//printf("%i\n",src_matches[i]);
			cvLine(img_combine,cvPoint(dst_keypoints.at(src_matches[i])->point->w+width,dst_keypoints.at(src_matches[i])->point->h),cvPoint(src_keypoints.at(i)->point->w,src_keypoints.at(i)->point->h),cvScalar(0,255,0, 0),1,8,0);
		}
	}
	cvShowImage("combined image", img_combine);
	cvWaitKey(0);

	exit(0);
	Transformation * transformation = new Transformation();
	transformation->transformationMatrix = Eigen::Matrix4f::Identity();;
	transformation->src = src;
	transformation->dst = dst;
	transformation->weight = 100;

	if(debugg_DNET){printf("done\n");cvReleaseImage( &img_combine );}
	
	return transformation;
}
