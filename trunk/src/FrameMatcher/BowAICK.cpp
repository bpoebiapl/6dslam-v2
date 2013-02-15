#include "BowAICK.h"
#include <vector>

using namespace std;
const bool debugg_BowAICK = false;

BowAICK::BowAICK()
{
	name = "BowAICK";
	nr_iter = 30;
	
	feature_scale = 1;
	
	bow_threshold = 0.2;
	
	distance_threshold = 0.015f * feature_scale;
	feature_threshold = 0.15f;
	shrinking = 0.8f;
	stabilety_threshold = 0.000001f;
	max_points = 100000;
}

BowAICK::~BowAICK(){printf("delete BowAICK\n");}

float BowAICK::getAlpha(int iteration){return 1-pow(shrinking,float(iteration));}

Transformation * BowAICK::getTransformation(RGBDFrame * src, RGBDFrame * dst)
{
	struct timeval start, end;
	gettimeofday(&start, NULL);
	Eigen::Matrix4f transformationMat = Eigen::Matrix4f::Identity();
	
	Transformation * transformation = new Transformation();
	transformation->transformationMatrix = transformationMat;
	transformation->src = src;
	transformation->dst = dst;
	transformation->weight = 100;
	
	if(debugg_BowAICK){printf("BowAICK::getTransformation(%i,%i)\n",src->id,dst->id);}
	IplImage* img_combine;
	int width;
	int height;
	if(debugg_BowAICK)
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

				data[3 * (j * (2*width) + (i)) + 0] = data_src[ind +2];
				data[3 * (j * (2*width) + (i)) + 1] = data_src[ind +2];
				data[3 * (j * (2*width) + (i)) + 2] = data_src[ind +2];
				
			}
		}
		
		//cvNamedWindow("combined image", CV_WINDOW_AUTOSIZE );
		//cvShowImage("combined image", img_combine);
		//cvWaitKey(0);
		cvReleaseImage( &rgb_img_src );
		cvReleaseImage( &rgb_img_dst );
	}
	
//printf("%i\n",__LINE__);
	
	int nr_loop_src = src->keypoints->valid_key_points.size();
	if(nr_loop_src > max_points){nr_loop_src = max_points;}
	int nr_loop_dst = dst->keypoints->valid_key_points.size();
	if(nr_loop_dst > max_points){nr_loop_dst = max_points;}
	
//printf("%i\n",__LINE__);
	vector<KeyPoint * > src_keypoints;
	for(int i = 0; i < nr_loop_src; i++){src_keypoints.push_back(src->keypoints->valid_key_points.at(i));}

	vector<KeyPoint * > dst_keypoints;
	for(int i = 0; i < nr_loop_dst; i++){dst_keypoints.push_back(dst->keypoints->valid_key_points.at(i));}
	int nr_bow = src->keypoints->valid_key_points.at(0)->cluster_distances.size();
	vector<int> * bow = new vector<int>[nr_bow];
	for(int i = 0; i < nr_bow; i++){bow[i] = vector<int>();}
//printf("%i\n",__LINE__);
	for(int i = 0; i < dst_keypoints.size(); i++){
		KeyPoint * kp = dst_keypoints.at(i);
		if(kp->cluster_distance_pairs.size()>0){
			int id = kp->cluster_distance_pairs.at(0).first;
			bow[id].push_back(i);
		}
	}

	vector< vector< int > > possible_matches;
	for(int i = 0; i < src_keypoints.size(); i++){
		possible_matches.push_back(vector< int >());
	}


	for(int i = 0; i < src_keypoints.size(); i++){
		KeyPoint * src_kp = src_keypoints.at(i);
		for(int j = 0; j < src_kp->cluster_distance_pairs.size(); j++){
			float d = src_kp->cluster_distance_pairs.at(j).second;
			int id = src_kp->cluster_distance_pairs.at(j).first;
			//printf("%i %i -> %i %f\n",i,j,id,d);
			if(d < bow_threshold){
				vector<int> vec = bow[id];
				for(int k = 0; k < vec.size(); k++){
					possible_matches.at(i).push_back(vec.at(k));
				}
			}else{break;}
		}
	}
	
	
	int src_nr_points = src_keypoints.size();
	int dst_nr_points = dst_keypoints.size();
	float ** feature_distances = new float*[src_nr_points];
//printf("%i\n",__LINE__);
	for(int i = 0; i < src_nr_points;i++){
		FeatureDescriptor * descriptorA = src_keypoints.at(i)->descriptor;
		int dst_nr_matches = possible_matches.at(i).size();
		feature_distances[i] = new float[dst_nr_matches];		
		for(int j = 0; j < dst_nr_matches;j++)
		{
			FeatureDescriptor * descriptorB = dst_keypoints.at(possible_matches.at(i).at(j))->descriptor;
			feature_distances[i][j] = feature_scale*(descriptorA->distance(descriptorB));
		}
	}

	float * pos_src_x 	= new float[src_nr_points];
	float * pos_src_y 	= new float[src_nr_points];
	float * pos_src_z 	= new float[src_nr_points];
	for(int i = 0; i < src_nr_points; i++)
	{
		pos_src_x[i] = src_keypoints.at(i)->point->x;
		pos_src_y[i] = src_keypoints.at(i)->point->y;
		pos_src_z[i] = src_keypoints.at(i)->point->z;
	}	
	
	float * pos_dst_x 	= new float[dst_nr_points];
	float * pos_dst_y 	= new float[dst_nr_points];
	float * pos_dst_z 	= new float[dst_nr_points];
	for(int i = 0; i < dst_nr_points; i++)
	{
		pos_dst_x[i] = dst_keypoints.at(i)->point->x;
		pos_dst_y[i] = dst_keypoints.at(i)->point->y;
		pos_dst_z[i] = dst_keypoints.at(i)->point->z;
	}
	
	for(int iter = 0; iter < nr_iter; iter++)
	{
	
		float alpha = getAlpha(iter);
		//printf("alpha= %f\n",alpha);
		pcl::TransformationFromCorrespondences tfc;
		float threshold = distance_threshold*alpha + (1 - alpha)*feature_threshold;
		transformation->weight = 0;
		int nr_matches = 0;
		
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
		
		IplImage * img_combine_clone;
		if(debugg_BowAICK){
			img_combine_clone = cvCreateImage(cvSize(img_combine->width, img_combine->height), IPL_DEPTH_8U, 3);
			cvCopy( img_combine, img_combine_clone, NULL );
		}
		
		for(int i = 0; i < src_nr_points;i++)
		{
			float x_tmp = pos_src_x[i];
			float y_tmp = pos_src_y[i];
			float z_tmp = pos_src_z[i];
				
			float x = x_tmp*mat00+y_tmp*mat01+z_tmp*mat02+mat03;
			float y = x_tmp*mat10+y_tmp*mat11+z_tmp*mat12+mat13;
			float z = x_tmp*mat20+y_tmp*mat21+z_tmp*mat22+mat23;

			float dx,dy,dz;
			int dst_nr_matches = possible_matches.at(i).size();
			float best_d = 100000000;
			int best_j = -1;
			for(int jj = 0; jj < dst_nr_matches;jj++)
			{
				int j = possible_matches.at(i).at(jj);
				dx = x-pos_dst_x[j];
				dy = y-pos_dst_y[j];
				dz = z-pos_dst_z[j];

				float d = (1-alpha)*feature_distances[i][jj] + alpha*sqrt(dx*dx + dy*dy + dz*dz);
				if(d < best_d){
					best_d = d;
					best_j = j;
				}
			}
			if(best_d < threshold && best_j != -1){
				KeyPoint * src_kp = src_keypoints.at(i);
				KeyPoint * dst_kp = dst_keypoints.at(best_j);
				if(debugg_BowAICK){
					cvCircle(img_combine_clone,cvPoint(dst_kp->point->w + width	, dst_kp->point->h), 5,cvScalar(0, 255, 0, 0),2, 8, 0);
					cvCircle(img_combine_clone,cvPoint(src_kp->point->w			, src_kp->point->h), 5,cvScalar(0, 255, 0, 0),2, 8, 0);
					cvLine(img_combine_clone,cvPoint(dst_kp->point->w  + width ,dst_kp->point->h),cvPoint(src_kp->point->w,src_kp->point->h),cvScalar(0, 0, 255, 0),1, 8, 0);
				}
				tfc.add(src_kp->point->pos, dst_kp->point->pos);
				if(iter == nr_iter-1){
					transformation->weight++;
					transformation->matches.push_back(make_pair (src_kp, dst_kp));
				}
			}
		}
		
		transformationMat = tfc.getTransformation().matrix();
		transformation->transformationMatrix = transformationMat;
		
		if(debugg_BowAICK ){
			cvShowImage("combined image", img_combine_clone);
			cvWaitKey(0);
			cvReleaseImage( &img_combine_clone);
		}
	}
	
	if(debugg_BowAICK){printf("done\n");cvReleaseImage( &img_combine );}
	
	for(int i = 0; i < src_nr_points;i++){delete[] feature_distances[i];}
	delete[] feature_distances;
	
	delete[] pos_src_x;
	delete[] pos_src_y;
	delete[] pos_src_z;
	
	delete[] pos_dst_x;
	delete[] pos_dst_y;
	delete[] pos_dst_z;
	
	delete[] bow;
	
	
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	//printf("BowAICK cost: %f\n",time);
	return transformation;
}
