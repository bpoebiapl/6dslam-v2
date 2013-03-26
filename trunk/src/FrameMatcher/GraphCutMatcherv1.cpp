#include "GraphCutMatcherv1.h"
#include "GraphForCut.h"
#include <vector>

bool comparison_distancev3 (linkData * i,linkData * j) { return (i->distance<j->distance); }

using namespace std;

//GraphCutMatcherv1::GraphCutMatcherv1(){}

GraphCutMatcherv1::GraphCutMatcherv1(){
	feature_limit = 0.075;
	point_to_point_zero = 0.015;
	point_to_plane_zero = 0.01;
	plane_to_plane_zero = 0.05;
}

GraphCutMatcherv1::~GraphCutMatcherv1(){printf("delete GraphCutMatcherv1\n");}

using namespace std;
const bool debugg_GraphCutMatcherv1 = true;

Transformation * GraphCutMatcherv1::getTransformation(RGBDFrame * src, RGBDFrame * dst){
	IplImage* img_combine;
	int width;
	int height;
	if(debugg_GraphCutMatcherv1){	
		IplImage* rgb_img_src 	= cvLoadImage(src->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
		char * data_src = (char *)rgb_img_src->imageData;
		IplImage* rgb_img_dst 	= cvLoadImage(dst->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
		char * data_dst = (char *)rgb_img_dst->imageData;
		
		width = rgb_img_src->width;
		height = rgb_img_src->height;
		
		img_combine = cvCreateImage(cvSize(2*width,height), IPL_DEPTH_8U, 3);
		char * data = (char *)img_combine->imageData;
		for (int j = 0; j < height; j++){
			for (int i = 0; i < width; i++){
				int ind = 3*(640*j+i);
				data[3 * (j * (2*width) + (width+i)) + 0] = data_dst[ind +0];
				data[3 * (j * (2*width) + (width+i)) + 1] = data_dst[ind +1];
				data[3 * (j * (2*width) + (width+i)) + 2] = data_dst[ind +2];

				data[3 * (j * (2*width) + (i)) + 0] = data_src[ind +0];
				data[3 * (j * (2*width) + (i)) + 1] = data_src[ind +1];
				data[3 * (j * (2*width) + (i)) + 2] = data_src[ind +2];
			}
		}
		cvNamedWindow("combined image", CV_WINDOW_AUTOSIZE );
		cvReleaseImage( &rgb_img_src );
		cvReleaseImage( &rgb_img_dst );
	}
	struct timeval start, end;
	gettimeofday(&start, NULL);
	
	int src_nr_points = src->keypoints->valid_key_points.size();
	int dst_nr_points = dst->keypoints->valid_key_points.size();
	
	int src_nr_planes = src->planes->size();
	int dst_nr_planes = dst->planes->size();
	
	vector<KeyPoint * > src_keypoints = src->keypoints->valid_key_points;
	vector<KeyPoint * > dst_keypoints = dst->keypoints->valid_key_points;
	
	vector<Plane *> * src_planes = src->planes;
	vector<Plane *> * dst_planes = dst->planes;
	
	vector<int> src_matches = vector<int>();
	vector<int> dst_matches = vector<int>();
	vector<float> val_matches = vector<float>();
	
	for(int i = 0; i < src_nr_points; i++){
		for(int j = 0; j < dst_nr_points; j++){
			float d = src_keypoints.at(i)->descriptor->distance(dst_keypoints.at(j)->descriptor);
			if(d < feature_limit){
				src_matches.push_back(i);
				dst_matches.push_back(j);
				val_matches.push_back(d);
				
				if(false && debugg_GraphCutMatcherv1){
					Point * i_src = src_keypoints.at(i)->point;
					Point * i_dst = dst_keypoints.at(j)->point;
					cvLine(img_combine,cvPoint(i_src->w ,i_src->h),cvPoint(i_dst->w+width,i_dst->h),cvScalar(0, 0, 255, 0),1, 8, 0);
					cvCircle(img_combine,cvPoint(i_src->w, i_src->h), 		5,	cvScalar(0,255,0, 0),2, 8, 0);
					cvCircle(img_combine,cvPoint(i_dst->w+width,i_dst->h), 	5,	cvScalar(0,255,0, 0),2, 8, 0);
				}
			}
		}
	}

	
	printf("src_matches.size() = %i\n",src_matches.size());
	gettimeofday(&start, NULL);
	vector<GraphEdge> edges;
	edges.resize((src_matches.size()*(src_matches.size()-1))/2+src_nr_planes*src_nr_planes*dst_nr_planes*dst_nr_planes+src_matches.size()*src_nr_planes*dst_nr_planes);
	
	int counter = 0;
	for(int i = 0; i < src_matches.size();i++){
		Point * i_src = src_keypoints.at(src_matches.at(i))->point;
		Point * i_dst = dst_keypoints.at(dst_matches.at(i))->point;
		
		for(int j = 0; j < i;j++){
			Point * j_src = src_keypoints.at(src_matches.at(j))->point;
			Point * j_dst = dst_keypoints.at(dst_matches.at(j))->point;
			
			GraphEdge & tmp = edges.at(counter);
			tmp.value = (point_to_point_zero-fabs(i_src->distance(j_src)-i_dst->distance(j_dst)))/point_to_point_zero;
			tmp.vertexes[0] = i;
			tmp.vertexes[1] = j;
			tmp.nr_vertexes = 2;
			counter++;
		}
	}
	vector<int> src_planes_matches = vector<int>();
	vector<int> dst_planes_matches = vector<int>();
	for(int i = 0; i < src_nr_planes;i++){
		for(int j = 0; j < dst_nr_planes;j++){
			src_planes_matches.push_back(i);
			dst_planes_matches.push_back(j);
			//printf("%i -> planes:%i %i\n",src_planes_matches.size()-1,i,j);
		}
	}
	int plane_start = counter;
	for(int i = 0; i < src_planes_matches.size();i++){
		Plane * src_plane = src->planes->at(src_planes_matches.at(i));
		Plane * dst_plane = dst->planes->at(dst_planes_matches.at(i));
		int total = 0;
		for(int j = 0; j < src_matches.size();j++){
			Point * src_point = src_keypoints.at(src_matches.at(j))->point;
			Point * dst_point = dst_keypoints.at(dst_matches.at(j))->point;
			GraphEdge & tmp = edges.at(counter);
			tmp.value = (point_to_plane_zero-fabs(src_plane->distance(src_point)-dst_plane->distance(dst_point)))/point_to_plane_zero;
			if(tmp.value > 0.0){total++;}
			tmp.vertexes[0] = i+plane_start;
			tmp.vertexes[1] = j;
			tmp.nr_vertexes = 2;
			counter++;
		}
		//printf("%i->%i %i->total:%i\n",i,src_planes_matches.at(i),dst_planes_matches.at(i),total);
	}
	//printf("angles:\n");
	for(int i = 0; i < src_planes_matches.size();i++){
		Plane * src_plane_i = src->planes->at(src_planes_matches.at(i));
		Plane * dst_plane_i = dst->planes->at(dst_planes_matches.at(i));
		for(int j = 0; j < i;j++){
			Plane * src_plane_j = src->planes->at(src_planes_matches.at(j));
			Plane * dst_plane_j = dst->planes->at(dst_planes_matches.at(j));
			double src_angle = src_plane_i->angle(src_plane_j);
			double dst_angle = dst_plane_i->angle(dst_plane_j);
			double diff = fabs(src_angle-dst_angle);
			GraphEdge & tmp = edges.at(counter);
			tmp.value = (plane_to_plane_zero-fabs(src_plane_i->angle(src_plane_j)-dst_plane_i->angle(dst_plane_j)))/plane_to_plane_zero;
			//if(tmp.value>0){printf("%i %i->%f\n",i,j,tmp.value);}
			//printf("%i %i->%f->%f\n",i,j,fabs(src_plane_i->angle(src_plane_j)-dst_plane_i->angle(dst_plane_j)),tmp.value);
			tmp.vertexes[0] = i+plane_start;
			tmp.vertexes[1] = j+plane_start;
			tmp.nr_vertexes = 2;
			counter++;
		}
	}
	edges.resize(counter);
	printf("counter:%i, size: %i\n",counter,edges.size());
	gettimeofday(&end, NULL);
	float setup_time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("setup cost: %f\n",setup_time);
	gettimeofday(&start, NULL);
	GraphForCut test;
	vector<vector<int> * > * seg = test.segment(edges,src_matches.size()+src_planes_matches.size());


	gettimeofday(&end, NULL);
	float cut_time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("cut cost: %f\n",cut_time);
	/*
	GraphEdge * newgraph = new GraphEdge[src_matches.size()*src_matches.size()];
	int current = 0;
	for(int i = 0; i < src_matches.size();i++){
		Point * i_src = src_keypoints.at(src_matches.at(i))->point;
		Point * i_dst = dst_keypoints.at(dst_matches.at(i))->point;
		
		for(int j = 0; j < i;j++){
			Point * j_src = src_keypoints.at(src_matches.at(j))->point;
			Point * j_dst = dst_keypoints.at(dst_matches.at(j))->point;

			newgraph[current].value = (zero_place-fabs(i_src->distance(j_src)-i_dst->distance(j_dst)))/zero_place;
			newgraph[current].vertex1 = i;
			newgraph[current].vertex2 = j;
			current++;
		}
	}

	gettimeofday(&end, NULL);
	float setup_time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("setup cost: %f\n",setup_time);
	gettimeofday(&start, NULL);
	
	delete[] newgraph;

	int * seg_r;
	int * seg_g;
	int * seg_b;

	if(debugg_GraphCutMatcherv1){
		seg_r = new int[src_matches.size()];
		seg_g = new int[src_matches.size()];
		seg_b = new int[src_matches.size()];
		
		seg_r[0] = 255;
		seg_g[0] = 0;
		seg_b[0] = 0;
		
		seg_r[1] = 0;
		seg_g[1] = 255;
		seg_b[1] = 0;
		
		seg_r[2] = 0;
		seg_g[2] = 0;
		seg_b[2] = 255;
	
		seg_r[3] = 255;
		seg_g[3] = 255;
		seg_b[3] = 0;
	
		seg_r[4] = 0;
		seg_g[4] = 255;
		seg_b[4] = 255;
	
		seg_r[5] = 255;
		seg_g[5] = 0;
		seg_b[5] = 255;
	
		for(int i = 6; i < src_matches.size(); i++){
			seg_r[i] = rand()%256;
			seg_g[i] = rand()%256;
			seg_b[i] = rand()%256;
		}
	}
	
	Transformation * transformation = new Transformation();
	transformation->transformationMatrix = Eigen::Matrix4f::Identity();
	transformation->src = src;
	transformation->dst = dst;
	
	vector<int> * s0 = seg->at(0);
	
	for(int ii = 0; ii < s0->size();ii++){
		int i = s0->at(ii);
		int si = 0;
		Point * i_src = src_keypoints.at(src_matches.at(i))->point;
		Point * i_dst = dst_keypoints.at(dst_matches.at(i))->point;
		transformation->matches.push_back(make_pair (src_keypoints.at(src_matches.at(i)), dst_keypoints.at(dst_matches.at(i))));
		if(debugg_GraphCutMatcherv1){
			cvLine(img_combine,cvPoint(i_src->w ,i_src->h),cvPoint(i_dst->w+width,i_dst->h),cvScalar(0, 255, 0, 0),1, 8, 0);
			cvCircle(img_combine,cvPoint(i_src->w, i_src->h), 5,cvScalar(seg_b[si], seg_g[si], seg_r[si], 0),2, 8, 0);
			cvCircle(img_combine,cvPoint(i_dst->w+width,i_dst->h), 5,cvScalar(seg_b[si], seg_g[si], seg_r[si], 0),2, 8, 0);
		}
	}

	transformation->weight = transformation->matches.size();
	if(transformation->weight >= 3){
		pcl::TransformationFromCorrespondences tfc;
		for(int i = 0; i < transformation->weight; i++){tfc.add(transformation->matches.at(i).first->point->pos, transformation->matches.at(i).second->point->pos);}
		transformation->transformationMatrix = tfc.getTransformation().matrix();
	}
*/
	if(debugg_GraphCutMatcherv1){
		//delete[] seg_r;
		//delete[] seg_g;
		//delete[] seg_b;
		printf("done\n");
		cvShowImage("combined image", img_combine);
		cvWaitKey(0);
		cvReleaseImage( &img_combine);
	}

	//return transformation;

	return 0;
}
