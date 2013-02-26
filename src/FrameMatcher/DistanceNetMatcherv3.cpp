#include "DistanceNetMatcherv3.h"
#include <vector>

bool comparison_distancev3 (linkData * i,linkData * j) { return (i->distance<j->distance); }

using namespace std;

DistanceNetMatcherv3::DistanceNetMatcherv3(){
	name = "DistanceNetMatcherv3";
	max_points = 200;
	nr_iter = 15;
	std_dist = 0.01;
	lookup_size = 100000;
	bounds = 1.96*std_dist;
	probIgnore = 0.001;
	
	movement_prior = true;
	movementPerFrame = 0.02f;
	feature_prior = true;
	feature_smoothing = 0.01f;
	
	lookup = new float[lookup_size];
	lookup_step = 2*bounds/float(lookup_size+1);
	float lookup_size_half = float(lookup_size)/2;
	float norm = 1/(std_dist*sqrt(2*3.14));
	for(int i = 0; i < lookup_size; i++){
		float tmp = (i-lookup_size_half)*lookup_step;
		lookup[i] = norm*exp(-0.5*(tmp*tmp)/(std_dist*std_dist));
	}
}

DistanceNetMatcherv3::DistanceNetMatcherv3(int iter_,int max_points_, float std_dist_, float bounds_, bool movement_prior_, float movementPerFrame_, bool feature_prior_, float feature_smoothing_){
	name = "DistanceNetMatcherv3";
	max_points = max_points_;
	nr_iter = iter_;
	std_dist = std_dist_;
	lookup_size = 100000;
	bounds = bounds_;
	probIgnore = 0.001;
	
	movement_prior = movement_prior_;
	movementPerFrame = movementPerFrame_;
	feature_prior = feature_prior_;
	feature_smoothing = feature_smoothing_;
	
	lookup = new float[lookup_size];
	lookup_step = 2*bounds/float(lookup_size+1);
	float lookup_size_half = float(lookup_size)/2;
	float norm = 1/(std_dist*sqrt(2*3.14));
	for(int i = 0; i < lookup_size; i++){
		float tmp = (i-lookup_size_half)*lookup_step;
		lookup[i] = norm*exp(-0.5*(tmp*tmp)/(std_dist*std_dist));
	}
}

DistanceNetMatcherv3::~DistanceNetMatcherv3(){printf("delete DistanceNetMatcherv3\n");}

using namespace std;
const bool debugg_DistanceNetMatcherv3 = false;

Transformation * DistanceNetMatcherv3::getTransformation(RGBDFrame * src, RGBDFrame * dst)
{
	printf("getting transform: %i %i\n",src->id,dst->id);
	IplImage* img_combine;
	int width;
	int height;
	if(debugg_DistanceNetMatcherv3)
	{	
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
	
	float frame_dist = fabs(src->id - dst->id);
	float movement = (frame_dist*movementPerFrame)*(frame_dist*movementPerFrame);
	int src_nr_points = src->keypoints->valid_key_points.size();
	if(src_nr_points > max_points){src_nr_points = max_points;}
	int dst_nr_points = dst->keypoints->valid_key_points.size();
	if(dst_nr_points > max_points){dst_nr_points = max_points;}
	
	vector<KeyPoint * > src_keypoints;
	for(int i = 0; i < src_nr_points; i++){src_keypoints.push_back(src->keypoints->valid_key_points.at(i));}
	vector<KeyPoint * > dst_keypoints;
	for(int i = 0; i < dst_nr_points; i++){dst_keypoints.push_back(dst->keypoints->valid_key_points.at(i));}
	
	float ** current_p = new float*[src_nr_points];
	float * src_likelihood = new float[src_nr_points];
	float ** counter = new float*[src_nr_points];

	

	for(int i = 0; i < src_nr_points;i++){
		current_p[i] = new float[dst_nr_points];
		counter[i] = new float[dst_nr_points];
	}
	
	Point * zero_p = new Point();
	for(int i = 0; i < src_nr_points;i++){
		Point * src_p = src_keypoints.at(i)->point;
		float src_radius = src_p->distance(zero_p);
		for(int j = 0; j < dst_nr_points;j++){
			Point * dst_p = dst_keypoints.at(j)->point;
			float dst_radius = dst_p->distance(zero_p);
			float radius_diff = src_radius-dst_radius;
			float d = src_keypoints.at(i)->descriptor->distance(dst_keypoints.at(j)->descriptor);
			current_p[i][j] = 1;
			if(feature_prior)	{current_p[i][j] /= feature_smoothing+d;}
			if(movement_prior)	{current_p[i][j] *=exp(-0.5*(radius_diff*radius_diff)/movement);}
			counter[i][j] = 0;
		}
	}
	delete zero_p;
	
	for(int i = 0; i < src_nr_points;i++){
			float sum = 0;
			for(int j = 0; j < dst_nr_points;j++){sum+= current_p[i][j];}
			if(sum!=0){for(int j = 0; j < dst_nr_points;j++){current_p[i][j] = current_p[i][j]/sum;}}
	}
	
	vector< linkData * > dst_links_vector = vector< linkData * >();
	for(int i = 0; i < dst_nr_points;i++){

		for(int j = i-1; j >= 0;j--){
			linkData * tmp = new linkData;
			tmp->distance = dst_keypoints.at(i)->point->distance(dst_keypoints.at(j)->point);
			tmp->id1 = i;
			tmp->id2 = j;
			dst_links_vector.push_back(tmp);
		}

	}
	sort(dst_links_vector.begin(),dst_links_vector.end(),comparison_distancev3);
	int dst_links_total = dst_links_vector.size();
	linkData** dst_links_sort = new linkData*[dst_links_total]; 
	for(int i = 0; i < dst_links_total; i++){dst_links_sort[i] = dst_links_vector.at(i);}
	 
	vector< linkData * > src_links_vector = vector< linkData * >();
	for(int i = 0; i < src_nr_points;i++){
		for(int j = i-1; j >= 0;j--){
			linkData * tmp = new linkData;
			tmp->distance = src_keypoints.at(i)->point->distance(src_keypoints.at(j)->point);
			tmp->id1 = i;
			tmp->id2 = j;
			src_links_vector.push_back(tmp);
		}
	}
	sort(src_links_vector.begin(),src_links_vector.end(),comparison_distancev3);
	
	int src_links_total = src_links_vector.size();
	linkData** src_links_sort = new linkData*[src_links_total];
	for(int i = 0; i < src_links_total; i++){src_links_sort[i] = src_links_vector.at(i);}
	
	struct timeval start, end;
	gettimeofday(&start, NULL);
	
		
	vector<linkPair* > linkPair_vec;
	const int vlen = 10000;
	linkPair_vec.push_back(new linkPair[vlen]);
	
	int cvpos = 0;
	linkPair * cvec = linkPair_vec.back();
	float minusbounds = -bounds;

	int last_start = 0;
	
	for(int i = 0; i < src_links_total;i++){
		linkData * src_ld = src_links_sort[i];
		float src_distance = src_ld->distance;
		int src_id1 = src_ld->id1;
		int src_id2 = src_ld->id2;
		float * current_p_1 = current_p[src_id1];
		float * current_p_2 = current_p[src_id2];
		float * counter_1 = counter[src_id1];
		float * counter_2 = counter[src_id2];		
		for(int j = last_start; j < dst_links_total; j++){
			linkData * dst_ld = dst_links_sort[j];	
			float diff_distance = src_distance - dst_ld->distance;
			int dst_id1 = dst_ld->id1;
			int dst_id2 = dst_ld->id2;	
						
			if(diff_distance > minusbounds){
				if(diff_distance < bounds){
					linkPair & lp = cvec[cvpos];
					lp.src_link = src_ld;
					lp.dst_link = dst_ld;
					lp.value = lookup[int((0.5f*((diff_distance/bounds)+1.0f))*lookup_size)];
					cvpos++;
					if(cvpos == vlen){
					 	linkPair_vec.push_back(new linkPair[vlen]);
						cvpos = 0;
						cvec = linkPair_vec.back();
					}
				}else{last_start++;}
			}else{break;}
		}
	}

	gettimeofday(&end, NULL);
	float setup1_time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("setup1 cost: %f\n",setup1_time);
	gettimeofday(&start, NULL);

	for(int iter = 0; iter < nr_iter; iter++){
		gettimeofday(&start, NULL);
		for(int i = 0; i < src_nr_points;i++){for(int j = 0; j < dst_nr_points;j++){counter[i][j] = 0;}}
		linkPair * cvec;
		linkPair lp;
		for(int i = 0; i < linkPair_vec.size()-1; i++){
			cvec = linkPair_vec.at(i);
			for(int j = 0; j < vlen; j++){
				lp = cvec[j];
				float val = lp.value;
				int src_id1 = lp.src_link->id1;
				int src_id2 = lp.src_link->id2;
				int dst_id1 = lp.dst_link->id1;
				int dst_id2 = lp.dst_link->id2;
				counter[src_id1][dst_id1]+=val;
				counter[src_id1][dst_id2]+=val;
				counter[src_id2][dst_id1]+=val;
				counter[src_id2][dst_id2]+=val;
			}
		}
		cvec = linkPair_vec.back();
		for(int j = 0; j < cvpos; j++){
			linkPair lp = cvec[j];
			float val = lp.value;
			int src_id1 = lp.src_link->id1;
			int src_id2 = lp.src_link->id2;
			int dst_id1 = lp.dst_link->id1;
			int dst_id2 = lp.dst_link->id2;
			counter[src_id1][dst_id1]+=val;
			counter[src_id1][dst_id2]+=val;
			counter[src_id2][dst_id1]+=val;
			counter[src_id2][dst_id2]+=val;
		}
		
		for(int i = 0; i < src_nr_points;i++)
		{
			float sum = 0;
			for(int j = 0; j < dst_nr_points;j++){sum+= counter[i][j];}
			src_likelihood[i] = sum;
			if(sum!=0){
				for(int j = 0; j < dst_nr_points;j++){current_p[i][j] = counter[i][j]/sum;}
			}
		}
		gettimeofday(&end, NULL);
		float time1 = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
		if(debugg_DistanceNetMatcherv3){printf("net cost: %f\n",time1);}
	}

	vector<int> src_matches = vector<int>();
	vector<int> dst_matches = vector<int>();
	

	gettimeofday(&start, NULL);
		
	for(int i = 0; i < src_nr_points;i++){
		float best_v = -1;
		int best_i = 0;
		for(int j = 0; j < dst_nr_points;j++){
			if(best_v < current_p[i][j]){
				best_v = current_p[i][j];
				best_i = j;
			}
		}
		src_matches.push_back(i);
		dst_matches.push_back(best_i);
	}
	
	float zero_place = 0.025f;
	float ** graph = new float*[src_matches.size()];
	int * segment_id = new int[src_matches.size()];
	for(int i = 0; i < src_matches.size();i++){
		graph[i] = new float[src_matches.size()];
		segment_id[i]= -1;

		Point * i_src = src_keypoints.at(src_matches.at(i))->point;
		Point * i_dst = dst_keypoints.at(dst_matches.at(i))->point;
		
		for(int j = 0; j < src_matches.size();j++){
			if(i==j){graph[i][j] = 0;}
			else{
				Point * j_src = src_keypoints.at(src_matches.at(j))->point;
				Point * j_dst = dst_keypoints.at(dst_matches.at(j))->point;
			
				float d1 = i_src->distance(j_src);
				float d2 = i_dst->distance(j_dst);
				float diff = fabs(d1-d2);

				float v = (zero_place-diff)/zero_place;
				if(v < -5){v = -5;}
				graph[i][j] = v;
			}
		}
	}
	
	int current_segment_id = 0;
	vector< int > current_segment = vector< int >();
	for(int i = 0; i < src_matches.size(); i++){current_segment.push_back(i);}
	while(current_segment.size()>0){
		vector< float > current_score = vector< float >();
		for(int i = 0; i < current_segment.size(); i++){
			int current_i = current_segment.at(i);
			float sum = 0;
			for(int j = 0; j < current_segment.size(); j++){sum += graph[current_i][current_segment.at(j)];}
			current_score.push_back(sum);
		}
		vector< int > next_segment = vector< int >();
		while(current_segment.size() > 1){
			float worst = current_score.at(0);
			int worst_index_val = current_segment.at(0);
			int worst_index = 0;
			for(int i = 1; i < current_score.size(); i++){
				if(worst > current_score.at(i)){
					worst = current_score.at(i);
					worst_index_val = current_segment.at(i);
					worst_index = i;
				}
			}
			if(worst <= 0){
				for(int i = 0; i < current_segment.size(); i++){current_score.at(i) -= graph[worst_index_val][current_segment.at(i)];}
				current_score.at(worst_index) 	= current_score.back();		current_score.pop_back();
				current_segment.at(worst_index) = current_segment.back();	current_segment.pop_back();
				next_segment.push_back(worst_index_val);
			}else{break;}
		}
		for(int i = 0; i < current_segment.size(); i++){segment_id[current_segment.at(i)] = current_segment_id;}
		current_segment_id++;
		current_segment = next_segment;
	}
	
	gettimeofday(&end, NULL);
	float segment_time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("segment1 cost: %f\n",segment_time);
	
	int * seg_r;
	int * seg_g;
	int * seg_b;

	if(debugg_DistanceNetMatcherv3){
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
	
	for(int i = 0; i < src_matches.size();i++){
		Point * i_src = src_keypoints.at(src_matches.at(i))->point;
		Point * i_dst = dst_keypoints.at(dst_matches.at(i))->point;
		int si = segment_id[i];
		if(si == 0){transformation->matches.push_back(make_pair (src_keypoints.at(src_matches.at(i)), dst_keypoints.at(dst_matches.at(i))));}
		if(debugg_DistanceNetMatcherv3){
			if(si == 0){
				cvLine(img_combine,cvPoint(i_src->w ,i_src->h),cvPoint(i_dst->w+width,i_dst->h),cvScalar(0, 255, 0, 0),1, 8, 0);
				cvCircle(img_combine,cvPoint(i_src->w, i_src->h), 5,cvScalar(seg_b[si], seg_g[si], seg_r[si], 0),2, 8, 0);
				cvCircle(img_combine,cvPoint(i_dst->w+width,i_dst->h), 5,cvScalar(seg_b[si], seg_g[si], seg_r[si], 0),2, 8, 0);
			}
		}
	}

	transformation->weight = transformation->matches.size();
	if(transformation->weight >= 3){
		pcl::TransformationFromCorrespondences tfc;
		for(int i = 0; i < transformation->weight; i++){tfc.add(transformation->matches.at(i).first->point->pos, transformation->matches.at(i).second->point->pos);}
		transformation->transformationMatrix = tfc.getTransformation().matrix();
	}

	if(debugg_DistanceNetMatcherv3){
		delete[] seg_r;
		delete[] seg_g;
		delete[] seg_b;
		printf("done\n");
		cvShowImage("combined image", img_combine);
		cvWaitKey(0);
		cvReleaseImage( &img_combine);
	}

	//CLEAN
	
	for(int i = 0; i < dst_links_total; i++){delete dst_links_sort[i];}
	for(int i = 0; i < src_links_total; i++){delete src_links_sort[i];}
	delete[] dst_links_sort;
	delete[] src_links_sort;
	
	for(int i = 0; i < src_nr_points;i++){
		delete[] current_p[i];
		delete[] counter[i];
	}
	delete[] current_p;
	delete[] counter;

	for(int i = 0; i < src_matches.size();i++){ delete[] graph[i];}
	delete[] graph;
	delete[] segment_id;
	
	for(int i = 0; i < linkPair_vec.size(); i++)		{delete[] linkPair_vec.at(i);}	
	return transformation;
}
