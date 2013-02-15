#include "DistanceNetFilter.h"
#include <map>

using namespace std;
DistanceNetFilter::DistanceNetFilter(){
	max_dist = 0.01;
	counter_threshold = 10;
}
DistanceNetFilter::~DistanceNetFilter(){}
Transformation * DistanceNetFilter::filterTransformation(Transformation * input){
	//printf("DistanceNetFilter:start\n");
	//input->show();
	
	Transformation * transformation = new Transformation();
	transformation->transformationMatrix = input->transformationMatrix;
	transformation->src = input->src;
	transformation->dst = input->dst;
	transformation->weight = 0;
	
	RGBDFrame * src = input->src;
	RGBDFrame * dst = input->dst;
	
	struct timeval start, end;
	gettimeofday(&start, NULL);

	map<KeyPoint *,int> src_map;
	vector< KeyPoint * > src_vec;
	vector<vector<int> > src_matches_id;
	
	map<KeyPoint *,int> dst_map;
	vector< KeyPoint * > dst_vec;
	
	map<KeyPoint *,int>::iterator it;
	pair<map<KeyPoint *,int>::iterator,bool> ret;
	for(int i = 0; i < input->matches.size();i++){
		KeyPoint * src_kp = input->matches.at(i).first;
		KeyPoint * dst_kp = input->matches.at(i).second;
		
		ret=src_map.insert(make_pair(src_kp,src_vec.size()));
		int src_index;
		if (ret.second==false){
			src_index = ret.first->second;
		}else{
			src_index = src_vec.size();
			src_vec.push_back(src_kp);
			src_matches_id.push_back(vector<int>());
		}

		ret=dst_map.insert(make_pair(dst_kp,dst_vec.size()));
		int dst_index;
		if (ret.second==false){
			dst_index = ret.first->second;
		}else{
			dst_index = dst_vec.size();
			dst_vec.push_back(dst_kp);
		}		
		
		src_matches_id.at(src_index).push_back(dst_index);
	}
	//printf("keypoints: %i %i\n",src_vec.size(),dst_vec.size());

	int src_nr_points = src_vec.size();
	int dst_nr_points = dst_vec.size();

	int ** counter = new int*[src_nr_points];
	for(int i = 0; i < src_nr_points; i++){
		counter[i] = new int[dst_nr_points];
		for(int j = 0; j < dst_nr_points; j++){counter[i][j] = 0;}
	}
	
	float ** dst_dist = new float*[dst_nr_points]; 
	for(int i = 0; i < dst_nr_points;i++){
		dst_dist[i] = new float[dst_nr_points]; 
		for(int j = i-1; j >= 0;j--){
			dst_dist[i][j] = dst_dist[j][i] = dst_vec.at(i)->point->distance(dst_vec.at(j)->point);
		}
	}

	int nr_good = 0;
	for(int i = 0; i < src_nr_points;i++){
		//printf("i:%i\n",i);
		int nr_i = src_matches_id.at(i).size();
		for(int j = i-1; j >= 0;j--){
			int nr_j = src_matches_id.at(j).size();
			float src_distance = src_vec.at(i)->point->distance(src_vec.at(j)->point);
			if(src_distance > max_dist){
				vector<int> di = src_matches_id.at(i);
				vector<int> dj = src_matches_id.at(j);
				
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
	for(int i = 0; i < src_nr_points; i++){
		for(int j = 0; j < dst_nr_points; j++){
			if(counter[i][j]>counter_threshold){
				transformation->matches.push_back(make_pair(src_vec.at(i), dst_vec.at(j)));
			}
		}
	}
/*
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
		if(best >= counter_threshold){src_matches[i] = best_id;}
		else{src_matches[i] = -1;}
	}
	
	int * dst_matches = new int[dst_nr_points];
	for(int i = 0; i < dst_nr_points; i++){
		int best = 0;
		int best_id = 0;
		for(int j = 0; j < src_nr_points; j++){
			if(counter[j][i]>best){
				best = counter[j][i];
				best_id = j;
			}
		}
		if(best >= counter_threshold){dst_matches[i] = best_id;}
		else{dst_matches[i] = -1;}
		
	}
	
	pcl::TransformationFromCorrespondences tfc;
	for(int i = 0; i < src_nr_points; i++){
		if(!(src_matches[i] == -1) && (dst_matches[src_matches[i]] == i)){
			tfc.add(src_vec.at(i)->point->pos, dst_vec.at(src_matches[i])->point->pos);
			transformation->weight++;
			transformation->matches.push_back(make_pair(src_vec.at(i), dst_vec.at(src_matches[i])));
		}
	}
	transformation->transformationMatrix = tfc.getTransformation().matrix();

	delete[] src_matches;
	delete[] dst_matches;
	*/
	
	for(int i = 0; i < src_nr_points; i++){delete[] counter[i];}
	delete[] counter;
	
	for(int i = 0; i < dst_nr_points; i++){delete[] dst_dist[i];}
	delete[] dst_dist;
	
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	//printf("net cost: %f\n",time);
	//printf("DistanceNetFilter:end\n");
	return transformation;
}
void DistanceNetFilter::print(){printf("%s\n",name.c_str());}
void DistanceNetFilter::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}
