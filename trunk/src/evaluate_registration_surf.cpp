#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <pthread.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

#include "Map/Map3D.h"
#include "graph/graph.h"
#include "FeatureDescriptor/FeatureDescriptor.h"
#include "core/core.h"
#include "FeatureExtractor/FeatureExtractor.h"
#include "mygeometry/mygeometry.h"
#include "FrameMatcher/FrameMatcher.h"
#include "TransformationFilter/TransformationFilter.h"
#include "RGBDSegmentation/RGBDSegmentation.h"

using namespace std;
using namespace Eigen;

string out_path = "output4/";

struct groundtruth {
	double timestamp;
	double tx;
	double ty;
	double tz;
	double qx;
	double qy;
	double qz;
	double qw;
};

struct timestamp_path {
	double timestamp;
	string path;
};

struct frame_data {
	timestamp_path * rgb;
	timestamp_path * depth;
	groundtruth * gt;
	Frame_input * frame_input;
	RGBDFrame * frame;
};

struct Analyzation_data {
	float val_time;
	float pos_error;
	float trans_error;
	RGBDFrame * src;
	RGBDFrame * dst;
	Transformation * transformation;
};

struct test_task {
	frame_data * src;
	frame_data * dst;
	FrameMatcher * matcher;
	int matcher_id;
	string filename;
	Analyzation_data * result;
};

vector< Frame_input * > * all_input;
////////////
pthread_mutex_t tasks_mutex = PTHREAD_MUTEX_INITIALIZER;
list<int > tasks;

pthread_mutex_t done_tasks_mutex = PTHREAD_MUTEX_INITIALIZER;
int done_tasks = 0;

////////////
pthread_mutex_t transform_tasks_mutex = PTHREAD_MUTEX_INITIALIZER;
vector<test_task * > * transform_tasks;

pthread_mutex_t transform_done_tasks_mutex = PTHREAD_MUTEX_INITIALIZER;
int transform_done_tasks = 0;

////////////
int nr_done_tasks(){
    pthread_mutex_lock(&done_tasks_mutex);
	int val = done_tasks;
	pthread_mutex_unlock(&done_tasks_mutex);
	return val;
}

////////////
int transform_nr_done_tasks(){
    pthread_mutex_lock(&transform_done_tasks_mutex);
	int transform_val = transform_done_tasks;
	pthread_mutex_unlock(&transform_done_tasks_mutex);
	return transform_val;
}

vector< frame_data * > * frames;

Map3D * mymap;
////////////

bool mycomparison1 (test_task * i,test_task * j) {return (i->src->frame->id - i->dst->frame->id) > (j->src->frame->id - j->dst->frame->id);}

/////////////////////////////////////////////

void * start_test_thread( void *ptr ){
	bool run = true;
	while(run){
		pthread_mutex_lock( &tasks_mutex );
		if(tasks.size() > 0)
		{
			int t = tasks.front();
			tasks.pop_front();
			pthread_mutex_unlock(&tasks_mutex);
			frames->at(t)->frame = new RGBDFrame(frames->at(t)->frame_input,mymap->extractor,mymap->segmentation);
			//usleep(100000);
			pthread_mutex_lock(&done_tasks_mutex);
			done_tasks++;
			pthread_mutex_unlock(&done_tasks_mutex);
		}else{
			pthread_mutex_unlock(&tasks_mutex);
			usleep(100000);
			//run = false;
		}
	}
}

void test_a_task(test_task * t)
{
	frame_data * src_fd = t->src;
	frame_data * dst_fd = t->dst;
	struct timeval start, end;
	gettimeofday(&start, NULL);
	Transformation * trans = t->matcher->getTransformation(src_fd->frame, dst_fd->frame);
	gettimeofday(&end, NULL);
	float eval_time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;

	g2o::SE3Quat src_poseSE3 (Eigen::Quaterniond(src_fd->gt->qw, src_fd->gt->qx, src_fd->gt->qy, src_fd->gt->qz), Eigen::Vector3d(src_fd->gt->tx, src_fd->gt->ty, src_fd->gt->tz));
	g2o::SE3Quat dst_poseSE3 (Eigen::Quaterniond(dst_fd->gt->qw, dst_fd->gt->qx, dst_fd->gt->qy, dst_fd->gt->qz), Eigen::Vector3d(dst_fd->gt->tx, dst_fd->gt->ty, dst_fd->gt->tz));

	Eigen::Matrix4f src_mat = src_poseSE3.to_homogenious_matrix().cast<float>();
	Eigen::Matrix4f dst_mat = dst_poseSE3.to_homogenious_matrix().cast<float>();

	Eigen::Matrix4f true_trans_mat = dst_mat.inverse()*src_mat;
	if(trans == NULL){
		trans = new Transformation();
		trans->src = src_fd->frame;
		trans->dst = dst_fd->frame;
		trans->transformationMatrix = true_trans_mat;
		trans->weight = 100000;
	}

	Eigen::Matrix4f err = true_trans_mat.inverse() * trans->transformationMatrix;
	err(0,0) -= 1;
	err(1,1) -= 1;
	err(2,2) -= 1;
	err(3,3) -= 1;

	float dx = err(0,3);
	float dy = err(1,3);
	float dz = err(2,3);
	float pos_diff = sqrt(dx*dx+dy*dy+dz*dz);
	float trans_diff = 0;
	for(int q = 0; q < 3; q++){for(int l = 0; l < 3; l++){trans_diff+= fabs(err(q,l));}}

	Analyzation_data * data = new Analyzation_data();
	data->val_time			= eval_time;
	data->pos_error			= pos_diff;
	data->trans_error		= trans_diff;
	data->src 				= src_fd->frame;
	data->dst 				= dst_fd->frame;
	data->transformation	= trans;
	t->result = data;
}

void * transform_start_test_thread( void *ptr )
{
	int c = 0;
	while(true)
	{
		pthread_mutex_lock( &transform_tasks_mutex );
		if(transform_tasks->size() > 0)
		{
			test_task * t = transform_tasks->back();
			transform_tasks->pop_back();
			pthread_mutex_unlock(&transform_tasks_mutex);
			
			test_a_task(t);
			
			pthread_mutex_lock(&transform_done_tasks_mutex);
			transform_done_tasks++;
			pthread_mutex_unlock(&transform_done_tasks_mutex);
		}else{
			pthread_mutex_unlock(&transform_tasks_mutex);
			usleep(1000);
		}
	}
}


vector< frame_data * > * getFrameData(string path,Map3D * map, int max){
	string path_gt		= path+"/groundtruth.txt";
	string path_rgb		= path+"/rgb.txt";
	string path_depth	= path+"/depth.txt";
	string path_asso	= path+"/asso_w_gt.txt";

	printf("starting setup\n");
	string line;
	ifstream asso_file (path_asso.c_str());
	vector<frame_data *> frame_inputs;
	vector<FeatureDescriptor *> * centers = new vector<FeatureDescriptor *>();
	vector< frame_data * > * all_frames = new vector< frame_data * >();
	if (asso_file.is_open()){
		int counter = 0;
		while ( asso_file.good() && counter < max){
			counter++;
			getline (asso_file,line);
			if(line[0] != '#'){
				int space1 = line.find(" ");
				if(space1 != -1){
					int space2  = line.find(" ",space1+1,1);
					int space3  = line.find(" ",space2+1,1);
					int space4  = line.find(" ",space3+1,1);
					
					int space5  = line.find(" ",space4+1,1);
					int space6  = line.find(" ",space5+1,1);
					int space7  = line.find(" ",space6+1,1);
					int space8  = line.find(" ",space7+1,1);
					int space9  = line.find(" ",space8+1,1);
					int space10 = line.find(" ",space9+1,1);
					int space11 = line.find(" ",space10+1,1);
					int space12 = line.find(" ",space11+1,1);
					
					timestamp_path * tp_depth 	= new timestamp_path();
					tp_depth->timestamp 		= atof(line.substr(0,space1).c_str());
					tp_depth->path				= path+"/"+line.substr(space1+1,space2-space1-1).c_str();
					
					timestamp_path * tp_rgb 	= new timestamp_path();
					tp_rgb->timestamp 			= atof(line.substr(space2+1,space3-space2-1).c_str());
					tp_rgb->path				= path+"/"+line.substr(space3+1,space4-space3-1).c_str();
					
					//printf("%s %s\n",tp_depth->path.c_str(),tp_rgb->path.c_str());
					
					groundtruth * gt 			= new groundtruth();
					gt->timestamp 				= atof(line.substr(space4+1 ,space5 -space4 -1).c_str());
					gt->tx 						= atof(line.substr(space5+1 ,space6 -space5 -1).c_str());
					gt->ty 						= atof(line.substr(space6+1 ,space7 -space6 -1).c_str());
					gt->tz 						= atof(line.substr(space7+1 ,space8 -space7 -1).c_str());
					gt->qx 						= atof(line.substr(space8+1 ,space9 -space8 -1).c_str());
					gt->qy 						= atof(line.substr(space9+1 ,space10-space9 -1).c_str());
					gt->qz 						= atof(line.substr(space10+1,space11-space10-1).c_str());
					gt->qw 						= atof(line.substr(space11+1,space12-space11-1).c_str());
					
					Frame_input * fi 			=  new Frame_input();
					fi->depth_timestamp			= tp_depth->timestamp;
					fi->depth_path				= tp_depth->path;
					fi->rgb_timestamp			= tp_rgb->timestamp;
					fi->rgb_path 				= tp_rgb->path;
					fi->calibration				= map->extractor->calibration;
					
					frame_data * fd				= new frame_data();
					fd->rgb 					= tp_rgb;
					fd->depth 					= tp_depth;
					fd->gt 						= gt;
					fd->frame_input				= fi;
					fd->frame					= 0;//new RGBDFrame(fi,map);
					all_frames->push_back(fd);
				}
			}
		}
		asso_file.close();
	}else{cout << "Unable to open " << path_asso;}
	return all_frames;
}

int getdir (string dir, vector<string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }
    int size_now = 0;
    int i;
    for(i = 0; (dirp = readdir(dp)) != NULL; i++) {
        files.push_back(string(dirp->d_name));
    }
    closedir(dp);
    return 0;
}

void test(vector< frame_data * > * all_frames, string dataset, vector<FrameMatcher * > matchers, vector<int> backing)
{
	transform_done_tasks = 0;
	int transform_added_tasks = 0;
		
	pthread_mutex_lock( &transform_tasks_mutex );
	vector<test_task * > transform_tasks_now;
	for(unsigned int k = 0; k < matchers.size(); k++){
		printf("matcher: %i\n",k);
		for(int i = 0; i < (int)all_frames->size(); i++){
			for(int j = i-1; j >=  max(0,i-backing.at(k)); j-=1){
				test_task * t = new test_task();
				t->src = frames->at(i);
				t->dst = frames->at(j);
				t->matcher = matchers.at(k);
				t->matcher_id = k;
				transform_tasks->push_back(t);
				transform_tasks_now.push_back(t);
				transform_added_tasks++;
			}
		}
	}
    printf("total tasks to be done: %i\n",transform_tasks->size());
	pthread_mutex_unlock(&transform_tasks_mutex);
	
	struct timeval test_start, test_end;
	gettimeofday(&test_start, NULL);
	while(transform_nr_done_tasks() < transform_added_tasks){
		gettimeofday(&test_end, NULL);
		printf("%i/%i Time spent: %f\n",transform_nr_done_tasks(),transform_added_tasks,(test_end.tv_sec*1000000+test_end.tv_usec-(test_start.tv_sec*1000000+test_start.tv_usec))/1000000.0f);
		sleep(10);
		//usleep(5000000);
	}

	float max_thresh_rot = 0.25;
	float max_thresh_pos = 0.1;
	int thresh_steps = 150;
	
	vector< vector< vector<Analyzation_data * > * > * > * results = new vector<vector<vector<Analyzation_data * > * > * >();
	for(int i = 0; i < matchers.size(); i++){
		results->push_back(new vector<vector<Analyzation_data * > * >());
		results->back()->push_back(new vector<Analyzation_data * >());
		for(int j = 0; j < backing.at(i); j++){
			results->back()->push_back(new vector<Analyzation_data * >());
		}
	}
	for(int i = 0; i < transform_tasks_now.size(); i++){
		test_task * t = transform_tasks_now.at(i);
		int step = abs(t->src->frame->id - t->dst->frame->id);
		int matcher_id = t->matcher_id;
		results->at(matcher_id)->at(step)->push_back(t->result);
	}
	
	printf("thresholds = [ ");
	for(float thresh_counter = 1; thresh_counter <= thresh_steps; thresh_counter++)
	{
		printf("%.5f ", max_thresh_pos*float(thresh_counter)/float(thresh_steps));
	}
	printf("];\n");
	
	for(int matcher = 0; matcher < results->size(); matcher++)
	{
		printf("%s_%i_%s_mat_pos = [ ",dataset.c_str(),matcher,matchers.at(matcher)->name.c_str());

		for(int step = 1; step < results->at(matcher)->size(); step++)
		{
			vector<Analyzation_data * > * current = results->at(matcher)->at(step);
			
			for(float thresh_counter = 1; thresh_counter <= thresh_steps; thresh_counter++)
			{
				float thresh = max_thresh_pos*float(thresh_counter)/float(thresh_steps);
				float nr_good = 0;
				float nr_bad = 0;
				for(int nr = 0; nr < current->size(); nr++)
				{
					Analyzation_data * data = current->at(nr);
					if(data->pos_error < thresh){nr_good++;}
					else						{nr_bad++;}
				}
				if(nr_good+nr_bad == 0){
					printf("%.5f ",0.0f);
				}else{
					printf("%.5f ",float(nr_good)/float(nr_good+nr_bad));
				}
			}
			printf("; ");
		}
		printf("];\n");
		printf("%s_%i_%s_avg_time = [",dataset.c_str(),matcher,matchers.at(matcher)->name.c_str());

		for(int step = 1; step < results->at(matcher)->size(); step++)
		{
			double avg_time = 0;
			double total_trans = 0;
			vector<Analyzation_data * > * current = results->at(matcher)->at(step);
			for(int nr = 0; nr < current->size(); nr++){
				Analyzation_data * data = current->at(nr);
				avg_time+=data->val_time;
				total_trans++;
			}
			printf("%.5f ",avg_time/total_trans);
		}
		printf("];\n");
	}
}

BowAICKv2 * current_matcher;

void switch_callback_iter( int position )				{current_matcher->nr_iter = position;}
void switch_callback_max_points( int position )			{current_matcher->max_points = position;}
void switch_callback_shrinking( int position )			{current_matcher->shrinking = 0.01*float(position);}
void switch_callback_scaling( int position )			{current_matcher->feature_scale = 0.001*float(position);}
void switch_callback_feature_threshold( int position )	{current_matcher->feature_threshold = 0.0001*float(position);}
void switch_callback_distance_threshold( int position )	{current_matcher->distance_threshold = 0.0001*float(position);}
void switch_callback_bow_threshold( int position )		{
	string bow_path = "bow_output/library_1000_1_10_%i.feature.orb";
	vector<FeatureDescriptor * > words;
	words.clear();
	for(int i = 0; i < 1000; i++){char buff[250];sprintf(buff,bow_path.c_str(),i);words.push_back(new OrbFeatureDescriptor(string(buff)));}
	for(int i = 0; i < frames->size(); i++){printf("updating wrods for %i\n",i);frames->at(i)->frame->setWords(words,0.0001*float(position));}
}

int main(int argc, char **argv)
{
	printf("--------------------START--------------------\n");
	Calibration * calib1 = new Calibration();
	calib1->fx			= 517.3;
	calib1->fy			= 516.5;
	calib1->cx			= 318.6;
	calib1->cy			= 255.3;
	calib1->ds			= 1.035;
	calib1->scale			= 5000;
	calib1->words 			= vector<FeatureDescriptor * >();

	SurfExtractor * surf = new SurfExtractor();
	surf->calibration = calib1;

	Map3D * map = new Map3Dbase();
	mymap = map;
	map->segmentation = new RGBDSegmentationDummy();
	map->segmentation->calibration = calib1;
	map->extractor = surf;
	frames = getFrameData("/home/johane/test_data/rgbd_dataset_freiburg1_room",map,400000);

	struct timeval test_start, test_end;
	done_tasks = 0;
	gettimeofday(&test_start, NULL);

	tasks.clear();
	int added_tasks = frames->size();
	for(int i = 0; i < frames->size(); i++){tasks.push_back(i);}
	
	transform_tasks = new vector<test_task * >();
	for(int i = 0; i < 10; i++){
		pthread_t mythread;
		pthread_create( &mythread, NULL, start_test_thread, NULL);
	}

	while(nr_done_tasks() < added_tasks){
		gettimeofday(&test_end, NULL);
		printf("%f %i/%i\n",(test_end.tv_sec*1000000+test_end.tv_usec-(test_start.tv_sec*1000000+test_start.tv_usec))/1000000.0f,nr_done_tasks(),added_tasks);
		usleep(500000);
	}
	gettimeofday(&test_end, NULL);
	printf("%f %i/%i\n",(test_end.tv_sec*1000000+test_end.tv_usec-(test_start.tv_sec*1000000+test_start.tv_usec))/1000000.0f,nr_done_tasks(),added_tasks);
	
	for(int i = 0; i < 12; i++){
		pthread_t mythread;
		pthread_create( &mythread, NULL, transform_start_test_thread, NULL);
	}
	
	vector<FrameMatcher * > matchers;
	vector<int> backing;
	string bow_path;
	vector<FeatureDescriptor * > words;
	
	matchers.clear();
	backing.clear();

	matchers.push_back(new AICK(10000,30,0.8));
	backing.push_back(30);

	matchers.push_back(new AICK(200,5,0.3));
	backing.push_back(30);

        test(frames,"originalAICKsurf",matchers,backing);

        matchers.clear();
        backing.clear();

        matchers.push_back(new BowAICKv2(10000,30,0.8));
        backing.push_back(30);

        matchers.push_back(new BowAICKv2(200,5,0.3));
        backing.push_back(30);

        bow_path = "bow_output/library_1000_1_10_%i.feature.surf";
        words.clear();
        for(int i = 0; i < 1000; i++){char buff[250];sprintf(buff,bow_path.c_str(),i);words.push_back(new SurfFeatureDescriptor64(string(buff)));}

        for(int i = 0; i < frames->size(); i++){frames->at(i)->frame->setWords(words,0.15f);}
        test(frames,"bowAICKsurf_bl_0_15",matchers,backing);

        for(int i = 0; i < frames->size(); i++){frames->at(i)->frame->setWords(words,0.2f);}
        test(frames,"bowAICKsurf_bl_0_2",matchers,backing);

        for(int i = 0; i < frames->size(); i++){frames->at(i)->frame->setWords(words,0.25f);}
	test(frames,"bowAICKsurf_bl_0_25",matchers,backing);

        for(int i = 0; i < frames->size(); i++){frames->at(i)->frame->setWords(words,0.3f);}
        test(frames,"bowAICKsurf_bl_0_3",matchers,backing);

        for(int i = 0; i < frames->size(); i++){frames->at(i)->frame->setWords(words,0.35f);}
        test(frames,"bowAICKsurf_bl_0_35",matchers,backing);

	printf("---------------------END---------------------\n");
	return 0;
}
