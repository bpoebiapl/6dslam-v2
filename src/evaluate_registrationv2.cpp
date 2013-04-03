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
	//printf("stopping thread\n");
}

void test_a_task(test_task * t)
{
	//printf("test_a_task\n");
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
	
	/*
	printf("----------------------------\n");
	printf("Matcher:%s\n",t->matcher->name.c_str());
	printf("trans->weight = %f\n",trans->weight);
	printf("pos_diff = %f trans_diff = %f\n",pos_diff,trans_diff);
	cout << "gt: \n" << true_trans_mat << endl;
	cout << "est: \n" << trans->transformationMatrix << endl;
	cout << "err: \n" << err << endl;
	printf("----------------------------\n");
	*/
}

void * transform_start_test_thread( void *ptr )
{
	int c = 0;
	while(true)
	{
		pthread_mutex_lock( &transform_tasks_mutex );
		if(transform_tasks->size() > 0)
		{
			c++;
			//printf("c: %i\n",c);
			test_task * t = transform_tasks->back();
			transform_tasks->pop_back();
			pthread_mutex_unlock(&transform_tasks_mutex);
			
			test_a_task(t);
	
			long size 				= sizeof(int)*(8);
			char * buffer_char 		= new char[size];
			int * buffer_int 		= (int *)buffer_char;
			float * buffer_float	= (float *)buffer_char;

			string path = out_path+t->filename;
			//printf("%s\n",path.c_str());	
			ifstream task_file (path.c_str());
			if (task_file.is_open()){
				//printf("doing task...\n");
				
				task_file.seekg(0,ifstream::end);
				long size=task_file.tellg();
				char * buffer_char 		= new char [size];
				int * buffer_int 		= (int *) buffer_char;
				float * buffer_float	= (float *) buffer_char;
		
				task_file.seekg(0);
				task_file.read (buffer_char,size);
				task_file.close();
				buffer_int[3] = 1;
				buffer_float[4] = t->result->val_time;
				buffer_float[5] = t->result->pos_error;
				buffer_float[6] = t->result->trans_error;
				buffer_float[7] = t->result->transformation->weight;
				
				buffer_float[8 ] = t->result->transformation->transformationMatrix(0,0);
				buffer_float[9 ] = t->result->transformation->transformationMatrix(0,1);
				buffer_float[10] = t->result->transformation->transformationMatrix(0,2);
				buffer_float[11] = t->result->transformation->transformationMatrix(0,3);
				buffer_float[12] = t->result->transformation->transformationMatrix(1,0);
				buffer_float[13] = t->result->transformation->transformationMatrix(1,1);
				buffer_float[14] = t->result->transformation->transformationMatrix(1,2);
				buffer_float[15] = t->result->transformation->transformationMatrix(1,3);
				buffer_float[16] = t->result->transformation->transformationMatrix(2,0);
				buffer_float[17] = t->result->transformation->transformationMatrix(2,1);
				buffer_float[18] = t->result->transformation->transformationMatrix(2,2);
				buffer_float[19] = t->result->transformation->transformationMatrix(2,3);
				buffer_float[20] = t->result->transformation->transformationMatrix(3,0);
				buffer_float[21] = t->result->transformation->transformationMatrix(3,1);
				buffer_float[22] = t->result->transformation->transformationMatrix(3,2);
				buffer_float[23] = t->result->transformation->transformationMatrix(3,3);
				
				ofstream outfile (path.c_str(),ofstream::binary);
				outfile.write (buffer_char,size);
				outfile.close();
				delete buffer_char;
			}
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
    	//if(i >= size_now){files.resize(size_now+10000);size_now+=files.size();}
        //files.at(i) = string(dirp->d_name);
        files.push_back(string(dirp->d_name));
    }
    //files.resize(i);
    closedir(dp);
    return 0;
}

void test(vector< frame_data * > * all_frames, string dataset, vector<FrameMatcher * > matchers, vector<int> backing)
{
	transform_done_tasks = 0;
	
	vector<vector< Analyzation_data * > > matchers_data;	
	matchers_data.push_back(vector< Analyzation_data * >());
	
	printf("starting test\n");
	for(unsigned int k = 0; k < matchers.size(); k++){
		printf("matcher: %i\n",k);
		for(int i = 0; i < (int)all_frames->size(); i++){
			//printf("frame: %i\n",i);
			for(int j = i-1; j >=  max(0,i-backing.at(k)); j-=1){
				char fpath [150];
				sprintf(fpath,"%s%s_%i_%i_%i.task",out_path.c_str(),dataset.c_str(),i,j,k);
				ifstream task_file (fpath);
				if (!task_file.is_open()){
					long size 				= sizeof(int)*(8+16);
					char * buffer_char 		= new char[size];
					int * buffer_int 		= (int *)buffer_char;
					float * buffer_float	= (float *)buffer_char;
					buffer_int[0] = i;
					buffer_int[1] = j;
					buffer_int[2] = k;
					buffer_int[3] = 0;//done?
					buffer_float[4] = 0;//val_time;
					buffer_float[5] = 0;//pos_error;
					buffer_float[6] = 0;//trans_error;
					buffer_float[7] = 0;//weight;
					
					buffer_float[8] = 0;
					buffer_float[9] = 0;
					buffer_float[10] = 0;
					buffer_float[11] = 0;
					buffer_float[12] = 0;
					buffer_float[13] = 0;
					buffer_float[14] = 0;
					buffer_float[15] = 0;
					
					buffer_float[16] = 0;
					buffer_float[17] = 0;
					buffer_float[18] = 0;
					buffer_float[19] = 0;
					buffer_float[20] = 0;
					buffer_float[21] = 0;
					buffer_float[22] = 0;
					buffer_float[23] = 0;
					
					ofstream outfile (fpath,ofstream::binary);
					outfile.write (buffer_char,size);
					outfile.close();
					delete buffer_char;
				}else{task_file.close();}
			}
		}
	}
	
	printf("matchers setup\n");
	
	vector<string> files = vector<string>();
	getdir(out_path,files);
	int transform_added_tasks = 0;
	
	printf("files.size() = %i\n",files.size());
    for (unsigned int i = 0;i < files.size();i++) {
        if(files.at(i).find(dataset)==0 && files.at(i).find("_") == dataset.length()){
        	ifstream task_file ((out_path+files.at(i)).c_str());
			if (task_file.is_open()){
				task_file.seekg(0,ifstream::end);
				long size=task_file.tellg();
				char * b_char 		= new char [size];
				int * b_int 		= (int *) b_char;
				float * b_float	= (float *) b_char;
				task_file.seekg(0);
				task_file.read (b_char,size);
				task_file.close();
				
				if(rand()%100==0){printf("%f\n",float(i)/float(files.size()));}
				
				if(b_int[3] == 0){
					test_task * t = new test_task();
					t->src = frames->at(b_int[0]);
					t->dst = frames->at(b_int[1]);
					t->matcher = matchers.at(b_int[2]);
					t->matcher_id = b_int[2];
					t->filename = string(files.at(i));
					
					pthread_mutex_lock( &transform_tasks_mutex );
					transform_tasks->push_back(t);
					pthread_mutex_unlock(&transform_tasks_mutex);
					transform_added_tasks++;
				}
				delete b_char;
        	}
        }
    }
    printf("total tasks to be done: %i\n",transform_tasks->size());
	pthread_mutex_lock( &transform_tasks_mutex );
	sort(transform_tasks->begin(),transform_tasks->end(),mycomparison1);
	pthread_mutex_unlock(&transform_tasks_mutex);
	
	struct timeval test_start, test_end;
	gettimeofday(&test_start, NULL);
	printf("transform_added_tasks:%i\n",transform_added_tasks);
	while(transform_nr_done_tasks() < transform_added_tasks){
		gettimeofday(&test_end, NULL);
		printf("%i/%i Time spent: %f\n",transform_nr_done_tasks(),transform_added_tasks,(test_end.tv_sec*1000000+test_end.tv_usec-(test_start.tv_sec*1000000+test_start.tv_usec))/1000000.0f);
		usleep(50000);
	}
}

void analyze(vector< frame_data * > * all_frames, string dataset,vector<FrameMatcher * > matchers,float max_thresh_rot,float max_thresh_pos,int thresh_steps){
	//printf("starting analyze %s\n",dataset.c_str());
	vector<string> files = vector<string>();
	getdir(out_path,files);
	//printf("nr files: %i\n",files.size());
	//printf("%s\n",out_path.c_str());
	vector< vector< vector<Analyzation_data * > * > * > * results = new vector<vector<vector<Analyzation_data * > * > * >();
	for (unsigned int i = 0;i < files.size();i++) {
		//if(i%1000 == 0){printf("%i:%s\n",i,files.at(i).c_str());}
		if(files.at(i).find(dataset)==0 && files.at(i).find("_") == dataset.length()){
			
			ifstream task_file ((out_path+files.at(i)).c_str());
			if (task_file.is_open()){
				task_file.seekg(0,ifstream::end);
				long size =task_file.tellg();
				if(size == 0){/*printf("size == 0\n");*/task_file.close();}
				else{
					char * buffer_char 		= new char [size];
					int * buffer_int 		= (int *) buffer_char;
					float * buffer_float	= (float *) buffer_char;
					task_file.seekg(0);
					task_file.read (buffer_char,size);
					task_file.close();
					int step 			= buffer_int[0] - buffer_int[1];
					int matcher_id		= buffer_int[2];
					while(results->size() <= matcher_id){results->push_back(new vector<vector<Analyzation_data * > * >());}
					while(results->at(matcher_id)->size() <= step){results->at(matcher_id)->push_back(new vector<Analyzation_data * >());}
					if(buffer_int[3] == 1){
						Analyzation_data * data = new Analyzation_data();
						if(all_frames != 0)
						{
							data->src 				= all_frames->at(buffer_int[0])->frame;
							data->dst 				= all_frames->at(buffer_int[1])->frame;
						}
						data->val_time 			= buffer_float[4];
						data->pos_error 		= buffer_float[5];
						data->trans_error 		= buffer_float[6];

						Transformation * transformation = new Transformation();
						transformation->src = data->src;
						transformation->dst = data->dst;
						transformation->weight = buffer_float[7];

						transformation->transformationMatrix(0,0) = buffer_float[8 ];
					 	transformation->transformationMatrix(0,1) = buffer_float[9 ];
						transformation->transformationMatrix(0,2) = buffer_float[10];
						transformation->transformationMatrix(0,3) = buffer_float[11];
						transformation->transformationMatrix(1,0) = buffer_float[12];
						transformation->transformationMatrix(1,1) = buffer_float[13];
						transformation->transformationMatrix(1,2) = buffer_float[14];
						transformation->transformationMatrix(1,3) = buffer_float[15];
						transformation->transformationMatrix(2,0) = buffer_float[16];
						transformation->transformationMatrix(2,1) = buffer_float[17];
						transformation->transformationMatrix(2,2) = buffer_float[18];
						transformation->transformationMatrix(2,3) = buffer_float[19];
						transformation->transformationMatrix(3,0) = buffer_float[20];
						transformation->transformationMatrix(3,1) = buffer_float[21];
						transformation->transformationMatrix(3,2) = buffer_float[22];
						transformation->transformationMatrix(3,3) = buffer_float[23];

						data->transformation = transformation;
						
						results->at(matcher_id)->at(step)->push_back(data);

					}else{
						//printf("not calculated\n");
					}
					delete buffer_char;
				}
	    	}else{printf("failed to open file\n");}
	    } 
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
		printf("%s_%i_%s_avg_time = ",dataset.c_str(),matcher,matchers.at(matcher)->name.c_str());
		double avg_time = 0;
		double total_trans = 0;
		for(int step = 1; step < results->at(matcher)->size(); step++)
		{
			vector<Analyzation_data * > * current = results->at(matcher)->at(step);
			for(int nr = 0; nr < current->size(); nr++){
				Analyzation_data * data = current->at(nr);
				avg_time+=data->val_time;
				total_trans++;
			}
		}
		printf("%f;\n",avg_time/total_trans);
	}
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
	calib1->scale		= 5000;
	calib1->words 		= vector<FeatureDescriptor * >();
	
	OrbExtractor * orb = new OrbExtractor();
	orb->nr_features = 1000;
	orb->calibration = calib1;
	
	SurfExtractor * surf = new SurfExtractor();
	surf->calibration = calib1;
	
	Map3D * map = new Map3Dbase();	
	mymap = map;
	map->segmentation = new RGBDSegmentationDummy();
	map->segmentation->calibration = calib1;
	map->extractor = surf;
	
	
	frames = getFrameData("/home/johane/test_data/rgbd_dataset_freiburg1_room",map,100000);
	
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
	
	float max_thresh_rot = 0.25;
	float max_thresh_pos = 0.1;
	int thresh_steps = 150;
	
	vector<FrameMatcher * > matchers;
	vector<int> backing;
	
	backing.push_back(30);
	matchers.push_back(new BowAICKv2(100));
	backing.push_back(30);
	matchers.push_back(new BowAICKv2(200));
	backing.push_back(30);
	matchers.push_back(new BowAICKv2(300));
	backing.push_back(30);
	matchers.push_back(new BowAICKv2(400));
	backing.push_back(30);
	matchers.push_back(new BowAICKv2(500));
	backing.push_back(30);
	matchers.push_back(new BowAICKv2(1000));
	backing.push_back(30);
	matchers.push_back(new BowAICKv2(200000));
	
	for(int i = 0; i < 12; i++){
		pthread_t mythread;
		pthread_create( &mythread, NULL, transform_start_test_thread, NULL);
	}
	string bow_path;
	vector<FeatureDescriptor * > words;
	/*
	out_path = "/media/D6287F55287F3399/Users/Johan/output2/bowAICKsurf10/";
	bow_path = "bow_output/library_10_1_10_%i.feature.surf";
	words.clear();
	for(int i = 0; i < 10; i++){char buff[250];sprintf(buff,bow_path.c_str(),i);words.push_back(new SurfFeatureDescriptor64(string(buff)));}
	for(int i = 0; i < frames->size(); i++){printf("fixing words for %i\n",i);frames->at(i)->frame->setWords(words,0.2f);}
	test(frames,"bowAICKsurf10",matchers,backing);
	
	out_path = "/media/D6287F55287F3399/Users/Johan/output2/bowAICKsurf100/";
	bow_path = "bow_output/library_100_1_10_%i.feature.surf";
	words.clear();
	for(int i = 0; i < 100; i++){char buff[250];sprintf(buff,bow_path.c_str(),i);words.push_back(new SurfFeatureDescriptor64(string(buff)));}
	for(int i = 0; i < frames->size(); i++){printf("fixing words for %i\n",i);frames->at(i)->frame->setWords(words,0.2f);}
	test(frames,"bowAICKsurf100",matchers,backing);
	
	out_path = "/media/D6287F55287F3399/Users/Johan/output2/bowAICKsurf500/";
	bow_path = "bow_output/library_500_1_10_%i.feature.surf";
	words.clear();
	for(int i = 0; i < 500; i++){char buff[250];sprintf(buff,bow_path.c_str(),i);words.push_back(new SurfFeatureDescriptor64(string(buff)));}
	for(int i = 0; i < frames->size(); i++){printf("fixing words for %i\n",i);frames->at(i)->frame->setWords(words,0.2f);}
	test(frames,"bowAICKsurf500",matchers,backing);
	
	out_path = "/media/D6287F55287F3399/Users/Johan/output2/bowAICKsurf1000/";
	bow_path = "bow_output/library_1000_1_10_%i.feature.surf";
	words.clear();
	for(int i = 0; i < 1000; i++){char buff[250];sprintf(buff,bow_path.c_str(),i);words.push_back(new SurfFeatureDescriptor64(string(buff)));}
	for(int i = 0; i < frames->size(); i++){printf("fixing words for %i\n",i);frames->at(i)->frame->setWords(words,0.2f);}
	test(frames,"bowAICKsurf1000",matchers,backing);
	
	out_path = "/media/D6287F55287F3399/Users/Johan/output2/bowAICKsurf5000/";
	bow_path = "bow_output/library_5000_1_10_%i.feature.surf";
	words.clear();
	for(int i = 0; i < 5000; i++){char buff[250];sprintf(buff,bow_path.c_str(),i);words.push_back(new SurfFeatureDescriptor64(string(buff)));}
	for(int i = 0; i < frames->size(); i++){printf("fixing words for %i\n",i);frames->at(i)->frame->setWords(words,0.2f);}
	test(frames,"bowAICKsurf5000",matchers,backing);
	*/
	out_path = "/media/D6287F55287F3399/Users/Johan/output2/bowAICKsurf10/";
	analyze(frames,"bowAICKsurf10",matchers,max_thresh_rot,max_thresh_pos,thresh_steps);
	out_path = "/media/D6287F55287F3399/Users/Johan/output2/bowAICKsurf100/";
	analyze(frames,"bowAICKsurf100",matchers,max_thresh_rot,max_thresh_pos,thresh_steps);
	out_path = "/media/D6287F55287F3399/Users/Johan/output2/bowAICKsurf500/";
	analyze(frames,"bowAICKsurf500",matchers,max_thresh_rot,max_thresh_pos,thresh_steps);
	out_path = "/media/D6287F55287F3399/Users/Johan/output2/bowAICKsurf1000/";
	analyze(frames,"bowAICKsurf1000",matchers,max_thresh_rot,max_thresh_pos,thresh_steps);
	out_path = "/media/D6287F55287F3399/Users/Johan/output2/bowAICKsurf5000/";
	analyze(frames,"bowAICKsurf1000",matchers,max_thresh_rot,max_thresh_pos,thresh_steps);
	printf("---------------------END---------------------\n");
	
	return 0;
}
