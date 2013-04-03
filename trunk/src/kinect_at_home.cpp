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

vector< Frame_input * > * all_input;

pthread_mutex_t tasks_mutex = PTHREAD_MUTEX_INITIALIZER;
list<int > tasks;

pthread_mutex_t done_tasks_mutex = PTHREAD_MUTEX_INITIALIZER;
int done_tasks = 0;

int nr_done_tasks(){
    pthread_mutex_lock(&done_tasks_mutex);
	int val = done_tasks;
	pthread_mutex_unlock(&done_tasks_mutex);
	return val;
}

vector< RGBDFrame * > * frames;

int angle_switch_value = 0;
int angleInt = 0;
int scale_switch_value = 0;
int scaleInt = 0;

bool recalc_frames = true;
bool recalc_transformations = true;
bool recalc_estimate = true;
bool recalc_rendering = true;




boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_global;
Map3D * mymap;

/////////////////////////////////////////////

void * start_test_thread( void *ptr )
{
	while(true)
	{
		pthread_mutex_lock( &tasks_mutex );
		if(tasks.size() > 0)
		{
			int t = tasks.front();
			tasks.pop_front();
			pthread_mutex_unlock(&tasks_mutex);
			RGBDFrame * f= new RGBDFrame(all_input->at(t),mymap->extractor,mymap->segmentation);
			printf("t:%i->%i\n",t,f->id);
			frames->at(t) = f;//(new RGBDFrame(all_input->at(t),mymap->extractor,mymap->segmentation));
			
			pthread_mutex_lock(&done_tasks_mutex);
			done_tasks++;
			pthread_mutex_unlock(&done_tasks_mutex);
		}else{
			pthread_mutex_unlock(&tasks_mutex);
			usleep(1000);
		}
	}
}

void switch_callback_w_limit_close( int position ){
	((Map3DPlanesGraphv4 *)mymap)->w_limit_close = 10*float(position)/float(1000);
	printf("position %i -> close threshold = %f\n",position,10*float(position)/float(1000));
}

void switch_callback_w_limit_loop( int position ){
	((Map3DPlanesGraphv4 *)mymap)->w_limit_loop = 10*float(position)/float(1000);
	printf("position %i -> loop threshold = %f\n",position,10*float(position)/float(1000));
}

/////////////////////////////////////////////

void switch_callback_match_limit_close( int position ){
	((Map3DPlanesGraphv4 *)mymap)->match_limit_close = float(position);
	printf("position %i -> match close threshold = %f\n",position,float(position));
}

void switch_callback_match_limit_loop( int position ){
	((Map3DPlanesGraphv4 *)mymap)->match_limit_loop = float(position);
	printf("position %i -> match loop threshold = %f\n",position,float(position));
}

/////////////////////////////////////////////

void switch_callback_smoothing( int position ){
	((Map3DPlanesGraphv4 *)mymap)->smoothing = 1*float(position)/float(1000);
	printf("position %i -> smoothing = %f\n",position,1*float(position)/float(1000));
}


void switch_callback_img_threshold( int position ){
	
	((Map3DPlanesGraphv4 *)mymap)->img_threshold = float(position)/float(10000);
	printf("position %i -> img_threshold = %f\n",position,float(position)/float(10000));
}



void switch_callback_aick_iter( int position ){
	((BowAICK *)(mymap->loopclosure_matcher))->nr_iter = position;
	printf("bowaick iter %i\n",position);
}

void switch_callback_aick_max_points( int position ){
	((BowAICK *)(mymap->loopclosure_matcher))->max_points = position;
	printf("bowaick max_points %i\n",position);
}

void switch_callback_aick_shrink( int position ){
	((BowAICK *)(mymap->loopclosure_matcher))->shrinking = float(position)/float(1000);
	printf("bowaick shrinking %i\n",float(position)/float(1000));
}

void switch_callback_iter( int position ){
	((Map3DPlanesGraphv4 *)mymap)->estimateIter = position;
	printf("estimateIter %i\n",position);
}


void switch_callback_render_full( int position ){
	if(position == 1){
		((Map3DPlanesGraphv4 *)mymap)->render_full = true;
		printf("render full\n");
	}else{
		((Map3DPlanesGraphv4 *)mymap)->render_full = false;
		printf("dont render full\n");
	}
}

void switch_callback_redo_estimation( int position )	{
	if(position == 1){
		recalc_estimate = true;
		printf("redo Estimation\n");
	}else{
		recalc_estimate = false;
	}
}

void switch_callback_redo_transformation( int position ){
	if(position == 1){
		recalc_transformations = true;
		printf("redo transformations\n");
	}else{
		recalc_transformations = false;
	}
}

void switch_callback_redo_rendering( int position ){
	if(position == 1){
		recalc_rendering = true;
		printf("redo rendering\n");
	}else{
		recalc_rendering = false;
	}
}

vector< Frame_input * > * getFrameInput(string path,int start, int max, Calibration * calibration){
	vector< Frame_input * > * all_inp = new vector< Frame_input * >();
	char fpath [150];
	for(int i = start; i <= start+max; i+=1){
		Frame_input * fi =  new Frame_input();
		sprintf(fpath,"%s/rgb_frames/frame_%05i.png",path.c_str(),i);
		printf("----------------------%i---------------------\n",i);
		printf("%s\n",fpath);
		fi->rgb_path 			= string(fpath);
		sprintf(fpath,"%s/depth_frames/depth_%05i.png",path.c_str(),i);
		printf("%s\n",fpath);
		fi->depth_path			= string(fpath);
		fi->calibration			= calibration;
		all_inp->push_back(fi);
	}
	return all_inp;
}

void * viewerThread( void *ptr )
{
	while(true)
	{
		viewer_global->spinOnce (100);
		usleep(100);
	}
}


int main(int argc, char **argv)
{
	printf("--------------------START--------------------\n");
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	//viewer->setBackgroundColor (255.5f, 255.5f, 255.5f);
	//viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	viewer->setBackgroundColor (1, 1, 1);
	
	string bow_path = "output/bowTest7_%i.feature.surf";
	//string bow_path = "output/bowTest_%i.feature.ORB";
	
	vector<FeatureDescriptor * > words;
	/*
	for(int i = 0; i < 300; i++){
		char buff[50];
		sprintf(buff,"output/bowTest_%i.feature.ORB",i);
		words.push_back(new OrbFeatureDescriptor(string(buff)));
	}
	*/
	for(int i = 0; i < 500; i++){
		char buff[250];
		sprintf(buff,bow_path.c_str(),i);
		words.push_back(new SurfFeatureDescriptor64(string(buff)));
	}
	
	Calibration * calib0 = new Calibration();
	calib0->fx			= 525.0;
	calib0->fy			= 525.0;
	calib0->cx			= 319.5;
	calib0->cy			= 239.5;
	calib0->ds			= 3*1;
	calib0->scale		= 5000;
	calib0->words 		= words;
	
	OrbExtractor * orb = new OrbExtractor();
	orb->nr_features = 1000;
	orb->calibration = calib0;
	SurfExtractor * surf = new SurfExtractor();
	surf->calibration = calib0;
	surf->thres *= 1.0;
	
	
	//Map3D * map = new Map3Dbase();
	//Map3D * map = new Map3Dbow(bow_path);
	//Map3D * map = new Map3DbaseGraph();
	//Map3D * map = new Map3DPlanesGraph();
	//Map3D * map = new Map3DPlanesGraphv3();
	Map3D * map = new Map3DPlanesGraphv4();
	
	mymap = map;
	
	AICK * aick = new AICK();
	aick->max_points 			= 400;
	aick->distance_threshold	= 0.01f;
	aick->feature_threshold		= 0.2f;
	aick->nr_iter				= 10;
	aick->shrinking				= 0.5;
	
	BowAICKv2 * bowaick				= new BowAICKv2();
	bowaick->max_points				= 200;
	bowaick->distance_threshold		= 0.01f;
	bowaick->nr_iter				= 10;
	bowaick->shrinking				= 0.55;
	//bowaick->bow_threshold			= 0.17;
	//bowaick->feature_scale			= 0.5;
	//bowaick->feature_threshold		*= bowaick->feature_scale;
	//bowaick->feature_threshold		*= 2;
	
	DNET * dnet = new DNET();
	

	
	MultiFilterMatcher * mlf = new MultiFilterMatcher();
	mlf->addFilter(new PixelDistanceFilter());
	//mlf->addFilter(new BowFilter());
	mlf->addFilter(new FeatureDistanceFilter());
	mlf->addFilter(new DistanceNetFilter());
	mlf->addFilter(new DistanceNetFilter());
	mlf->addFilter(new DistanceNetFilter());
	mlf->addFilter(new DistanceNetFilter());
	mlf->addFilter(new RansacFilter());
	
	

	MultiLayerMatcher * mlm = new MultiLayerMatcher();
	mlm->addMatcher(aick);
	mlm->addMatcher(mlf);
	mlm->addMatcher(new FrameMatcher());


	map->matcher = aick;//new AICK();//new DistanceNetMatcherv5(1,500, 0.01, 1.96*0.01, true, 0.02, true, 0.01f);//aick;//mlm;//bowaick;//mlf;//mlm;
	map->loopclosure_matcher = aick;//new DistanceNetMatcherv5(1,500, 0.01, 1.96*0.01, true, 0.02, true, 0.01f);//aick;
	map->segmentation = new RGBDSegmentationBase();
	//map->segmentation = new RGBDSegmentationTest();
	//map->segmentation = new RGBDSegmentationPCL();
	map->segmentation = new RGBDSegmentationDummy();
	map->segmentation->calibration = calib0;
	map->extractor = surf;
	map->setVisualization(viewer);
	viewer_global = viewer;
	
	//all_input = getFrameInput("/home/johane/LibraryRecsForJohan/45743f89-6014-48bc-937c-0acfbae19d93",1, 100,calib0);
	//all_input = getFrameInput("/home/johane/LibraryRecsForJohan/6e079fee-6c5f-42ec-b1d5-cb01cea5dedd",001, 5,calib0);
	//all_input = getFrameInput("/home/johane/alper_office",001, 1226,calib0);
	//vector< Frame_input * > * all_input = getFrameInput("/home/johane/LibraryRecsForJohan/6e079fee-6c5f-42ec-b1d5-cb01cea5dedd",1, 1500,calib0);
	//all_input = getFrameInput("/home/johane/LibraryRecsForJohan/f6ab52d2-57c5-4ced-a53b-42a34dacc7a2",1000, 500,calib0);
	//vector< Frame_input * > * all_input = getFrameInput("/home/johane/johan_cvap_run",1000, 500,calib0);


	//all_input = getFrameInput("/home/johane/alper_office",1, 100,calib0);

	//vector< Frame_input * > * all_input = getFrameInput("/home/johane/johan_cvap_run",1, 4000,calib0);
	//all_input = getFrameInput("/home/johane/johan_cvap_run",1450, 1000,calib0);
	//all_input = getFrameInput("/home/johane/johan_cvap_run",800, 50,calib0);
	
	all_input = getFrameInput("/home/johane/office1",1, 300,calib0);
	//vector< Frame_input * > * all_input = getFrameInput("/home/johane/johan_cvap_run",850, 2000,calib0);
	//vector< Frame_input * > * all_input = getFrameInput("/home/johane/johan_cvap_run",1250+1400, 500,calib0);
	//vector< Frame_input * > * all_input = getFrameInput("/home/johane/johan_cvap_run",3977, 1,calib0);
	//vector< Frame_input * > * all_input = getFrameInput("/home/johane/johan_cvap_run",3960, 40,calib0);
	//vector< Frame_input * > * all_input = getFrameInput("/home/johane/johan_cvap_run",3977, 1,calib0);
	




	cvNamedWindow( "Transformations", 1 );
//	cvCreateTrackbar( "aick iter", "Transformations", &angle_switch_value, 100, switch_callback_aick_iter);
//	cvSetTrackbarPos( "aick iter", "Transformations", int(bowaick->nr_iter));
	
//	cvCreateTrackbar( "aick shrink", "Transformations", &angle_switch_value, 1000, switch_callback_aick_shrink);
//	cvSetTrackbarPos( "aick shrink", "Transformations", int(bowaick->shrinking*1000));
	
//	cvCreateTrackbar( "aick max_points", "Transformations", &angle_switch_value, 5000, switch_callback_aick_max_points);
//	cvSetTrackbarPos( "aick max_points", "Transformations", int(bowaick->max_points));
	
	cvCreateTrackbar( "w_limit_close", "Transformations", &angle_switch_value, 1000, switch_callback_w_limit_close );
	cvSetTrackbarPos( "w_limit_close", "Transformations", int(((Map3DPlanesGraphv4 *)mymap)->w_limit_close * 100.0));
	
	cvCreateTrackbar( "match_limit_close", "Transformations", &angle_switch_value, 1000, switch_callback_match_limit_close );
	cvSetTrackbarPos( "match_limit_close", "Transformations", int(((Map3DPlanesGraphv4 *)mymap)->match_limit_close));
	
	cvCreateTrackbar( "w_limit_loop", "Transformations", &angle_switch_value, 1000, switch_callback_w_limit_loop );
	cvSetTrackbarPos( "w_limit_loop", "Transformations", int(((Map3DPlanesGraphv4 *)mymap)->w_limit_loop * 100.0));
	
	cvCreateTrackbar( "match_limit_loop", "Transformations", &angle_switch_value, 1000, switch_callback_match_limit_loop );
	cvSetTrackbarPos( "match_limit_loop", "Transformations", int(((Map3DPlanesGraphv4 *)mymap)->match_limit_loop));
	
//	cvCreateTrackbar( "img_threshold", "Transformations", &angle_switch_value, 10000, switch_callback_img_threshold);
//	cvSetTrackbarPos( "img_threshold", "Transformations", int(((Map3DPlanesGraphv4 *)mymap)->img_threshold * 10000.0));
	
	cvCreateTrackbar( "redo transformations", "Transformations", &angle_switch_value, 1, switch_callback_redo_transformation);
	cvSetTrackbarPos( "redo transformations", "Transformations", 0);
	
	
	cvNamedWindow( "Estimation", 1 );
	cvCreateTrackbar( "Iterations", "Estimation", &scale_switch_value, 200, switch_callback_iter );
	cvSetTrackbarPos( "Iterations", "Estimation",((Map3DPlanesGraphv4 *)mymap)->estimateIter);
	
	cvCreateTrackbar( "smoothing", "Estimation", &angle_switch_value, 10000, switch_callback_smoothing);
	cvSetTrackbarPos( "smoothing", "Estimation", int(((Map3DPlanesGraphv4 *)mymap)->smoothing * 1000.0));
	
	cvCreateTrackbar( "redo estimation", "Estimation", &angle_switch_value, 1, switch_callback_redo_estimation);
	cvSetTrackbarPos( "redo estimation", "Estimation", 0);
	
	cvNamedWindow( "Rendering", 1 );
	cvCreateTrackbar( "path/full", "Rendering", &scale_switch_value, 1, switch_callback_render_full);
	if(((Map3DPlanesGraphv4 *)mymap)->render_full)	{cvSetTrackbarPos( "path/full", "Rendering",1);}
	else											{cvSetTrackbarPos( "path/full", "Rendering",0);}
	cvCreateTrackbar( "redo rendering", "Rendering", &angle_switch_value, 1, switch_callback_redo_rendering);


	cvNamedWindow( "dummy", 1 );
	
	
	
	frames = new vector< RGBDFrame * > ();
	struct timeval start, end;
	
	recalc_frames = true;
	recalc_transformations = true;
	recalc_estimate = true;
	recalc_rendering = true;
	

	for(int i = 0; i < 11; i++){
		pthread_t mythread;
		pthread_create( &mythread, NULL, start_test_thread, NULL);
	}
	
	while(true){
		cvWaitKey(0);
		gettimeofday(&start, NULL);
		if(recalc_frames){
			recalc_frames = false;
			struct timeval test_start, test_end;
			done_tasks = 0;
			gettimeofday(&test_start, NULL);
			
			frames->clear();
			tasks.clear();
			int added_tasks = 0;
			for(int i = 0; i < all_input->size(); i+=1){
				frames->push_back(NULL);
				tasks.push_back(i);
				added_tasks++;
				usleep(1000);
			}

			while(nr_done_tasks() < added_tasks){
				gettimeofday(&test_end, NULL);
				float test_time = (test_end.tv_sec*1000000+test_end.tv_usec-(test_start.tv_sec*1000000+test_start.tv_usec))/1000000.0f;
				usleep(500000);
			}
		}
		if(recalc_transformations){
			map->poses.clear();
			map->frames.clear();
			map->transformations.clear();

			for(int i = 0; i < frames->size(); i+=1){printf("i:%i\n",i);map->addFrame(frames->at(i));}
		}
		if(recalc_estimate){map->estimate();}
		if(recalc_rendering){
			map->show = false;
			map->visualize();
		}
		
		recalc_frames = false;
		recalc_transformations = false;
		recalc_estimate = false;
		recalc_rendering = false;
		
		gettimeofday(&end, NULL);
		float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
		printf("Task cost: %f\n",time);
		cvSetTrackbarPos( "redo estimation", "Estimation", 0);
		cvSetTrackbarPos( "redo rendering", "Rendering", 0);
		cvSetTrackbarPos( "redo transformations", "Transformations", 0);
	}
	printf("---------------------END---------------------\n");
	return 0;
}
