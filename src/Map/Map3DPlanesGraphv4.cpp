#include "Map3DPlanesGraphv4.h"
#include "Frame_input.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

pthread_mutex_t map_tasks_mutex = PTHREAD_MUTEX_INITIALIZER;

struct map_task {
	RGBDFrame * src;
	RGBDFrame * dst;
	FrameMatcher * matcher;
	Map3DPlanesGraphv4 * m;
};

vector< map_task * > map_tasks;

int map_tasks_to_do(){
    pthread_mutex_lock(&map_tasks_mutex);
	int val = map_tasks.size();
	pthread_mutex_unlock(&map_tasks_mutex);
	return val;
}

void * map_start_test_thread( void *ptr )
{
	while(true)
	{
		pthread_mutex_lock( &map_tasks_mutex );
		if(map_tasks.size() > 0)
		{
			map_task * task = map_tasks.back();
			map_tasks.pop_back();
			pthread_mutex_unlock(&map_tasks_mutex);
			//RGBDFrame * f= new RGBDFrame(all_input->at(t),mymap->extractor,mymap->segmentation);
			//frames->at(t) = f;//(new RGBDFrame(all_input->at(t),mymap->extractor,mymap->segmentation));
			
			Transformation * t = task->matcher->getTransformation(task->src, task->dst);
			if(t->weight != 0){task->m->transformations.push_back(t);}
			else{delete t;}
			delete task;
			
			//printf("done task...%i -> %i\n",t->src->id,t->dst->id);
			
		}else{
			pthread_mutex_unlock(&map_tasks_mutex);
			usleep(100);
		}
	}
}

bool comparison_Map3DPlanesGraphv4 (Transformation * i,Transformation * j) {
	if(i->src == j->src){return (i->weight<j->weight);}
	else{return (i->src->id<j->src->id);}
}

using namespace std;

Map3DPlanesGraphv4::Map3DPlanesGraphv4(){
	max_backing = 5;
	
	match_limit_close = 10.00;
	w_limit_close = 3.50;
	
	match_limit_loop = 50.00;
	w_limit_loop = 3.80;
	
	estimateIter = 30;
	img_threshold = 0.005;
	smoothing = 0.1;
	render_full = false;
	graphoptimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
	graphoptimizer.setVerbose(false);
	g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(&graphoptimizer,linearSolver);
	graphoptimizer.setSolver(solver_ptr);
	
	for(int i = 0; i < 8; i++){
		pthread_t mythread;
		pthread_create( &mythread, NULL, map_start_test_thread, NULL);
	}
}

Map3DPlanesGraphv4::~Map3DPlanesGraphv4(){}

g2o::SparseOptimizer * Map3DPlanesGraphv4::startNewGraph()
{
	g2o::SparseOptimizer * graph = new g2o::SparseOptimizer;
	graph->setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
	graph->setVerbose(false);
	g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(&graphoptimizer,linearSolver);
	graph->setSolver(solver_ptr);
	return graph;
}

g2o::VertexSE3 * Map3DPlanesGraphv4::getG2OVertex(RGBDFrame * frame)
{
	Eigen::Affine3f eigenTransform(frame->matrix);
	Eigen::Quaternionf eigenRotation(eigenTransform.rotation());
	g2o::SE3Quat poseSE3(
		Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),
		Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
	g2o::VertexSE3 *vertexSE3 = new g2o::VertexSE3();
	vertexSE3->setId(frame->id);
	vertexSE3->estimate() = poseSE3;
	return vertexSE3;
}

g2o::EdgeSE3 * Map3DPlanesGraphv4::getG2OEdge(Transformation * transformation)
{
	Eigen::Affine3f eigenTransform(transformation->transformationMatrix);
	Eigen::Quaternionf eigenRotation(eigenTransform.rotation());
	g2o::SE3Quat transfoSE3(
			Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),
			Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
	g2o::EdgeSE3* edgeSE3 = new g2o::EdgeSE3;	
	edgeSE3->vertices()[0] = graphoptimizer.vertex(transformation->src->id);
	edgeSE3->vertices()[1] = graphoptimizer.vertex(transformation->dst->id);
	edgeSE3->setMeasurement(transfoSE3.inverse());
	edgeSE3->setInverseMeasurement(transfoSE3);
	Eigen::Matrix<double, 6, 6, 0, 6, 6> mat;
	mat.setIdentity(6,6);
	edgeSE3->information() = mat;
	return edgeSE3;
}

void Map3DPlanesGraphv4::addFrame(Frame_input * fi){addFrame(new RGBDFrame(fi,extractor,segmentation));}
void Map3DPlanesGraphv4::addFrame(RGBDFrame * frame){
	Matrix4f pose = Matrix4f::Identity();

	printf("--------------------------------%i--------------------------------\n",frames.size());
	/*
	float best = -1;
	Transformation * best_t = 0;
	
	//IplImage* depth_img = cvLoadImage(frame->input->depth_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	//unsigned short * depth_data	= (unsigned short *)depth_img->imageData;
	UpdateWeightFilter * update = new UpdateWeightFilter();
	//update->depth_data = depth_data;
	
	UpdateWeightFilterv2 * updatev2 = new UpdateWeightFilterv2();
	
	
	
	FilterMatcher * fmclose = new FilterMatcher();
	fmclose->fm = matcher;
	fmclose->thresholds.push_back(match_limit_close);
	
	fmclose->filters.push_back(update);
	fmclose->thresholds.push_back(w_limit_close);
	
	fmclose->filters.push_back(updatev2);
	fmclose->thresholds.push_back(w_limit_close);
	
	
	
	FilterMatcher * fmloop = new FilterMatcher();
	fmloop->fm = loopclosure_matcher;
	fmloop->thresholds.push_back(match_limit_loop);
	
	fmloop->filters.push_back(update);
	fmloop->thresholds.push_back(w_limit_loop);
	
	fmloop->filters.push_back(updatev2);
	fmloop->thresholds.push_back(w_limit_loop);
	*/
	
	for(int i = frames.size(); (i > 0) && ((frames.size() - i) < max_backing); i--){
	
		map_task * current_task = new map_task;
		current_task->src = frame;
		current_task->dst = frames.at(i-1);
		
		FilterMatcher * current_matcher = new FilterMatcher();
		current_task->matcher = current_matcher;
		current_matcher->fm = matcher;
		current_matcher->thresholds.push_back(match_limit_close);
		current_matcher->filters.push_back(new UpdateWeightFilter());
		current_matcher->thresholds.push_back(w_limit_loop);
		current_matcher->filters.push_back(new UpdateWeightFilterv2());
		current_matcher->thresholds.push_back(w_limit_loop);
		current_task->m = this;
		pthread_mutex_lock(&map_tasks_mutex);
		map_tasks.push_back(current_task);
		pthread_mutex_unlock(&map_tasks_mutex);
		/*
		Transformation * t = fmclose->getTransformation(frame, frames.at(i-1));
		if(t->weight > best){
			pose = poses.at(i-1)*t->transformationMatrix;
			best = t->weight;
			best_t = t;
		}
		if(t->weight > w_limit_close){transformations.push_back(t);}
		*/
	}
	//printf("best: %f\n",best);
	//if(best <= w_limit_close && best_t != 0){transformations.push_back(best_t);}
	
	poses.push_back(pose);
	frames.push_back(frame);

	int stop = frames.size()-max_backing-1;
	FeatureDescriptor * frame_desc = frame->image_descriptor;
	
	
	for(int i = 0; i < stop; i++){
		RGBDFrame * test = frames.at(i);
		float d = test->image_descriptor->distance(frame_desc);
		if(d<img_threshold)
		{
			map_task * current_task = new map_task;
			current_task->src = frame;
			current_task->dst = test;
			FilterMatcher * current_matcher = new FilterMatcher();
			current_task->matcher = current_matcher;
			current_matcher->fm = loopclosure_matcher;
			current_matcher->thresholds.push_back(match_limit_loop);
			current_matcher->filters.push_back(new UpdateWeightFilter());
			current_matcher->thresholds.push_back(w_limit_loop);
			current_matcher->filters.push_back(new UpdateWeightFilterv2());
			current_matcher->thresholds.push_back(w_limit_loop);
			current_task->m = this;
			pthread_mutex_lock(&map_tasks_mutex);
			map_tasks.push_back(current_task);
			pthread_mutex_unlock(&map_tasks_mutex);
			/*
			Transformation * t = fmloop->getTransformation(frame, test);
			if(t->weight > w_limit_loop)	{
				printf("add %i %i\n",i,frames.size());
				transformations.push_back(t);}
			else					{delete t;}
			*/
		}
	}
	//delete update;
	//cvReleaseImage( &depth_img );
	
	while(map_tasks_to_do() > 0){usleep(1000);}
}
void Map3DPlanesGraphv4::addTransformation(Transformation * transformation){transformations.push_back(transformation);}
void Map3DPlanesGraphv4::estimate(){
	graphoptimizer.clear();
	graphoptimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
	graphoptimizer.setVerbose(false);
	g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(&graphoptimizer,linearSolver);
	graphoptimizer.setSolver(solver_ptr);
	
	printf("estimate\n");
	//printf("nr frames: %i nr trans: %i\n",frames.size(),transformations.size());
	for(int i  = 0; i < frames.size(); i++){
		Eigen::Affine3f eigenTransform(poses.at(i));
		Eigen::Quaternionf eigenRotation(eigenTransform.rotation());
		g2o::SE3Quat poseSE3(Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
		g2o::VertexSE3 * vertexSE3 = new g2o::VertexSE3();
		vertexSE3->setId(frames.at(i)->id);
		vertexSE3->estimate() = poseSE3;
		graphoptimizer.addVertex(vertexSE3);
		frames.at(i)->g2oVertex = vertexSE3;
		if(i == 0){vertexSE3->setFixed(true);}
	}
	
	for(int i  = 0; i < transformations.size(); i++){
		Transformation * transformation = transformations.at(i);
		//if((transformation->weight >= w_limit_close))
		{
			Eigen::Affine3f eigenTransform(transformation->transformationMatrix);
			Eigen::Quaternionf eigenRotation(eigenTransform.rotation());
			g2o::SE3Quat transfoSE3(Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
			g2o::EdgeSE3* edgeSE3 = new g2o::EdgeSE3;	
			edgeSE3->vertices()[0] = graphoptimizer.vertex(transformation->src->id);
			edgeSE3->vertices()[1] = graphoptimizer.vertex(transformation->dst->id);
			edgeSE3->setMeasurement(transfoSE3.inverse());
			edgeSE3->setInverseMeasurement(transfoSE3);
			Eigen::Matrix<double, 6, 6, 0, 6, 6> mat;
			mat.setIdentity(6,6);
			edgeSE3->information() = mat;
			graphoptimizer.addEdge(edgeSE3);
		}
	}
	
	for(int i  = 1; i < poses.size(); i++){
		Eigen::Affine3f eigenTransform(Eigen::Matrix4f::Identity());
		Eigen::Quaternionf eigenRotation(eigenTransform.rotation());
		g2o::SE3Quat transfoSE3(Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
		g2o::EdgeSE3* edgeSE3 = new g2o::EdgeSE3;	
		edgeSE3->vertices()[0] = graphoptimizer.vertex(i-1);
		edgeSE3->vertices()[1] = graphoptimizer.vertex(i);
		edgeSE3->setMeasurement(transfoSE3.inverse());
		edgeSE3->setInverseMeasurement(transfoSE3);
		Eigen::Matrix<double, 6, 6, 0, 6, 6> mat;
		mat.setIdentity(6,6);
		edgeSE3->information() = mat*smoothing;
		graphoptimizer.addEdge(edgeSE3);
	}
	
	graphoptimizer.initializeOptimization();
	graphoptimizer.setVerbose(true);
	graphoptimizer.optimize(estimateIter);
	for(int i  = 0; i < frames.size(); i++){
		g2o::VertexSE3 * vertexSE3_src = (g2o::VertexSE3*)(graphoptimizer.vertex(frames.at(i)->id));
		poses.at(i) = (vertexSE3_src->estimate().to_homogenious_matrix()).cast<float>();
		//cout<<i<<endl<<poses.at(i)<<endl;
	}
	printf("estimate done\n");
}
void Map3DPlanesGraphv4::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}

//bool goToNext = false;
//void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,void* viewer_void){goToNext = true;}

void Map3DPlanesGraphv4::visualize(){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	if(!render_full){
		cloud->width    = poses.size();
		cloud->height   = 1;
		cloud->points.resize (cloud->width);
		cloud->width = 0;
		for(int i = 0; i < poses.size(); i+=1){
			int randr = 0;rand()%256;
			int randg = 255;rand()%256;
			int randb = 0;rand()%256;
			Eigen::Matrix4f m1 = poses.at(i);
			cloud->points[cloud->width].x = m1(0,3);
			cloud->points[cloud->width].y = m1(1,3);
			cloud->points[cloud->width].z = m1(2,3);
			printf("%f %f %f\n",cloud->points[cloud->width].x,cloud->points[cloud->width].y,cloud->points[cloud->width].z);
			cloud->points[cloud->width].r = randr;
			cloud->points[cloud->width].g = randg;
			cloud->points[cloud->width].b = randb;
			cloud->width++;
		}
	}else{
		int step = poses.size()/200;
		if(step<1){step = 1;}
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		tmpcloud->width    = 640*480;
		tmpcloud->height   = 1;
		tmpcloud->points.resize (tmpcloud->width);
	
		cloud->width    = 0;
		cloud->height   = 1;
		//cloud->points.resize (640*480*200);
	
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpcloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
		for(int i = 0; i < frames.size(); i+=step){
			RGBDFrame * frame = frames.at(i);
			tmpcloud->width = 0;
			IplImage* rgb_img 	= cvLoadImage(frame->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
			IplImage* depth_img = cvLoadImage(frame->input->depth_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
		
			float d_scaleing	= frame->input->calibration->ds/frame->input->calibration->scale;
			float centerX		= frame->input->calibration->cx;
			float centerY		= frame->input->calibration->cy;
			float invFocalX		= 1.0f/frame->input->calibration->fx;
			float invFocalY		= 1.0f/frame->input->calibration->fy;
		
			char * rgb_data		= (char *)rgb_img->imageData;
			unsigned short * depth_data	= (unsigned short *)depth_img->imageData;
		
			int pixelstep = 10;
			for(int w = pixelstep; w < 640; w+=pixelstep){
				for(int h = pixelstep; h < 480; h+=pixelstep){
					int ind = 640*h+w;
					float x = 0;
					float y = 0;
					float z = float(depth_data[ind]) * d_scaleing;
				
					int r = char(rgb_data[3*ind+2]);
					int g = char(rgb_data[3*ind+1]);
					int b = char(rgb_data[3*ind+0]);
				
					if(r < 0){r = 255+r;}
					if(g < 0){g = 255+g;}
					if(b < 0){b = 255+b;}

					if(z > 0){
						x = (w - centerX) * z * invFocalX;
					   	y = (h - centerY) * z * invFocalY;
					   	
					   	tmpcloud->points[tmpcloud->width].x = x;
						tmpcloud->points[tmpcloud->width].y = y;
						tmpcloud->points[tmpcloud->width].z = z;
						tmpcloud->points[tmpcloud->width].r = float(r);
						tmpcloud->points[tmpcloud->width].g = float(g);
						tmpcloud->points[tmpcloud->width].b = float(b);
						tmpcloud->width++;
					}
				}
			}
		
			cvReleaseImage( &rgb_img );
			cvReleaseImage( &depth_img );
		
			pcl::transformPointCloud (*tmpcloud, *tmpcloud2, poses.at(i));
		
			cloud->points.resize (cloud->width+tmpcloud2->width);
		
			for(int j = 0; j < tmpcloud2->width*tmpcloud2->height; j++)
			{
				cloud->points[cloud->width].x = tmpcloud2->points[j].x;
				cloud->points[cloud->width].y = tmpcloud2->points[j].y;
				cloud->points[cloud->width].z = tmpcloud2->points[j].z;
				cloud->points[cloud->width].r = tmpcloud2->points[j].r;
				cloud->points[cloud->width].g = tmpcloud2->points[j].g;
				cloud->points[cloud->width].b = tmpcloud2->points[j].b;
				cloud->width++;
			}
		}
	}
	//viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->removePointCloud("sample cloud");
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
	
	usleep(100);
	show = true;
	while (show){
		viewer->spinOnce (100);
		//boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		int key = cvWaitKey( 15 );
		if( key != -1 ){printf("stop show\n");break;}
		//printf("key: %i\n",key);
	}
	
}

