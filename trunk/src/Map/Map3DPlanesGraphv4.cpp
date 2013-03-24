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

#include "VertexPlane.cpp"
#include "EdgeSe3Plane.cpp"
#include "EdgeSe3Plane2.cpp"
#include "EdgePlane.cpp"



pthread_mutex_t map_tasks_mutex = PTHREAD_MUTEX_INITIALIZER;

pthread_mutex_t transformation_mutex = PTHREAD_MUTEX_INITIALIZER;

struct map_task {
	RGBDFrame * src;
	RGBDFrame * dst;
	FrameMatcher * matcher;
	Map3DPlanesGraphv4 * m;
	float weight_limit;
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
			//printf("map_tasks:%i\n",map_tasks.size());
			map_task * task = map_tasks.back();
			map_tasks.pop_back();
			pthread_mutex_unlock(&map_tasks_mutex);
			
			Transformation * t = task->matcher->getTransformation(task->src, task->dst);
			if(t->weight > task->weight_limit){
				printf("closing loop: %i to %i\n",task->src->id,task->dst->id);
				//vector< vector<Transformation *> * > * tmat =  task->m->transformations_mat;
				//tmat->at(task->src->id)->push_back(t);
				pthread_mutex_lock( &transformation_mutex );
				task->m->transformations.push_back(t);
				pthread_mutex_unlock( &transformation_mutex );
			}
			else{delete t;}
			delete task;	
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
	transformations_mat = new vector< vector<Transformation *> * >();
	max_backing = 1;
	
	match_limit_close = 30.00;
	w_limit_close = 3.70;
	
	match_limit_loop = 5.00;
	w_limit_loop = 3.20;
	
	estimateIter = 150;
	img_threshold = 0.00025;
	smoothing = 0.1;
	render_full = true;
	graphoptimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
	graphoptimizer.setVerbose(false);
	g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(&graphoptimizer,linearSolver);
	graphoptimizer.setSolver(solver_ptr);
	
	for(int i = 0; i < 10; i++){
		pthread_t mythread;
		pthread_create( &mythread, NULL, map_start_test_thread, NULL);
	}
	
	plane_segment_id_counter = 1;
	mergedPlanes.push_back(vector<pair<RGBDFrame * , Plane * > > ());
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
	transformations_mat->push_back(new vector<Transformation *>());
	Matrix4f pose = Matrix4f::Identity();

	printf("--------------------------------%i--------------------------------\n",frames.size());
	
	vector< Plane * > * planes_src = frame->planes;
	for(int j = 0; j < planes_src->size(); j++){
		Plane * src_p = planes_src->at(j);
		int segment_id 		= map_id_plane.find(src_p)->second;
		Plane * src_p_test 	= map_id_plane.find(src_p)->first;
		if(src_p_test != src_p || segment_id < 0){
			segment_id = plane_segment_id_counter;
			plane_segment_id_counter++;
			map_id_plane.insert( make_pair(src_p,segment_id));
			mergedPlanes.push_back(vector< pair<RGBDFrame * , Plane * > >());
			mergedPlanes.back().push_back(make_pair(frame,src_p));	
		}
		printf("added planes with id: %i\n",segment_id);
	}
	
	float best = -1;
	Transformation * best_t = 0;
	

	UpdateWeightFilter * update = new UpdateWeightFilter();
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
	
	
	for(int i = frames.size(); (i > 0) && ((frames.size() - i) < max_backing); i--){
		/*
		map_task * current_task = new map_task;
		current_task->src = frame;
		current_task->dst = frames.at(i-1);
		current_task->matcher = fmclose;
		current_task->m = this;
		current_task->weight_limit = w_limit_close;
		pthread_mutex_lock(&map_tasks_mutex);
		map_tasks.push_back(current_task);
		pthread_mutex_unlock(&map_tasks_mutex);
		*/
		
		Transformation * t = fmclose->getTransformation(frame, frames.at(i-1));
		if(t->weight > 0){
			pose = poses.at(i-1)*t->transformationMatrix;
			best = t->weight;
			best_t = t;
		}
		if(t->weight > w_limit_close){transformations.push_back(t);}
	}
	
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
			current_task->matcher = fmloop;
			current_task->m = this;
			current_task->weight_limit = w_limit_loop;
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

	
}

vector< GraphEdge > * Map3DPlanesGraphv4::match_planes(RGBDFrame * src, RGBDFrame * dst, Eigen::Matrix4f trans){
	
	Eigen::Matrix4f trans_inverse = trans.inverse();
	vector< GraphEdge > * output = new vector< GraphEdge >();
	
	vector< Plane * > * planes_src = src->planes;
	vector< Plane * > * planes_dst = dst->planes;
	
	//printf("matches planes for %i and %i...\n",src->id,dst->id);
	for(int j = 0; j < planes_src->size(); j++){
		Plane * src_p = planes_src->at(j);
		Plane * src_p_trans = new Plane();
		src_p_trans->normal_x = trans(0,0)*src_p->normal_x+trans(0,1)*src_p->normal_y+trans(0,2)*src_p->normal_z;
		src_p_trans->normal_y = trans(1,0)*src_p->normal_x+trans(1,1)*src_p->normal_y+trans(1,2)*src_p->normal_z;
		src_p_trans->normal_z = trans(2,0)*src_p->normal_x+trans(2,1)*src_p->normal_y+trans(2,2)*src_p->normal_z;
		
		src_p_trans->point_x  = trans(0,0)*src_p->point_x+trans(0,1)*src_p->point_y+trans(0,2)*src_p->point_z+trans(0,3);
		src_p_trans->point_y  = trans(1,0)*src_p->point_x+trans(1,1)*src_p->point_y+trans(1,2)*src_p->point_z+trans(1,3);
		src_p_trans->point_z  = trans(2,0)*src_p->point_x+trans(2,1)*src_p->point_y+trans(2,2)*src_p->point_z+trans(2,3);
		
		for(int k = 0; k < planes_dst->size(); k++){
			Plane * dst_p = planes_dst->at(k);
			Plane * dst_p_trans = new Plane();
			dst_p_trans->normal_x = trans_inverse(0,0)*dst_p->normal_x+trans_inverse(0,1)*dst_p->normal_y+trans_inverse(0,2)*dst_p->normal_z;
			dst_p_trans->normal_y = trans_inverse(1,0)*dst_p->normal_x+trans_inverse(1,1)*dst_p->normal_y+trans_inverse(1,2)*dst_p->normal_z;
			dst_p_trans->normal_z = trans_inverse(2,0)*dst_p->normal_x+trans_inverse(2,1)*dst_p->normal_y+trans_inverse(2,2)*dst_p->normal_z;
		
			dst_p_trans->point_x = trans_inverse(0,0)*dst_p->point_x+trans_inverse(0,1)*dst_p->point_y+trans_inverse(0,2)*dst_p->point_z+trans_inverse(0,3);
			dst_p_trans->point_y = trans_inverse(1,0)*dst_p->point_x+trans_inverse(1,1)*dst_p->point_y+trans_inverse(1,2)*dst_p->point_z+trans_inverse(1,3);
			dst_p_trans->point_z = trans_inverse(2,0)*dst_p->point_x+trans_inverse(2,1)*dst_p->point_y+trans_inverse(2,2)*dst_p->point_z+trans_inverse(2,3);
			int src_inliers = 0;
			int src_possible_inliers = 0;
			for(int i = 0; i < src->validation_points.size(); i++){
				float * vp = src->validation_points.at(i);
				if(fabs(src_p->distance(vp[0],vp[1],vp[2]))<0.02f){
					src_possible_inliers++;
					if(fabs(dst_p_trans->distance(vp[0],vp[1],vp[2]))<0.02f){src_inliers++;}
				}
			}
			
			int dst_inliers = 0;
			int dst_possible_inliers = 0;
			for(int i = 0; i < dst->validation_points.size(); i++){
				float * vp = dst->validation_points.at(i);
				if(fabs(dst_p->distance(vp[0],vp[1],vp[2]))<0.02f){
					dst_possible_inliers++;
					if(fabs(src_p_trans->distance(vp[0],vp[1],vp[2]))<0.02f){dst_inliers++;}
				}
			}
			float src_score = float(src_inliers)/float(src_possible_inliers);
			float dst_score = float(dst_inliers)/float(dst_possible_inliers);
			GraphEdge ge;
			ge.value = src_score*dst_score;
			ge.vertexes[0] = j;
			ge.vertexes[1] = k;
			ge.nr_vertexes = 2;
			output->push_back(ge);
			//if(src_score*dst_score > 0.95){output->push_back(make_pair(j,k));}
		}
	}
	
/*
	IplImage* src_rgb_img 	= cvLoadImage(src->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	IplImage* src_depth_img = cvLoadImage(src->input->depth_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	
	IplImage* dst_rgb_img 	= cvLoadImage(dst->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	IplImage* dst_depth_img = cvLoadImage(dst->input->depth_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);

	int width = 640;
	int height = 480;
	IplImage * img_combine = cvCreateImage(cvSize(2*width,height), IPL_DEPTH_8U, 3);
	char * data = (char *)img_combine->imageData;
	char * data_src = (char *)src_rgb_img->imageData;
	char * data_dst = (char *)dst_rgb_img->imageData;
	
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
	
	float d_scaleing	= src->input->calibration->ds/src->input->calibration->scale;
	float centerX		= src->input->calibration->cx;
	float centerY		= src->input->calibration->cy;
	float invFocalX		= 1.0f/src->input->calibration->fx;
    float invFocalY		= 1.0f/src->input->calibration->fy;
    
	unsigned short * depth_data	= (unsigned short *)src_depth_img->imageData;
	for(int w = 0; w < width; w++){
		for(int h = 0; h < height; h++){
			int ind = 640*h+w;
			float x = 0;
			float y = 0;
			float z = float(depth_data[ind]) * d_scaleing;
		   	int best_i;
			float best = 1000;
			if(z > 0){
				x = (w - centerX) * z * invFocalX;
		       	y = (h - centerY) * z * invFocalY;

				for(int i = 0; i < output->size(); i++){
					float d = fabs(src->planes->at(output->at(i).first)->distance(x,y,z));
					if(d < best){
						best = d;
						best_i = i;
					}
				}
				if(best < 0.01){
					ind = (h * (2*width) + (w));
					if(best_i == 0){
						data[3*ind+0] = 0;
						data[3*ind+1] = 0;
						data[3*ind+2] = 0;
					}else if(best_i == 1){
						data[3*ind+0] = 255;
						data[3*ind+1] = 0;
						data[3*ind+2] = 0;
					}else if(best_i == 2){
						data[3*ind+0] = 0;
						data[3*ind+1] = 0;
						data[3*ind+2] = 255;
					}else if(best_i == 3){
						data[3*ind+0] = 0;
						data[3*ind+1] = 255;
						data[3*ind+2] = 0;
					}else if(best_i == 4){
						data[3*ind+0] = 255;
						data[3*ind+1] = 0;
						data[3*ind+2] = 255;
					}else if(best_i == 5){
						data[3*ind+0] = 0;
						data[3*ind+1] = 255;
						data[3*ind+2] = 255;
					}else if(best_i == 6){
						data[3*ind+0] = 0;
						data[3*ind+1] = 255;
						data[3*ind+2] = 255;
					}else if(best_i == 7){
						data[3*ind+0] = 255;
						data[3*ind+1] = 255;
						data[3*ind+2] = 255;
					}
				}
			}
		}
	}
	
	d_scaleing	= dst->input->calibration->ds/dst->input->calibration->scale;
	centerX		= dst->input->calibration->cx;
	centerY		= dst->input->calibration->cy;
	invFocalX	= 1.0f/dst->input->calibration->fx;
    invFocalY	= 1.0f/dst->input->calibration->fy;
    
	depth_data	= (unsigned short *)dst_depth_img->imageData;
	for(int w = 0; w < width; w++){
		for(int h = 0; h < height; h++){
			int ind = 640*h+w;
			float x = 0;
			float y = 0;
			float z = float(depth_data[ind]) * d_scaleing;
		   	int best_i;
			float best = 1000;
			if(z > 0){
				x = (w - centerX) * z * invFocalX;
		       	y = (h - centerY) * z * invFocalY;

				for(int i = 0; i < output->size(); i++){
					float d = fabs(dst->planes->at(output->at(i).second)->distance(x,y,z));
					if(d < best){
						best = d;
						best_i = i;
					}
				}
				if(best < 0.01){
					ind = (h * (2*width) + (width+w));
					if(best_i == 0){
						data[3*ind+0] = 0;
						data[3*ind+1] = 0;
						data[3*ind+2] = 0;
					}else if(best_i == 1){
						data[3*ind+0] = 255;
						data[3*ind+1] = 0;
						data[3*ind+2] = 0;
					}else if(best_i == 2){
						data[3*ind+0] = 0;
						data[3*ind+1] = 0;
						data[3*ind+2] = 255;
					}else if(best_i == 3){
						data[3*ind+0] = 0;
						data[3*ind+1] = 255;
						data[3*ind+2] = 0;
					}else if(best_i == 4){
						data[3*ind+0] = 255;
						data[3*ind+1] = 0;
						data[3*ind+2] = 255;
					}else if(best_i == 5){
						data[3*ind+0] = 0;
						data[3*ind+1] = 255;
						data[3*ind+2] = 255;
					}else if(best_i == 6){
						data[3*ind+0] = 0;
						data[3*ind+1] = 255;
						data[3*ind+2] = 255;
					}else if(best_i == 7){
						data[3*ind+0] = 255;
						data[3*ind+1] = 255;
						data[3*ind+2] = 255;
					}
				}
			}
		}
	}

	cvNamedWindow("combined image", CV_WINDOW_AUTOSIZE );
	cvShowImage("combined image", img_combine);
		
	cvWaitKey(0);
	
	cvReleaseImage( &src_rgb_img );
	cvReleaseImage( &src_depth_img );
	
	cvReleaseImage( &dst_rgb_img );
	cvReleaseImage( &dst_depth_img );
*/	
	return output;
}

void Map3DPlanesGraphv4::addTransformation(Transformation * transformation){transformations.push_back(transformation);}
void Map3DPlanesGraphv4::estimate(){
	while(map_tasks_to_do() > 0){
		printf("tasks to do: %i\n",map_tasks_to_do());
		usleep(100000);
	}
	usleep(500000);
	/*
	for(int i = 0; i < frames.size(); i++){
		for(int k = i-1; k >= 0 && i-k > 5; k--){
			vector< pair < int , int > > * matches = match_planes(frames.at(i),frames.at(k),Matrix4f::Identity());
			for(int j = 0; j < matches->size(); j++){
				Plane * src_p = frames.at(i)->planes->at(matches->at(j).first);
				Plane * dst_p = frames.at(k)->planes->at(matches->at(j).second);
			
				int src_segment_id	= map_id_plane.find(src_p)->second;
				Plane * src_p_test	= map_id_plane.find(src_p)->first;
					
				int dst_segment_id	= map_id_plane.find(dst_p)->second;
				Plane * dst_p_test	= map_id_plane.find(dst_p)->first;
				if(src_segment_id != dst_segment_id){
					while(mergedPlanes.at(src_segment_id).size() != 0){
						mergedPlanes.at(dst_segment_id).push_back(mergedPlanes.at(src_segment_id).back());
						mergedPlanes.at(src_segment_id).pop_back();
					}
					for(int l = 0; l < mergedPlanes.at(dst_segment_id).size(); l++){
						pair<RGBDFrame * , Plane * > tmp_pair = mergedPlanes.at(dst_segment_id).at(l);
						map_id_plane.find(tmp_pair.second)->second = dst_segment_id;
					}
				}
			}
			delete matches;
		}
	}
	
	for(int i = 0; i < transformations.size(); i++){
		Transformation * t = transformations.at(i);
		printf("Transformation: %i <--> %i\n",t->src->id,t->dst->id);
		vector< pair < int , int > > * matches = match_planes(t->src,t->dst,t->transformationMatrix);
		for(int j = 0; j < matches->size(); j++){
			Plane * src_p = t->src->planes->at(matches->at(j).first);
			Plane * dst_p = t->dst->planes->at(matches->at(j).second);
			
			int src_segment_id	= map_id_plane.find(src_p)->second;
			Plane * src_p_test	= map_id_plane.find(src_p)->first;
					
			int dst_segment_id	= map_id_plane.find(dst_p)->second;
			Plane * dst_p_test	= map_id_plane.find(dst_p)->first;
			if(src_segment_id != dst_segment_id){
				while(mergedPlanes.at(src_segment_id).size() != 0){
					mergedPlanes.at(dst_segment_id).push_back(mergedPlanes.at(src_segment_id).back());
					mergedPlanes.at(src_segment_id).pop_back();
				}
				for(int l = 0; l < mergedPlanes.at(dst_segment_id).size(); l++){
					pair<RGBDFrame * , Plane * > tmp_pair = mergedPlanes.at(dst_segment_id).at(l);
					map_id_plane.find(tmp_pair.second)->second = dst_segment_id;
				}
			}
		}
		delete matches;
	}
	*/
	graphoptimizer.clear();
	graphoptimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
	graphoptimizer.setVerbose(false);
	g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(&graphoptimizer,linearSolver);
	graphoptimizer.setSolver(solver_ptr);
	
	printf("estimate\n");
	for(int i  = 0; i < frames.size(); i++){
		Eigen::Affine3f eigenTransform(poses.at(i));
		Eigen::Quaternionf eigenRotation(eigenTransform.rotation());
		g2o::SE3Quat poseSE3(Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
		g2o::VertexSE3 * vertexSE3 = new g2o::VertexSE3();
		vertexSE3->setId(frames.at(i)->id);
		vertexSE3->estimate() = poseSE3;
		graphoptimizer.addVertex(vertexSE3);
		frames.at(i)->g2oVertex = vertexSE3;
		if(i == 0){
			//vertexSE3->setFixed(true);
		}else{
			Eigen::Affine3f eigenTransform(Matrix4f::Identity());
			Eigen::Quaternionf eigenRotation(eigenTransform.rotation());
			g2o::SE3Quat transfoSE3(Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
			g2o::EdgeSE3* edgeSE3 = new g2o::EdgeSE3;
			edgeSE3->vertices()[0] = frames.at(i)->g2oVertex;
			edgeSE3->vertices()[1] = frames.at(i-1)->g2oVertex;
			edgeSE3->setMeasurement(transfoSE3.inverse());
			edgeSE3->setInverseMeasurement(transfoSE3);
			Eigen::Matrix<double, 6, 6, 0, 6, 6> mat;
			mat.setIdentity(6,6);
			edgeSE3->information() = mat*0.01f;
			graphoptimizer.addEdge(edgeSE3);
		}
	}
	for(int i  = 0; i < transformations.size(); i++){
		Transformation * transformation = transformations.at(i);

		if(transformation != 0){
			int matches_req = 0;
			float w_limit = 0;
			if(fabs(transformation->src->id - transformation->dst->id) < max_backing){
				matches_req = match_limit_close;
				w_limit = w_limit_close;
			}else{
				matches_req = match_limit_loop;
				w_limit = w_limit_loop;
			}
			if(fabs(transformation->src->id - transformation->dst->id) > 50){
				transformation->show();
				transformation->show(viewer);
			}
			if((transformation->weight >= w_limit) && (transformation->matches.size() >= matches_req))
			{
				printf("transformation: %i %i\n",transformation->src->id,transformation->dst->id);
				//transformation->show();
				Eigen::Affine3f eigenTransform(transformation->transformationMatrix);
				Eigen::Quaternionf eigenRotation(eigenTransform.rotation());
				g2o::SE3Quat transfoSE3(Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
				g2o::EdgeSE3* edgeSE3 = new g2o::EdgeSE3;
				edgeSE3->vertices()[0] = frames.at(transformation->src->id)->g2oVertex;
				edgeSE3->vertices()[1] = frames.at(transformation->dst->id)->g2oVertex;
				edgeSE3->setMeasurement(transfoSE3.inverse());
				edgeSE3->setInverseMeasurement(transfoSE3);
				Eigen::Matrix<double, 6, 6, 0, 6, 6> mat;
				mat.setIdentity(6,6);
				edgeSE3->information() = mat;
				graphoptimizer.addEdge(edgeSE3);
			}
		}
	}
	
	for(int i = 0; i < mergedPlanes.size(); i++){
		
		if(mergedPlanes.at(i).size() >= 50){
			printf("size:%i->%i\n",i,mergedPlanes.at(i).size());
			g2o::VertexPlane * vertexPlane2 = new g2o::VertexPlane();
			vertexPlane2->setId(200000+i);
			graphoptimizer.addVertex(vertexPlane2);
		
			for(int j = 0; j < mergedPlanes.at(i).size(); j++){
				pair<RGBDFrame * , Plane * > tmp_pair = mergedPlanes.at(i).at(j);
				int id = tmp_pair.first->id;
				printf("Frame id: %i\n",id);
				if(j == 0){
					Eigen::Matrix4f mat = poses.at(id);
					Plane * current_p = tmp_pair.second;
					vertexPlane2->rx = current_p->normal_x*mat(0,0) + current_p->normal_y*mat(0,1) + current_p->normal_z*mat(0,2);
					vertexPlane2->ry = current_p->normal_x*mat(1,0) + current_p->normal_y*mat(1,1) + current_p->normal_z*mat(1,2);
					vertexPlane2->rz = current_p->normal_x*mat(2,0) + current_p->normal_y*mat(2,1) + current_p->normal_z*mat(2,2);
					
					vertexPlane2->px = current_p->point_x*mat(0,0) + current_p->point_y*mat(0,1) + current_p->point_z*mat(0,2) + mat(0,3);
					vertexPlane2->py = current_p->point_x*mat(1,0) + current_p->point_y*mat(1,1) + current_p->point_z*mat(1,2) + mat(1,3);
					vertexPlane2->pz = current_p->point_x*mat(2,0) + current_p->point_y*mat(2,1) + current_p->point_z*mat(2,2) + mat(2,3);
				}
				g2o::EdgeSe3Plane2 * edgeSe3Plane2 = new g2o::EdgeSe3Plane2();

				edgeSe3Plane2->vertices()[0] = tmp_pair.first->g2oVertex;
				edgeSe3Plane2->vertices()[1] = vertexPlane2;
				Matrix4d informationMat = Matrix4d::Identity();
				//informationMat(3,3) = 0.0;
				//informationMat*=0.0001f;
				edgeSe3Plane2->information() = informationMat;
				edgeSe3Plane2->setMeasurement(tmp_pair.second);
				graphoptimizer.addEdge(edgeSe3Plane2);
				//printf("added edge\n");
			}
		}
	}
	
	graphoptimizer.initializeOptimization();
	graphoptimizer.setVerbose(true);
	graphoptimizer.optimize(estimateIter);
	for(int i  = 0; i < frames.size(); i++){
		g2o::VertexSE3 * vertexSE3_src = (g2o::VertexSE3*)(graphoptimizer.vertex(frames.at(i)->id));
		poses.at(i) = (vertexSE3_src->estimate().to_homogenious_matrix()).cast<float>();
		cout<<i<<endl<<poses.at(i)<<endl;
	}
/*	
	for(int i  = 0; i < frames.size(); i++){
		Matrix4f pose_i = poses.at(i);
		Matrix4f pose_i_inv = pose_i.inverse();
		for(int j  = i+1; j < frames.size(); j++){
			Matrix4f pose_j = poses.at(j);
			Matrix4f pose_j_inv = pose_j.inverse();
			Matrix4f transform = pose_i_inv*pose_j;
			Matrix4f transform_inv = transform.inverse();
			
			RGBDFrame * dst = frames.at(i);
			RGBDFrame * src = frames.at(j);
			
			float mat00 = transform(0,0);
			float mat01 = transform(0,1);
			float mat02 = transform(0,2);
			float mat03 = transform(0,3);
			float mat10 = transform(1,0);
			float mat11 = transform(1,1);
			float mat12 = transform(1,2);
			float mat13 = transform(1,3);
			float mat20 = transform(2,0);
			float mat21 = transform(2,1);
			float mat22 = transform(2,2);
			float mat23 = transform(2,3);
			
			float counter_good = 0;
			float counter_bad = 0;
			float counter_total = 0;
			
			IplImage* depth_img = cvLoadImage(src->input->depth_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
			unsigned short * depth_data	= (unsigned short *)depth_img->imageData;
			
			float limit = 0.01;
			
			
			float d_scaleing	= dst->input->calibration->ds/dst->input->calibration->scale;
			float centerX		= dst->input->calibration->cx;
			float centerY		= dst->input->calibration->cy;
			float invFocalX		= 1.0f/dst->input->calibration->fx;
			float invFocalY		= 1.0f/dst->input->calibration->fy;
			
			for(int k = 0; k < src->validation_points.size(); k++){
				float * vp = src->validation_points.at(k);
	
				float x_tmp = vp[0];
				float y_tmp = vp[1];
				float z_tmp = vp[2];
				
				float x = x_tmp*mat00+y_tmp*mat01+z_tmp*mat02+mat03;
				float y = x_tmp*mat10+y_tmp*mat11+z_tmp*mat12+mat13;
				float z = x_tmp*mat20+y_tmp*mat21+z_tmp*mat22+mat23;
		
				//int d_data 	= int(0.5f+z/d_scaleing);
				int w 		= int(0.5f+x/(z * invFocalX) + centerX);
				int h 		= int(0.5f+y/(z * invFocalY) + centerY);
				if(w>=0 && w < 640 && h >= 0 && h < 480){
					float z_img = int(depth_data[640*h+w])*d_scaleing;
					if(z_img > 0.01){
						float diff = z-z_img;
						if(fabs(diff) < limit)		{counter_good++;}
						else if(z+0.075f < z_img)	{counter_bad++;}
						counter_total++;
					}
				}
			}
			cvReleaseImage( &depth_img );
			
			if(counter_total < 5000){counter_total = 5000;}
			if(counter_bad > counter_total*0.5){printf("%i %i src -> good: %f bad: %f total%f\n",i,j,counter_good,counter_bad,counter_total);}
			
			mat00 = transform_inv(0,0);
			mat01 = transform_inv(0,1);
			mat02 = transform_inv(0,2);
			mat03 = transform_inv(0,3);
			mat10 = transform_inv(1,0);
			mat11 = transform_inv(1,1);
			mat12 = transform_inv(1,2);
			mat13 = transform_inv(1,3);
			mat20 = transform_inv(2,0);
			mat21 = transform_inv(2,1);
			mat22 = transform_inv(2,2);
			mat23 = transform_inv(2,3);
			
			counter_good = 0;
			counter_bad = 0;
			counter_total = 0;
			
			depth_img = cvLoadImage(dst->input->depth_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
			depth_data	= (unsigned short *)depth_img->imageData;
						
			for(int k = 0; k < dst->validation_points.size(); k++){
				float * vp = dst->validation_points.at(k);
	
				float x_tmp = vp[0];
				float y_tmp = vp[1];
				float z_tmp = vp[2];
				
				float x = x_tmp*mat00+y_tmp*mat01+z_tmp*mat02+mat03;
				float y = x_tmp*mat10+y_tmp*mat11+z_tmp*mat12+mat13;
				float z = x_tmp*mat20+y_tmp*mat21+z_tmp*mat22+mat23;
		
				//int d_data 	= int(0.5f+z/d_scaleing);
				int w 		= int(0.5f+x/(z * invFocalX) + centerX);
				int h 		= int(0.5f+y/(z * invFocalY) + centerY);
				if(w>=0 && w < 640 && h >= 0 && h < 480){
					float z_img = int(depth_data[640*h+w])*d_scaleing;
					if(z_img > 0.01){
						float diff = z-z_img;
						if(fabs(diff) < limit)		{counter_good++;}
						else if(z+0.075f < z_img)	{counter_bad++;}
						counter_total++;
					}
				}
			}
			cvReleaseImage( &depth_img );
			
			if(counter_total < 5000){counter_total = 5000;}
			if(counter_bad > counter_total*0.5){printf("%i %i dst -> good: %f bad: %f total%f\n",i,j,counter_good,counter_bad,counter_total);}
			
		}
	}
*/
	printf("estimate done\n");
}
void Map3DPlanesGraphv4::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}

void Map3DPlanesGraphv4::visualize(){
	bool render_full = true;
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
		int step = poses.size()/100;
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
		
			int pixelstep = 2;
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

					if(z > 0 && z < 3.0f){
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
	for(int i = 0; i < 50; i++)///while (true)
	{
		printf("i:%i\n",i);
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	
}

