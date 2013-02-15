#include "Map3DPlanesGraphv2.h"
#include "Frame_input.h"
#include <utility>
#include <vector>
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

#include "graph/vertex_point_xy.cpp"
#include "graph/vertex_se2.cpp"
#include "graph/edge_se2_pointxy.cpp"
#include "graph/se2.h"

#include "VertexPlane.cpp"
#include "EdgeSe3Plane.cpp"
#include "EdgeSe3Plane2.cpp"
#include "EdgePlane.cpp"

bool comparison_Map3DPlanesGraphv2 (Transformation * i,Transformation * j) {
	if(i->src == j->src){return (i->weight<j->weight);}
	else{return (i->src->id<j->src->id);}
}

using namespace std;

Map3DPlanesGraphv2::Map3DPlanesGraphv2(){
	graphoptimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
	graphoptimizer.setVerbose(false);
	g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(&graphoptimizer,linearSolver);
	graphoptimizer.setSolver(solver_ptr);
	plane_segment_id_counter = 1;
	mergedPlanes.push_back(vector<pair<RGBDFrame * , Plane * > > ());
}

Map3DPlanesGraphv2::~Map3DPlanesGraphv2(){}

g2o::SparseOptimizer * Map3DPlanesGraphv2::startNewGraph()
{
	g2o::SparseOptimizer * graph = new g2o::SparseOptimizer;
	graph->setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
	graph->setVerbose(false);
	g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(&graphoptimizer,linearSolver);
	graph->setSolver(solver_ptr);
	return graph;
}

g2o::VertexSE3 * Map3DPlanesGraphv2::getG2OVertex(RGBDFrame * frame)
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

g2o::EdgeSE3 * Map3DPlanesGraphv2::getG2OEdge(Transformation * transformation)
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

void Map3DPlanesGraphv2::addFrame(Frame_input * fi){
	//new RGBDFrame(fi,extractor,segmentation);
	addFrame(new RGBDFrame(fi,extractor,segmentation));
}
void Map3DPlanesGraphv2::addFrame(RGBDFrame * frame){
	printf("Map3DPlanesGraphv2::addFrame(RGBDFrame * frame)\n");
	frame->id = frames.size();
	Matrix4f pose;
	Transformation * t = 0;
	if(frames.size() > 0){
		t = matcher->getTransformation(frame, frames.back());
		pose = poses.back()*t->transformationMatrix;
	}else{pose = Matrix4f::Identity();}
	poses.push_back(pose);
	frames.push_back(frame);
	
	if(t != 0){transformations.push_back(t);}
	if(frames.size()>1){
		RGBDFrame * src = frames.at(frames.size()-2);
		RGBDFrame * dst = frames.at(frames.size()-1);
		
		IplImage* img_combine;
		int width;
		int height;
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
		
		//cvNamedWindow("src image", CV_WINDOW_AUTOSIZE );
		//cvShowImage("src image", rgb_img_src);
		
		//cvNamedWindow("dst image", CV_WINDOW_AUTOSIZE );
		//cvShowImage("dst image", rgb_img_dst);
		

		int st4 = 1;

		int ** src_plane_mat = new int*[width];
		int ** dst_plane_mat = new int*[width];
		for (int i = 0; i < width; i++){
			src_plane_mat[i] = new int[height];
			dst_plane_mat[i] = new int[height];
			for (int j = 0; j < height; j++){
				src_plane_mat[i][j] = -1;
				dst_plane_mat[i][j] = -1;
			}
		}
		
		for(int i = 0; i < src->planes->size();i++){
			Plane * src_p = src->planes->at(i);
			for(int j = 0; j < src_p->w_vec->size();j++){
				src_plane_mat[src_p->w_vec->at(j)][src_p->h_vec->at(j)] = i;
			}
		}
		
		for(int i = 0; i < dst->planes->size();i++){
			Plane * dst_p = dst->planes->at(i);
			for(int j = 0; j < dst_p->w_vec->size();j++){
				dst_plane_mat[dst_p->w_vec->at(j)][dst_p->h_vec->at(j)] = i;
			}
		}
		
		float ** counter = new float*[src->planes->size()+1];
		for(int i = 0; i < src->planes->size()+1;i++){
			counter[i] = new float[dst->planes->size()+1];
			for(int j = 0; j < dst->planes->size()+1;j++){
				counter[i][j] = 0;
			}
		}
		for (int i = 0; i < width; i++){
			for (int j = 0; j < height; j++){
				int src_p_id = src_plane_mat[i][j]+1;
				int dst_p_id = dst_plane_mat[i][j]+1;
				counter[src_p_id][dst_p_id]++;
			}
		}
		
		for (int i = 0; i < width; i++){
			delete[] src_plane_mat[i];
			delete[] dst_plane_mat[i];
		}
		delete[] src_plane_mat;
		delete[] dst_plane_mat;
		/*
		for(int i = 0; i < src->planes->size()+1;i++){
			for(int j = 0; j < dst->planes->size()+1;j++){
				printf("%f ",counter[i][j]);
			}
			printf("\n");
		}
		*/
		//plane_segment_id_counter++;
		//map< Plane *, int > map_id_plane;
		for(int i = 0; i < src->planes->size();i++){
			Plane * src_p = src->planes->at(i);
			int r,g,b;
			if(i == 0)		{r=0;g=0;b=255;}
			else if(i == 1)	{r=0;g=255;b=0;}
			else if(i == 2)	{r=255;g=0;b=0;}
			else if(i == 3)	{r=0;g=255;b=255;}
			else if(i == 4)	{r=255;g=0;b=255;}
			else if(i == 5)	{r=255;g=255;b=0;}
			else 			{r=rand()%256;g=rand()%256;b=rand()%256;}
			
			for(int j = 0; j < src_p->w_vec->size();j++){
				int w = src_p->w_vec->at(j);
				int h = src_p->h_vec->at(j);
				cvRectangle(img_combine,cvPoint(w-st4, h-st4),cvPoint(w+st4, h+st4),cvScalar(r, g, b, 0),-1, 8, 0);
				int ind = 3 * (h * (2*width) + (w));
				//data[ind+0] = b;
				//data[ind+1] = g;
				//data[ind+2] = r;
			}
			
			cvNamedWindow("combined image", CV_WINDOW_AUTOSIZE );
			cvShowImage("combined image", img_combine);
			//cvWaitKey(0);
			
			int segment_id = map_id_plane.find(src_p)->second;
			Plane * src_p_test = map_id_plane.find(src_p)->first;
			if(src_p_test != src_p || segment_id < 0){
				segment_id = plane_segment_id_counter;
				plane_segment_id_counter++;
				mergedPlanes.push_back(vector< pair<RGBDFrame * , Plane * > >());
			}
			//printf("segment_id:%i\n",segment_id);	
			for(int j = 0; j < dst->planes->size();j++){
				Plane * dst_p = dst->planes->at(j);
				float ffangle = src_p->angle(dst_p);
				float ddiff = fabs(fabs(src_p->distance(0,0,0))-fabs(dst_p->distance(0,0,0)));

				float pij = counter[i+1][j+1]/float(dst_p->w_vec->size());
				float pji = counter[i+1][j+1]/float(src_p->w_vec->size());
				//printf("frameframe angle: %f ",src_p->angle(dst_p));printf("ddiff: %f ",ddiff);printf("pij: %f pji: %f\n",pij,pji);
				if(ffangle > 0.95 && ddiff < 0.05 && (pij > 0.5 || pji > 0.5)){
					map_id_plane.insert( make_pair(dst_p,segment_id));
					mergedPlanes.at(segment_id).push_back(make_pair(frame,src_p));
					
					for(int k = 0; k < dst_p->w_vec->size();k++){
						
						int w = dst_p->w_vec->at(k);
						int h = dst_p->h_vec->at(k);
						
						cvRectangle(img_combine,cvPoint(w+width-st4, h-st4),cvPoint(w+width+st4, h+st4),cvScalar(r, g, b, 0),-1, 8, 0);
						//int ind = 3 * (h * (2*width) + (width+w));
						//data[ind+0] = b;
						//data[ind+1] = g;
						//data[ind+2] = r;

						//cvRectangle(img_combine,cvPoint(w+width-st4, h-st4),cvPoint(w+width+st4, h+st4),cvScalar(r, g , b, 0),-1, 8, 0);
					}
					//printf("good\n");
					cvNamedWindow("combined image", CV_WINDOW_AUTOSIZE );
					cvShowImage("combined image", img_combine);
					//cvWaitKey(0);
				}
			}
		}
		
		cvNamedWindow("combined image", CV_WINDOW_AUTOSIZE );
		cvShowImage("combined image", img_combine);
		//cvWaitKey(0);
		cvReleaseImage( &rgb_img_src );
		cvReleaseImage( &rgb_img_dst );
	}
/*	
	int stop = frames.size()-2;
	FeatureDescriptor * frame_desc = frame->image_descriptor;
	for(int i = 0; i < stop; i++){
		RGBDFrame * test = frames.at(i);
		//printf("look:%i,%i,%f\n",frame->id,test->id,test->image_descriptor->distance(frame_desc));
		if(test->image_descriptor->distance(frame_desc) < 0.0025){
			printf("%i,%i\n",frame->id,test->id);
			
			t = matcher->getTransformation(frame, test);
			//printf("add:%i,%i->%f\n",frame->id,test->id,t->weight);
			if(t->weight > 65){
				printf("add:%i,%i->%f\n",frame->id,test->id,t->weight);
				transformations.push_back(t);
			}
			
		}
	}
*/
}
void Map3DPlanesGraphv2::addTransformation(Transformation * transformation){transformations.push_back(transformation);}
void Map3DPlanesGraphv2::estimate(){
	printf("estimate\n");
	printf("nr frames: %i nr trans: %i\n",frames.size(),transformations.size());
	for(unsigned int i  = 0; i < frames.size(); i++){
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
	
	for(unsigned int i  = 0; i < transformations.size(); i++){
		Transformation * transformation = transformations.at(i);
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
	
	for(unsigned int i  = 0; i < mergedPlanes.size(); i++){
		if(mergedPlanes.at(i).size() > 10){
			printf("merged segment: %i\n",mergedPlanes.at(i).size());
		

			g2o::VertexPlane * vertexPlane2 = new g2o::VertexPlane();
			vertexPlane2->setId(200000+i);
		
			graphoptimizer.addVertex(vertexPlane2);
		
			int min = 1000000000;
			for(unsigned int j  = 0; j < mergedPlanes.at(i).size(); j++){
				pair < RGBDFrame *, Plane * > p = mergedPlanes.at(i).at(j);
				RGBDFrame * current_frame = p.first;
				Plane * current_p = p.second;
			
				if(min > current_frame->id){
					min = current_frame->id;
					Matrix4f mat = poses.at(min);

					vertexPlane2->rx = current_p->normal_x*mat(0,0) + current_p->normal_y*mat(0,1) + current_p->normal_z*mat(0,2);
					vertexPlane2->ry = current_p->normal_x*mat(1,0) + current_p->normal_y*mat(1,1) + current_p->normal_z*mat(1,2);
					vertexPlane2->rz = current_p->normal_x*mat(2,0) + current_p->normal_y*mat(2,1) + current_p->normal_z*mat(2,2);
					//vertexPlane2->d	 = current_p->d;//*mat(2,0) + current_p->normal_y*mat(2,1) + current_p->normal_z*mat(2,2);
					//vertexPlane2->d	 = current_p->d;//*mat(2,0) + current_p->normal_y*mat(2,1) + current_p->normal_z*mat(2,2);
				}
			
			
				g2o::EdgeSe3Plane2 * edgeSe3Plane2 = new g2o::EdgeSe3Plane2();

				edgeSe3Plane2->vertices()[0] = graphoptimizer.vertex(current_frame->id);
				edgeSe3Plane2->vertices()[1] = vertexPlane2;
				edgeSe3Plane2->information() = Matrix4d::Identity();
				edgeSe3Plane2->setMeasurement(current_p);
				graphoptimizer.addEdge(edgeSe3Plane2);
			}
		}
	}
	/*
	for(unsigned int i  = 0; i < mergedPlanes.size(); i++){
			for(unsigned int ii  = i+1; ii < mergedPlanes.size(); ii++){

				for(unsigned int jj  = 0; jj < mergedPlanes.at(ii)->data.size(); jj++){
					if(mergedPlanes.at(i)->data.at(j)->frame == mergedPlanes.at(ii)->data.at(jj)->frame){
						Plane * pi = mergedPlanes.at(i)->data.at(j)->frame->planes->at(mergedPlanes.at(i)->data.at(j)->plane_index);
						Plane * pii = mergedPlanes.at(ii)->data.at(jj)->frame->planes->at(mergedPlanes.at(ii)->data.at(jj)->plane_index);
						//printf("angle: %f\n",pi->angle(pii));

						g2o::EdgePlane * edgePlane = new g2o::EdgePlane();	
						
						edgePlane->vertices()[0] = graphoptimizer.vertex(200000+i);
						edgePlane->vertices()[1] = graphoptimizer.vertex(200000+j);
						
						Eigen::Matrix<double, 1, 1, 0, 1, 1> mat;

						mat.setIdentity(1,1);
						edgePlane->information() = mat;
						
						edgePlane->setMeasurement(pi->angle(pii));
						

						//graphoptimizer.addEdge(edgePlane);
						
					}
				}
			}
		}

	}
	*/
	graphoptimizer.initializeOptimization();
	graphoptimizer.setVerbose(true);
	graphoptimizer.optimize(150);
	for(unsigned int i  = 0; i < frames.size(); i++){
		g2o::VertexSE3 * vertexSE3_src = (g2o::VertexSE3*)(graphoptimizer.vertex(frames.at(i)->id));
		poses.at(i) = (vertexSE3_src->estimate().to_homogenious_matrix()).cast<float>();
		//cout<<i<<endl<<poses.at(i)<<endl;
	}
	/*
	//vector< vector < pair < RGBDFrame * , int > > * > * merged_planes = new vector< vector < pair < RGBDFrame * , int > > * >();
	vector< PlaneStruct * > planeStructs	= vector< PlaneStruct * >();
	vector< MergedPlanes * > mergedPlanes	= vector< MergedPlanes * >();
	
	int c = 0;
	for(unsigned int i  = 0; i < frames.size(); i++){
		vector< Plane * > * planes = frames.at(i)->planes;
		for(unsigned int j = 0; j < planes->size(); j++){
			planes->at(j)->id = c;
			PlaneStruct * p = new PlaneStruct();
			p->frame = frames.at(i);
			p->plane_index = j;
			
			MergedPlanes * mp = new MergedPlanes();
			mp->data.push_back(p);
			mp->index = c;
			p->mergedPlanes = mp;
			
			
			planeStructs.push_back(p);
			mergedPlanes.push_back(mp);
			c++;
		}
	}
	printf("total planes: %i\n",c);
	//exit(0);
	for(unsigned int ii  = 0; ii < frames.size(); ii++){
		for(unsigned int i  = 0; i < ii; i++){
			RGBDFrame * src = frames.at(i);//transformations.at(i)->src;
			RGBDFrame * dst = frames.at(ii);//transformations.at(i)->dst;
			Matrix4f transformationmat = poses.at(src->id).inverse()*poses.at(dst->id);
			vector< Plane * > * planes_src = frames.at(src->id)->planes;
			vector< Plane * > * planes_dst = frames.at(dst->id)->planes;
			for(unsigned int j = 0; j < planes_dst->size(); j++){
				Plane * dst_p = planes_dst->at(j);
				float xn = transformationmat(0,0)*dst_p->normal_x+transformationmat(0,1)*dst_p->normal_y+transformationmat(0,2)*dst_p->normal_z;
				float yn = transformationmat(1,0)*dst_p->normal_x+transformationmat(1,1)*dst_p->normal_y+transformationmat(1,2)*dst_p->normal_z;
				float zn = transformationmat(2,0)*dst_p->normal_x+transformationmat(2,1)*dst_p->normal_y+transformationmat(2,2)*dst_p->normal_z;
			
				float xp = transformationmat(0,0)*dst_p->point_x+transformationmat(0,1)*dst_p->point_y+transformationmat(0,2)*dst_p->point_z+transformationmat(0,3);
				float yp = transformationmat(1,0)*dst_p->point_x+transformationmat(1,1)*dst_p->point_y+transformationmat(1,2)*dst_p->point_z+transformationmat(1,3);
				float zp = transformationmat(2,0)*dst_p->point_x+transformationmat(2,1)*dst_p->point_y+transformationmat(2,2)*dst_p->point_z+transformationmat(2,3);
				for(unsigned int k = 0; k < planes_src->size(); k++){
					Plane * src_p = planes_src->at(k);
					float normal_diff = 1-(src_p->normal_x*xn + src_p->normal_y*yn + src_p->normal_z*zn);
					float mid_diff = src_p->distance(xp,yp,zp);
					if(normal_diff < 0.025 && fabs(mid_diff) < 0.04){
						MergedPlanes * src_mp = planeStructs.at(src_p->id)->mergedPlanes;
						MergedPlanes * dst_mp = planeStructs.at(dst_p->id)->mergedPlanes;
						if(src_mp != dst_mp){
							for(unsigned int m = 0; m < dst_mp->data.size(); m++){
								PlaneStruct * current_planeStruct = dst_mp->data.at(m);
								current_planeStruct->mergedPlanes = src_mp;
								src_mp->data.push_back(current_planeStruct);
							}
							mergedPlanes.at(dst_mp->index) = mergedPlanes.back();
							mergedPlanes.at(dst_mp->index)->index = dst_mp->index;
							mergedPlanes.pop_back();
						}
					
					}
				}
			}
		}
	}

	for(unsigned int i  = 0; i < mergedPlanes.size(); i++){
		printf("merged segment: %i\n",mergedPlanes.at(i)->data.size());
		
		g2o::VertexPlane * vertexPlane2 = new g2o::VertexPlane();
		vertexPlane2->setId(200000+i);
		
		graphoptimizer.addVertex(vertexPlane2);
		
		int min = 1000000000;
		for(unsigned int j  = 0; j < mergedPlanes.at(i)->data.size(); j++){
			Plane * p = mergedPlanes.at(i)->data.at(j)->frame->planes->at(mergedPlanes.at(i)->data.at(j)->plane_index);
			if(min > mergedPlanes.at(i)->data.at(j)->frame->id){
				min = mergedPlanes.at(i)->data.at(j)->frame->id;
				Matrix4f mat = poses.at(min);
				vertexPlane2->rx = p->normal_x*mat(0,0) + p->normal_y*mat(0,1) + p->normal_z*mat(0,2);
				vertexPlane2->ry = p->normal_x*mat(1,0) + p->normal_y*mat(1,1) + p->normal_z*mat(1,2);
				vertexPlane2->rz = p->normal_x*mat(2,0) + p->normal_y*mat(2,1) + p->normal_z*mat(2,2);
			}
			g2o::EdgeSe3Plane2 * edgeSe3Plane2 = new g2o::EdgeSe3Plane2();	
			edgeSe3Plane2->vertices()[0] = graphoptimizer.vertex(mergedPlanes.at(i)->data.at(j)->frame->id);
			edgeSe3Plane2->vertices()[1] = vertexPlane2;
			edgeSe3Plane2->information() = Matrix4d::Identity();
			edgeSe3Plane2->setMeasurement(p);
			graphoptimizer.addEdge(edgeSe3Plane2);
		}
	}
	
	for(unsigned int i  = 0; i < mergedPlanes.size(); i++){
		for(unsigned int j  = 0; j < mergedPlanes.at(i)->data.size(); j++){
			for(unsigned int ii  = i+1; ii < mergedPlanes.size(); ii++){
				for(unsigned int jj  = 0; jj < mergedPlanes.at(ii)->data.size(); jj++){
					if(mergedPlanes.at(i)->data.at(j)->frame == mergedPlanes.at(ii)->data.at(jj)->frame){
						Plane * pi = mergedPlanes.at(i)->data.at(j)->frame->planes->at(mergedPlanes.at(i)->data.at(j)->plane_index);
						Plane * pii = mergedPlanes.at(ii)->data.at(jj)->frame->planes->at(mergedPlanes.at(ii)->data.at(jj)->plane_index);
						//printf("angle: %f\n",pi->angle(pii));
						g2o::EdgePlane * edgePlane = new g2o::EdgePlane();	
						
						edgePlane->vertices()[0] = graphoptimizer.vertex(200000+i);
						edgePlane->vertices()[1] = graphoptimizer.vertex(200000+j);
						
						Eigen::Matrix<double, 1, 1, 0, 1, 1> mat;
						mat.setIdentity(1,1);
						edgePlane->information() = mat;
						
						edgePlane->setMeasurement(pi->angle(pii));
						
						//graphoptimizer.addEdge(edgePlane);
						
					}
				}
			}
		}
	}
	
	graphoptimizer.initializeOptimization();
	graphoptimizer.setVerbose(true);
	graphoptimizer.optimize(50);
	
	printf("estimate done\n");
	for(unsigned int i  = 0; i < frames.size(); i++){
		g2o::VertexSE3 * vertexSE3_src = (g2o::VertexSE3*)(graphoptimizer.vertex(frames.at(i)->id));
		poses.at(i) = (vertexSE3_src->estimate().to_homogenious_matrix()).cast<float>();
		//cout<<i<<endl<<poses.at(i)<<endl;
	}
	*/
}
void Map3DPlanesGraphv2::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}

//bool goToNext = false;
//void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,void* viewer_void){goToNext = true;}

void Map3DPlanesGraphv2::visualize(){
	int step = poses.size()/150;
	if(step < 1){step = 1;}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->width    = 0;
	cloud->height   = 1;
	for(unsigned int i = 0; i < poses.size(); i+=step){cloud->width+=frames.at(i)->xyz_->width;}
	cloud->points.resize (cloud->width);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width    = 0;
	for(unsigned int i = 0; i < poses.size(); i+=step){
		//*(frames.at(i)->xyz_);
		//*tmp_cloud;
		//poses.at(i);
		pcl::transformPointCloud (*(frames.at(i)->xyz_), *tmp_cloud, poses.at(i));
		int randr = 0;rand()%256;
		int randg = 255;rand()%256;
		int randb = 0;rand()%256;
		for(int j = 0; j < tmp_cloud->width*tmp_cloud->height; j++)
		{
			cloud->points[cloud->width].x = tmp_cloud->points[j].x;
			cloud->points[cloud->width].y = tmp_cloud->points[j].y;
			cloud->points[cloud->width].z = tmp_cloud->points[j].z;
			cloud->points[cloud->width].r = randr;
			cloud->points[cloud->width].g = randg;
			cloud->points[cloud->width].b = randb;
			cloud->width++;
		}
	}
	
	//viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
	//for(int i = 0;; i++);{
	
	for(unsigned int i  = 0; i < mergedPlanes.size(); i++){
		if(mergedPlanes.at(i).size() > 10){
			g2o::VertexPlane * vertex = (g2o::VertexPlane*)(graphoptimizer.vertex(200000+i));
			pcl::ModelCoefficients coeffs;
			coeffs.values.push_back (vertex->rx);
			coeffs.values.push_back (vertex->ry);
			coeffs.values.push_back (vertex->rz);
			coeffs.values.push_back (0);//vertex->d);
			char buf[100];
			sprintf(buf,"plane%i",i);
			printf("buf:%s\n",buf);
			//viewer->addPlane (coeffs, buf);
		}
	}
	
	while(true){
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		viewer->removePointCloud("sample cloud");
		viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
		
		usleep(100);
		for(int i = 0; i < 100; i++){
			viewer->spinOnce (100);
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
	}
	
}

