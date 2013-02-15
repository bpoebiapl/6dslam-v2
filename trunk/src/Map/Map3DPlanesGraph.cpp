#include "Map3DPlanesGraph.h"
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

bool comparison_Map3DPlanesGraph (Transformation * i,Transformation * j) {
	if(i->src == j->src){return (i->weight<j->weight);}
	else{return (i->src->id<j->src->id);}
}

using namespace std;

Map3DPlanesGraph::Map3DPlanesGraph(){
	graphoptimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
	graphoptimizer.setVerbose(false);
	g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(&graphoptimizer,linearSolver);
	graphoptimizer.setSolver(solver_ptr);
}

Map3DPlanesGraph::~Map3DPlanesGraph(){}

g2o::SparseOptimizer * Map3DPlanesGraph::startNewGraph()
{
	g2o::SparseOptimizer * graph = new g2o::SparseOptimizer;
	graph->setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
	graph->setVerbose(false);
	g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(&graphoptimizer,linearSolver);
	graph->setSolver(solver_ptr);
	return graph;
}

g2o::VertexSE3 * Map3DPlanesGraph::getG2OVertex(RGBDFrame * frame)
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

g2o::EdgeSE3 * Map3DPlanesGraph::getG2OEdge(Transformation * transformation)
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

void Map3DPlanesGraph::addFrame(Frame_input * fi){addFrame(new RGBDFrame(fi,extractor,segmentation));}
void Map3DPlanesGraph::addFrame(RGBDFrame * frame){
	printf("Map3DPlanesGraph::addFrame(RGBDFrame * frame)\n");
	Matrix4f pose;
	Transformation * t = 0;
	if(frames.size() > 0){
		t = matcher->getTransformation(frame, frames.back());
		pose = poses.back()*t->transformationMatrix;
	}else{pose = Matrix4f::Identity();}
	poses.push_back(pose);
	frames.push_back(frame);
	
	if(t != 0){transformations.push_back(t);}
	
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
}
void Map3DPlanesGraph::addTransformation(Transformation * transformation){
	transformations.push_back(transformation);
	/*
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
	*/
}
void Map3DPlanesGraph::estimate(){
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
	graphoptimizer.initializeOptimization();
	graphoptimizer.setVerbose(true);
	graphoptimizer.optimize(50);
	for(unsigned int i  = 0; i < frames.size(); i++){
		g2o::VertexSE3 * vertexSE3_src = (g2o::VertexSE3*)(graphoptimizer.vertex(frames.at(i)->id));
		poses.at(i) = (vertexSE3_src->estimate().to_homogenious_matrix()).cast<float>();
		//cout<<i<<endl<<poses.at(i)<<endl;
	}
	
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
}
void Map3DPlanesGraph::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}

//bool goToNext = false;
//void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,void* viewer_void){goToNext = true;}

void Map3DPlanesGraph::visualize(){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->width    = 0;
	cloud->height   = 1;
	for(unsigned int i = 0; i < poses.size(); i+=10){cloud->width+=frames.at(i)->xyz_->width;}
	cloud->points.resize (cloud->width);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width    = 0;
	for(unsigned int i = 0; i < poses.size(); i+=10){
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
	g2o::VertexPlane * vertex = (g2o::VertexPlane*)(graphoptimizer.vertex(200000+0));
	pcl::ModelCoefficients coeffs;
	coeffs.values.push_back (vertex->rx);
	coeffs.values.push_back (vertex->ry);
	coeffs.values.push_back (vertex->rz);
	coeffs.values.push_back (vertex->d);
	viewer->addPlane (coeffs, "plane");
	
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

