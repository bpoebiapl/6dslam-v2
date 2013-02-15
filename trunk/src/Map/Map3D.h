#ifndef Map3D_H_
#define Map3D_H_

//other
#include <stdio.h>
#include <string.h>
#include <string>
#include <iostream>

#include "RGBDFrame.h"
#include "Transformation.h" 

#include "FrameMatcher.h"
#include "FeatureExtractor.h"
#include "RGBDSegmentation.h"

using namespace std;
using namespace Eigen;

class Map3D
{
	public:
	FrameMatcher * matcher;
	FrameMatcher * loopclosure_matcher;
	FeatureExtractor * extractor;
	RGBDSegmentation * segmentation;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	
	vector<RGBDFrame *> frames;
	vector<Transformation *> transformations;
	vector<Matrix4f> poses;
	bool show;
	
	Map3D();
	virtual ~Map3D(); 
	
	virtual void addFrame(Frame_input * fi);
	virtual void addFrame(RGBDFrame * frame);
	virtual void addTransformation(Transformation * transformation);
	
	virtual void estimate();
	virtual void setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view);
	virtual void visualize();
	virtual void showTuning();
};

#include "Map3Dbase.h"
#include "Map3Dbow.h"
#include "Map3DbaseGraph.h"
#include "Map3DPlanesGraph.h"
#include "Map3DPlanesGraphv2.h"
#include "Map3DPlanesGraphv3.h"
#include "Map3DPlanesGraphv4.h"

#endif
