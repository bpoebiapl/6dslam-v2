#ifndef GraphForCut_H_
#define GraphForCut_H_

//#include "cv.h"
//#include "highgui.h"
//#include <opencv.hpp>
#include <stdlib.h>
#include <math.h>
#include <sys/types.h>
#include <sys/time.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <iostream>
#include <stdio.h>

#include <boost/thread/thread.hpp>

using namespace Eigen;
using namespace std;

struct GraphEdge {
	float value;
	int vertexes[4];
	int nr_vertexes;
};


class GraphForCut
{
	public:
	GraphForCut();
	virtual ~GraphForCut();
	virtual vector<vector<int> * > * segment(vector<GraphEdge> graph, int nr_vertexes);
};

#endif
