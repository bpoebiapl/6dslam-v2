#include "GraphForCut.h"

using namespace std;

GraphForCut::~GraphForCut(){}
vector<vector<int> * > * GraphForCut::segment(GraphEdge * graph, int nr_edges, int nr_vertexes){
	//printf("nr_vertexes: %i\n",nr_vertexes);
	//printf("nr_edges: %i\n",nr_edges);
	int * edge_counter = new int[nr_vertexes];
	for(int i = 0; i < nr_vertexes; i++){edge_counter[i] = 0;}
	for(int i = 0; i < nr_edges; i++){
		float v = graph[i].value;
		edge_counter[graph[i].vertex1]++;
		edge_counter[graph[i].vertex2]++;
	}
	
	int ** edges_ind = new int*[nr_vertexes];
	float ** edges_val = new float*[nr_vertexes];
	int * edge_counter_tmp = new int[nr_vertexes];
	for(int i = 0; i < nr_vertexes; i++){
		int c = edge_counter[i];
		edges_ind[i] = new int[c];
		edges_val[i] = new float[c];
		edge_counter_tmp[i] = 0;
	}
	
	int * segment_id 	= new int[nr_vertexes];
	for(int i = 0; i < nr_vertexes; i++){segment_id[i]=0;}
	for(int i = 0; i < nr_edges; i++){
		float v = graph[i].value;
		int v1 = graph[i].vertex1;
		int v2 = graph[i].vertex2;
		int v1_c = edge_counter_tmp[v1];
		int v2_c = edge_counter_tmp[v2];
		//printf("%f %i %i %i %i\n",v,v1,v2,v1_c,v2_c);
		edges_val[v1][v1_c] = v;	edges_val[v2][v2_c] = v;
		edges_ind[v1][v1_c] = v2;	edges_ind[v2][v2_c] = v1;	
		edge_counter_tmp[v1]++;		edge_counter_tmp[v2]++;
	}
	
	vector< vector< int > * > * segments = new vector< vector< int > * >();
	
	int current_segment_id = 0;
	vector< int > * current_segment = new vector< int >();
	int * current_segment_ind = new int[nr_vertexes];
	for(int i = 0; i < nr_vertexes; i++){
		current_segment->push_back(i);
		current_segment_ind[i] = i;
	}
	
	//while(current_segment.size()>0)
	{
		vector< float > * current_score = new vector< float >();
		for(int i = 0; i < current_segment->size(); i++){
			int current_i = current_segment->at(i);
			float sum = 0;
			for(int j = 0; j < edge_counter[current_i]; j++){
				if(segment_id[edges_ind[current_i][j]] == current_segment_id){sum+=edges_val[current_i][j];}
			}
			current_score->push_back(sum);
		}
		vector< int > * next_segment = new vector< int >();
		while(current_segment->size() > 1){
			float worst = current_score->at(0);
			int worst_index_val = current_segment->at(0);
			int worst_index = 0;
			for(int i = 1; i < current_score->size(); i++){
				if(worst > current_score->at(i)){
					worst = current_score->at(i);
					worst_index_val = current_segment->at(i);
					worst_index = i;
				}
			}
			if(worst <= 0){
				for(int j = 0; j < edge_counter[worst_index_val]; j++){
					int tmp = current_segment_ind[edges_ind[worst_index_val][j]];
					if(tmp != -1){current_score->at(tmp) -= edges_val[worst_index_val][j];}
				}
				current_score->at(worst_index) 	= current_score->back();		current_score->pop_back();
				
				current_segment_ind[current_segment->back()] 			= worst_index;
				current_segment_ind[current_segment->at(worst_index)] 	= -1;
				
				current_segment->at(worst_index) = current_segment->back();	current_segment->pop_back();
				next_segment->push_back(worst_index_val);
			}else{break;}
		}
		for(int i = 0; i < current_segment->size(); i++){segment_id[current_segment->at(i)] = current_segment_id;}
		current_segment_id++;
		segments->push_back(current_segment);
		delete current_score;
		current_segment = next_segment;
	}
	
	for(int i = 0; i < nr_vertexes; i++){
		delete[] edges_ind[i];
		delete[] edges_val[i];
	}
	
	delete[] edges_ind;
	delete[] edges_val;
	delete[] edge_counter_tmp;
	delete[] edge_counter;
	delete[] segment_id;
	delete[] current_segment_ind;
	
	return segments;
}
