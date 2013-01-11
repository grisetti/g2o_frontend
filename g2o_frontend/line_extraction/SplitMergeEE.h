#ifndef SPLITMERGE_EE_H
#define SPLITMERGE_EE_H

#include <iostream>
#include <map>
#include <set>
#include "IEdgesExtractor.h"

using namespace std;

class SplitMergeEE : public virtual IEdgesExtractor{
public:

	SplitMergeEE();
	virtual ~SplitMergeEE();
	virtual void step(VerticesMap &map_vertex, Edges &edges);

private:
	float distPE (Vertex* v, Edge* e);
	//Vertex* getVertexMaxDist(VerticesMap currentSet, Edge* e);
	vector<VerticesMap*> split(VerticesMap currVertices, Edge* e, Vertex* v);
	Edges merge(Edges VectorEdges, float th);
	//bool collinear(Edge* a, Edge* b, double th);
};

#endif