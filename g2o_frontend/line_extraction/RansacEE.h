
#ifndef RANSAC_EE_H
#define RANSAC_EE_H

#include <iostream>
#include <map>
#include <set>
#include "IEdgesExtractor.h"

using namespace std;

class RansacEE : public virtual IEdgesExtractor{
public:
	RansacEE();
	virtual ~RansacEE();
	virtual void step(VerticesMap &map_vertex, Edges &set_edge);

private:
  vector<int> randIndices(int range);
  float distPE (Vertex* v, Edge* e);
  VerticesMap getInlierSet(Edge* e, VerticesMap map, float th);
  Edges merge(Edges VectorEdges, float th);
};

#endif //RANSAC_EE_H



