#ifndef IEDGESEXTRACTOR_H
#define IEDGESEXTRACTOR_H

#include <iostream>
#include <Eigen/Dense>
#include <map>
#include <set>
#include "Edge.h"

typedef std::map<int,Vertex*> VerticesMap;
typedef std::vector<Edge*> Edges;

class IEdgesExtractor{
public:
	virtual ~IEdgesExtractor(){}

	/// extract edges from vertices
	virtual void step(VerticesMap &map_vertex, Edges& set_edge) = 0;

	/// eliminate all vertices falling in an empty space (not hitting obstacles)
	/// and put some edges built over their limit in satEdges
	virtual void purgeBoundaryVertices(VerticesMap & map_vertex, Edges & satEdges, float maxRho, float minRho = 0.001);
	
	/// sort the edges based on the bearing of one of the vertices.
	virtual void sort(Edges& ve) const;

	/// unify segment whose are approximately on the same line and are close enought
	virtual void mergeCollinearSegments(Edges& edges);

protected:

	/// maximum error for the LMS fitLine algorithm
	static const double errorThreshold;

	/// little value for double comparison
	static const double epsilon;

	/// max distance between two consecutive scan point to be on the same segment
	static const double maxVerticesDistance;

	/// return an edge if all the vertices fit with a computed line
	/// else return 0. using least square;
	virtual Edge* fitLine(const VerticesMap &someVertices, int edgeID, const double errThs = IEdgesExtractor::errorThreshold) const;

	/// same as fitLine(VerticesMap&, int) but with a value that says wether the edge have been found or not
	virtual Edge* fitLine(const VerticesMap &someVertices, int edgeID, bool& edgeFound, const double errThs = IEdgesExtractor::errorThreshold) const;

	/// TO BE DONE
	/// return an edge if all the vertices fit with a computed line
	/// else return 0. using total least square; //TODO
	virtual Edge* fitLineTLS(const VerticesMap &someVertices, int edgeID, const double errThs = IEdgesExtractor::errorThreshold) const;

	/// same as fitLine(VerticesMap&, int) but with a value that says wether the edge have been found or not
	virtual Edge* fitLineTLS(const VerticesMap &someVertices, int edgeID, bool& edgeFound, const double errThs = IEdgesExtractor::errorThreshold) const;

	/// cut a line within the given bounding box if needed and return an Edge with 0 id
	virtual Edge* segmentalize(double m, double q, const Eigen::Vector2d & minPoint, const Eigen::Vector2d & maxPoint) const;

};

#endif // IEDGESEXTRACTOR_H
