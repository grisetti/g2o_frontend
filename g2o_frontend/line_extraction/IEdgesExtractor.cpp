#include "IEdgesExtractor.h"
#include <iostream>
#include <iomanip>
//#include <R2.h>

using namespace std;

const double IEdgesExtractor::errorThreshold = 0.01;
const double IEdgesExtractor::epsilon = 0.01;
const double IEdgesExtractor::maxVerticesDistance = 0.01;

/*
 * 	The method of least squares is a standard technique used to find
 *  the equation of a straight line from a set of data. Equation for a
 *  straight line is given by
 *	y = mx + b
 *  where m is the slope of the line and b is the y-intercept.
 *
 *  Given a set of n points {(x1,y1), x2,y2),...,xn,yn)}, let
 *      SUMx = x1 + x2 + ... + xn
 *      SUMy = y1 + y2 + ... + yn
 *      SUMxy = x1*y1 + x2*y2 + ... + xn*yn
 *      SUMxx = x1*x1 + x2*x2 + ... + xn*xn
 *
 *  The slope and y-intercept for the least-squares line can be
 *  calculated using the following equations:
 *        slope (m) = ( SUMx*SUMy - n*SUMxy ) / ( SUMx*SUMx - n*SUMxx )
 *  y-intercept (b) = ( SUMy - slope*SUMx ) / n
 */
Edge * IEdgesExtractor::fitLine(const VerticesMap & someVertices, int edgeID, const double errThs) const{

	if(someVertices.size() < 2){
		return 0;
	}

	double SUMx = 0;
	double SUMy = 0;
	double SUMxy = 0;
	double SUMxx = 0;
	double SUMyy = 0; //++
	double slope;
	double y_intercept;

	double tx = 0;
	double ty = 0;

	VerticesMap::const_iterator it = someVertices.begin();
	double minX = ((Vertex*)it->second)->x();
	double maxX = ((Vertex*)it->second)->x();
	double minY = ((Vertex*)it->second)->y();
	double maxY = ((Vertex*)it->second)->y();

	Vertex* prevV = it->second;

	while(it != someVertices.end()){

		if((errThs != 1000)//ugly but for S&M
				&& prevV->dist((*it).second) > maxVerticesDistance) {
		  return 0; //SM
		}
		prevV = (*it).second;

		tx = ((Vertex*)(*it).second)->x();
		ty = ((Vertex*)(*it).second)->y();

		SUMx += tx;
		SUMy += ty;
		SUMxy += (tx*ty);
		SUMxx += (tx*tx);
		SUMyy += (ty*ty); //++

		if(tx<minX){
			minX = tx;
		}
		if(tx > maxX){
			maxX = tx;
		}
		if(ty<minY){
			minY = ty;
		}
		if(ty>maxY){
			maxY = ty;
		}
		++it;
	}

	int num = someVertices.size();
	double slopeNumerator	= (num*SUMxy)-(SUMx*SUMy);
	double slopeDenum		= (num*SUMxx)-(SUMx*SUMx);

	slope = slopeNumerator / slopeDenum;
	y_intercept = ((SUMxx*SUMy)-(SUMx*SUMxy)) / slopeDenum;

	double SUMres = 0;
	double res = 0;
	Edge * edge = 0;

	//// case its ok for vertical distance
	for(it = someVertices.begin(); it != someVertices.end(); ++it){

		tx = ((Vertex*)(*it).second)->x();
		ty = ((Vertex*)(*it).second)->y();

		res = (ty - slope*tx) - y_intercept;
		SUMres += res*res;
	}

	if(SUMres < errThs){

		Eigen::Vector2d min, max;
		min[0] = minX;
		min[1] = minY;
		max[0] = maxX;
		max[1] = maxY;
		edge = segmentalize(slope, y_intercept, min, max);
		edge->id = edgeID;
	}

	if(edge != 0){
		return edge;
	}

	//// case its ok for orizontal distance
	SUMres = 0;

	slopeNumerator	= (SUMx*SUMy)-(num*SUMxy);
	slopeDenum		= (SUMy*SUMy)-(num*SUMyy);
	slope = slopeNumerator / slopeDenum;
	y_intercept = ((SUMy*SUMxy)-(SUMyy*SUMx)) / slopeDenum;

	for(it = someVertices.begin(); it != someVertices.end(); ++it){

		tx = ((Vertex*)(*it).second)->x();
		ty = ((Vertex*)(*it).second)->y();

		res = (tx - slope*ty) - y_intercept;
		SUMres += res*res;
	}

	Edge * e = 0;

	if(SUMres < errThs){ //SM

		Eigen::Vector2d min, max;
		min[0] = minY;
		min[1] = minX;
		max[0] = maxY;
		max[1] = maxX;
		edge = segmentalize(slope, y_intercept, min, max);
		e->id = edgeID;

		Vertex* f = new Vertex(edgeID,e->getVertexFrom().y(), e->getVertexFrom().x());
		Vertex* t = new Vertex(edgeID,e->getVertexTo().y(), e->getVertexTo().x());

		edge = new Edge(edgeID, f, t);

		delete e;
	}

	return edge;

}

Edge * IEdgesExtractor::fitLineTLS(const VerticesMap& someVertices, int edgeID, const double errThs) const{

	// TODO
	return 0;

}

Edge * IEdgesExtractor::fitLine(const VerticesMap & someVertices, int edgeID, bool & edgeFound, const double errThs) const{
	Edge * edge = fitLine(someVertices, edgeID, errThs);

	if (edge == 0){
		edgeFound = false;
	}else{
		edgeFound = true;
	}
	return edge;
}

Edge * IEdgesExtractor::fitLineTLS(const VerticesMap & someVertices, int edgeID, bool & edgeFound, const double errThs) const{
	Edge * edge = fitLineTLS(someVertices, edgeID, errThs);

	if (edge == 0){
		edgeFound = false;
	}else{
		edgeFound = true;
	}
	return edge;
}

Edge * IEdgesExtractor::segmentalize(double m, double q, const Eigen::Vector2d & mi, const Eigen::Vector2d & ma) const{

	Eigen::Vector2d min;
	//Position min;
	Eigen::Vector2d max;
	//Position max;

	if(m < 0){
// 		min.setX(mi.x());
// 		min.setY(ma.y());
// 		max.setX(ma.x());
// 		max.setY(mi.y());
			min[0] = mi.x();
			min[1] = ma.y();
			max[0] = ma.x();
			max[1] = mi.y();
			
	}else{
// 		min.setX(mi.x());
// 		min.setY(mi.y());
// 		max.setX(ma.x());
// 		max.setY(ma.y());
			min[0] = mi.x();
			min[1] = mi.y();
			max[0] = ma.x();
			max[1] = ma.y();
	}
	
	float mm = m*m;
	float mq = m*q;

	float x1 = (min.x() + m*min.y() - mq) / (1+mm);
	float y1 = (min.x()*m + mm*min.y() + q) / (1+mm);
	float x2 = (max.x() + m*max.y() - mq) / (1+mm);
	float y2 = (max.x()*m + mm*max.y() + q) / (1+mm);

	Vertex* v1 = new Vertex(-1, x1, y1);
	Vertex* v2 = new Vertex(-1, x2, y2);
	return new Edge(0, v1, v2);

}

void IEdgesExtractor::purgeBoundaryVertices(VerticesMap& map_vertex, Edges & satEdges, float maxRho, float minRho){
	VerticesMap::iterator it = map_vertex.begin();
	int i = 0;
	Vertex * from = 0;
	Vertex * to = 0;
	while(it != map_vertex.end()){

		//not too close
		if(((Vertex*)it->second)->norm() > (minRho+epsilon)){
			if(((Vertex*)it->second)->norm() >= (maxRho - epsilon)){
				//erased
				if(from == 0){
					from = it->second;
				}else{
					to = it->second;
				}
				map_vertex.erase(it++);
			}else{
				//ok
				if(to != 0){ //have a saturated edge of at least 2 points
					Edge * e = new Edge(0);
					e->setVertexFrom(*from);
					e->setVertexTo(*to);
					satEdges.push_back(e);
					to = 0;
				}
				from = 0;
				++it;
			}
		}else{
			map_vertex.erase(it++);
		}
		i++;
	}
	if(to != 0){ //have a saturated edge of at least 2 points
		Edge * e = new Edge(0);
		e->setVertexFrom(*from);
		e->setVertexTo(*to);
		satEdges.push_back(e);
		to = 0;
	}

}

void IEdgesExtractor::sort(Edges& ve) const{
  
	int i, j, first;
	Edge * temp;
	int l = ve.size( );
	for (i= l - 1; i > 0; i--){

		first = 0;                 // initialize to subscript of first element
		for (j=1; j<=i; j++){   // locate smallest between positions 1 and i.


// 			double a = ve[j]->getVertexFrom().bearing().dCastDeg();
// 			double b = ve[first]->getVertexFrom().bearing().dCastDeg();

//fix it: Bearing = phase or anomaly of the vector.....theta of the vertex?!?!?!
		double a = ve[j]->getVertexFrom().theta();
		double b = ve[first]->getVertexFrom().theta();
		

//!!!!!!!!!!!!!!!! TODO code only for kephera symulated

// 			if(a > 180){
// 				a -= 225;
// 			}else{
// 				a +=135;
// 			}
// 
// 			if(b > 180){
// 				b -= 225;
// 			}else{
// 				b +=135;
// 			}

//!!!!!!!!!!!!!!!! END code only for kephera symulated

			if (a > b)
				first = j;
		}
		temp = ve[first];   // Swap smallest found with element in position i.
		ve[first] = ve[i];
		ve[i] = temp;
	}
     return;
}

void IEdgesExtractor::mergeCollinearSegments(Edges& edges){
	//TODO

// 	for each edge e1{
// 		for each edge e2{
// 			if(e1.slope ~= e2.slope &&
// 				e1.y_intercept ~= e2.y_intercept &&
// 				e1.enoughtClose(e2)){
// 
// // 				VerticesMap vertices;
// // 				vertices[e1.from.id] = e1.from
// // 				vertices[e1.to.id] = e1.to
// // 				vertices[e2.from.id] = e2.from
// // 				vertices[e2.to.id] = e2.to
// // 
// // 				fitLine(vertices)
// 			}
// 		}
// 	}
}
