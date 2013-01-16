#include "Edge.h"
#include <vector>

using namespace std;

Edge::Edge(int tid): id(tid){
	from = 0;
	to = 0;
}

Edge::Edge(int tid, Vertex* tfrom, Vertex* tto): id(tid){
	from = tfrom;
	to = tto;
}

Edge::~Edge(){
	delete from;
	delete to;
}

std::vector<float> Edge::getCanonicalParams(){
	vector<float> params;
	params.push_back(-(from->y() - to->y())); // a (- modified)
	params.push_back(from->x() - to->x()); //b
	params.push_back(-((from->x()*to->y()) - (to->x()*from->y()))); //c (- modified)
	return params;
}

void Edge::setVertexFrom(const Vertex& vertex){
	delete this->from;
	this->from = new Vertex(vertex);
}

void Edge::setVertexTo(const Vertex& vertex){
	delete this->to;
	this->to = new Vertex(vertex);
}

const Vertex& Edge::getVertexFrom() const{
	return *from;
}
const Vertex& Edge::getVertexTo() const{
	return *to;
}
float Edge::getSlopeInRad() const{

	float c1 = (to->x()-from->x());
	float c2 = (from->y()-to->y());	
	float alpha = atan2(c2,c1);
	
	return alpha;
}
