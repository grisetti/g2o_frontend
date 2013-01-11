#ifndef EDGE_H
#define EDGE_H

#include <vector>
#include <cmath>
#include <iostream>
// #include <complex>
// #include <R2.h>
// #include <S1.h>

struct Vertex{
  Vertex(int id): _id(id){}
  // 	Vertex(int id, float x, float y, bool a) : _x(x), _y(y), _id(id){
  // 		std::cout << "1" << std::endl;
  // 	}
  Vertex(int id, float rho, float theta) : _id(id){
    _x = rho*cos(theta);
    _y = rho*sin(theta);
  }
  // 	Vertex(int id, float rho, float theta) : _rho(rho), _theta(theta), _id(id){
  // 	}
	
  double dist(const Vertex* v) const{
    return sqrt(pow(v->x()-_x, 2) + pow(v->y()-_y, 2));
  }
  double norm() const{
    return sqrt(pow(x(),2) + pow(y(),2));
  }
  float x() const { return _x; }
  float y() const { return _y; }
  void setX(const float x) { _x = x; }
  void setY(const float y) { _y = y; }
  float theta() const { return atan2(_y,_x); }
  //float theta() const { return _theta; }
  // cartesian coordinates
  float _x, _y;
  float _rho;
	
  int _id;

  Vertex(const Vertex & other):	_x(other._x), _y(other._y), _id(other._id){}
};

class Edge{
public:
  /// simple constructor initialize vertex pointer to 0
  Edge(int id);

  /// this constructor take ownership (memory management rights) of the vertexes.
  Edge(int id, Vertex* from, Vertex* to);

  ~Edge();
  int id;

  /// return a 3 element vector representing a line in canonical form: ax + by + c = 0
  std::vector<float> getCanonicalParams();

  inline float getLength() const { return from->dist(to); }
  void setVertexFrom(const Vertex& vertex);
  void setVertexTo(const Vertex& vertex);

  const Vertex& getVertexFrom() const;
  const Vertex& getVertexTo() const;
	
  /// return the slope in radians //TODO can optimized saving a proxy result
  float getSlopeInRad() const;

private:

  Vertex* from;
  Vertex* to;

};

#endif // EDGE_H
