#include "frame.h"
#include <fstream>

#include "g2o_frontend/basemath/bm_se3.h"

using namespace std;

namespace pwn {

bool Frame::load(Eigen::Isometry3f &T, const char *filename) {
  ifstream is(filename);
  if (!is)
    return false;
  return load(T, is);
}

bool Frame::load(Eigen::Isometry3f &T, istream &is) {
  _points.clear();
  _normals.clear();
  char buf[1024];
  is.getline(buf, 1024);
  istringstream ls(buf);
  string tag;
  size_t numPoints;
  bool binary;
  ls >> tag;
  if (tag!="POINTWITHNORMALVECTOR")
    return false;
  ls >> numPoints >> binary;
  _points.resize(numPoints);
  _normals.resize(numPoints);
  cerr << "reading " << numPoints << " points, binary :" << binary << endl;
  is.getline(buf, 1024);
  istringstream lst(buf);
  Vector6f transform;
  lst >> transform[0] >> transform[1] >> transform[2] >> transform[3] >> transform[4] >> transform[5];
  T = v2t(transform);
  size_t k = 0;
  while (k < _points.size() && is.good()) {
    Point& point = _points[k];
    Normal& normal = _normals[k];
    if (!binary) {
      is.getline(buf, 1024);
      istringstream ls(buf);
      string s;
      ls >> s;
      if (s!="POINTWITHNORMAL")
	continue;
      for (int i=0; i<3 && ls; i++) {
	ls >> point[i];
      }
      for (int i=0; i<3 && ls; i++) {
	ls >> normal[i];
      }
    } else {
      is.read((char*) &point, sizeof(Point));
      is.read((char*) &normal, sizeof(Normal));
    }
    k++;
  }
  return is.good();
}

bool Frame::save(const char *filename, int step, bool binary, Eigen::Isometry3f T) {
  ofstream os(filename);
  if (!os)
    return false;
  return save(os, step, binary, T);  
}

  bool Frame::save(ostream &os, int step, bool binary, Eigen::Isometry3f T) {
  os << "POINTWITHNORMALVECTOR " << _points.size()/step << " " << binary << endl; 
  Vector6f transform = t2v(T);
  os << transform[0] << " " 
     << transform[1] << " " 
     << transform[2] << " " 
     << transform[3] << " " 
     << transform[4] << " " 
     << transform[5] << " " 
     << endl;
  for(size_t i = 0; i < _points.size(); i+=step) {
    const Point& point = _points[i];
    const Normal& normal = _normals[i];
    if (! binary) {
      os << "POINTWITHNORMAL ";
      for (int k=0; k<3; k++)
	os << point[k] << " ";
      for (int k=0; k<3; k++) {
	if(_normals.size() == _points.size())
	  os << normal[k] << " ";
	else {
	  float zero = 0.0f;
	  os << zero << " ";
	}
      }
      os << endl;
    } else {
      os.write((const char*) &point, sizeof(Point));
      if(_normals.size() == _points.size())
	os.write((const char*) &normal, sizeof(Normal));
      else {
	const Normal zero = Normal(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
	os.write((const char*) &zero, sizeof(Normal));
      }
    }
  }
  return os.good();
}

void Frame::clear(){
  _points.clear();
  _normals.clear(); 
  _stats.clear();
  _pointInformationMatrix.clear();
  _normalInformationMatrix.clear();
  _gaussians.clear();
  _traversabilityVector.clear();
}

void Frame::add(Frame frame, const Eigen::Isometry3f &T) {
  frame.transformInPlace(T);
  size_t k = _points.size();
  _points.resize(k + frame.points().size());
  _normals.resize(k + frame.normals().size());
  _stats.resize(k + frame.stats().size());
  _pointInformationMatrix.resize(k + frame.pointInformationMatrix().size());
  _normalInformationMatrix.resize(k + frame.normalInformationMatrix().size());
  _gaussians.resize(k + frame.gaussians().size());
  //_traversabilityVector.resize(k + frame.traversabilityVector().size())
  for(int i = 0; k < _points.size(); k++, i++) {
    _points[k] = frame.points()[i];
    _normals[k] = frame.normals()[i];
    _stats[k] = frame.stats()[i];
    _pointInformationMatrix[k] = frame.pointInformationMatrix()[i];
    _normalInformationMatrix[k] = frame.normalInformationMatrix()[i];
    _gaussians[k] = frame.gaussians()[i];
    //_traversabilityVector[k] = frame.traversabilityVector()[i];
  }
}

void Frame::transformInPlace(const Eigen::Isometry3f& T){
  Eigen::Matrix4f m = T.matrix();
  m.row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  if(m != Eigen::Matrix4f::Identity()) {
    _points.transformInPlace(m);
    _normals.transformInPlace(m);
    _stats.transformInPlace(m);
    _gaussians.transformInPlace(m);
    m.row(3) << 0,0,0,0;
    m.col(3) << 0,0,0,0;
    _pointInformationMatrix.transformInPlace(m);
    _normalInformationMatrix.transformInPlace(m);
  }
}

}
