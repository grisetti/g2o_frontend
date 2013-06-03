#include "frame.h"
#include <fstream>

using namespace std;

namespace pwn {

bool Frame::load(const char *filename) {
  ifstream is(filename);
  if (!is)
    return false;
  return load(is);
}

bool Frame::load(istream &is) {
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

bool Frame::save(const char *filename, Eigen::Isometry3f T, int step, bool binary) {
  ofstream os(filename);
  if (!os)
    return false;
  return save(os, T, step, binary);
}

bool Frame::save(ostream &os, Eigen::Isometry3f T, int step, bool binary) {
  os << "POINTWITHNORMALVECTOR " << _points.size()/step << " " << binary << endl; 
  for(size_t i = 0; i < _points.size(); i+=step) {
    const Point& point = T * _points[i];
    const Normal& normal = T * _normals[i];
    if (! binary) {
      os << "POINTWITHNORMAL ";
      for (int k=0; k<3; k++)
	os << point[k] << " ";
      for (int k=0; k<3; k++)
	os << normal[k] << " ";
      os << endl;
    } else {
      os.write((const char*) &point, sizeof(Point));
      os.write((const char*) &normal, sizeof(Normal));
    }
  }
  return os.good();
}

bool Frame::save(const char *filename, int step, bool binary) {
  ofstream os(filename);
  if (!os)
    return false;
  return save(os, step, binary);
}

bool Frame::save(ostream &os, int step, bool binary) {
  os << "POINTWITHNORMALVECTOR " << _points.size()/step << " " << binary << endl; 
  for(size_t i = 0; i < _points.size(); i+=step) {
    const Point& point = _points[i];
    const Normal& normal = _normals[i];
    if (! binary) {
      os << "POINTWITHNORMAL ";
      for (int k=0; k<3; k++)
	os << point[k] << " ";
      for (int k=0; k<3; k++)
	os << normal[k] << " ";
      os << endl;
    } else {
      os.write((const char*) &point, sizeof(Point));
      os.write((const char*) &normal, sizeof(Normal));
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
}

void Frame::transformInPlace(const Eigen::Isometry3f& T){
  Eigen::Matrix4f m = T.matrix();
  m.row(3) << 0,0,0,1;
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
