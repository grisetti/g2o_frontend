#include "homogeneouspoint3fscene.h"
#include <fstream>

using namespace std;

bool HomogeneousPoint3fScene::load(const char *filename) {
  ifstream is(filename);
  if (!is)
    return false;
  return load(is);
}

bool HomogeneousPoint3fScene::load(istream &is) {
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
    HomogeneousPoint3f& point = _points[k];
    HomogeneousNormal3f& normal = _normals[k];
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
      is.read((char*) &point, sizeof(HomogeneousPoint3f));
      is.read((char*) &normal, sizeof(HomogeneousNormal3f));
    }
    k++;
  }
  return is.good();
}

bool HomogeneousPoint3fScene::save(const char *filename, int step, bool binary) {
  ofstream os(filename);
  if (!os)
    return false;
  return save(os, step, binary);
}

bool HomogeneousPoint3fScene::save(ostream &os, int step, bool binary) {
  os << "POINTWITHNORMALVECTOR " << _points.size()/step << " " << binary << endl; 
  for(size_t i = 0; i < _points.size(); i+=step) {
    const HomogeneousPoint3f& point = _points[i];
    const HomogeneousNormal3f& normal = _normals[i];
    if (! binary) {
      os << "POINTWITHNORMAL ";
      for (int k=0; k<3; k++)
	os << point[k] << " ";
      for (int k=0; k<3; k++)
	os << normal[k] << " ";
      os << endl;
    } else {
      os.write((const char*) &point, sizeof(HomogeneousPoint3f));
      os.write((const char*) &normal, sizeof(HomogeneousNormal3f));
    }
  }
  return os.good();
}
