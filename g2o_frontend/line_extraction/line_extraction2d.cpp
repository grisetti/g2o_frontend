#include "line_extraction2d.h"
using namespace std;

void Point2DClusterer::compute() {
  _clusters.clear();
  if (!_points.size())
    return;
  _clusters.push_back(make_pair(0,0));
  for (size_t i=1; i<_points.size(); i++){
    Cluster& currentCluster=_clusters.back();
    const Vector2f& pprev = _points[currentCluster.second];
    const Vector2f& pcurr = _points[i];
    Vector2f dp = pprev - pcurr;
    if (dp.squaredNorm() > _squaredDistance) {
      _clusters.push_back(make_pair(i,i));
    } else {
      currentCluster.second = i;
    }
  }
}

Line2DExtractor::Line2DExtractor()
{

  _splitThreshold = 0.2;

}

Line2DExtractor::~Line2DExtractor()
{
}


bool Line2DExtractor::split(int k) {
  assert (_lines.find(k)!=_lines.end() && "LINE NOT IN THE SET");
  Line2D& line = _lines[k];
  if (line.p1Index - line.p0Index < _minPointsInLine)
    return false;
  // seek for the point with the larest distance between the indices contained in the line
  
  int imax= maxDistanceIndex(line);
  if (imax == -1 || imax == line.p1Index-1){
    return false;
  }
  std::pair<IntLineMap::iterator, bool> insertResult= _lines.insert(make_pair(imax, Line2D()));
  assert (insertResult.second && "FAILURE IN MAP INSERTION");;
  Line2D& newLine = insertResult.first->second;
  initializeFromIndices(newLine, imax, line.p1Index);
  initializeFromIndices(line, line.p1Index, imax);
  return true;
}

int Line2DExtractor::maxDistanceIndex(const Line2D& line){
  int imax=-1;
  float dmax=-1;
  for (int i = line.p0Index; i<= line.p1Index; i++){
    float d = line.squaredDistance(_points[i]);
    if (d>dmax && d>_splitThreshold){
      imax = i; 
      dmax = d;
    }
  }
  return imax;
}

bool Line2DExtractor::merge(int k) {
  assert (_lines.find(k)!=_lines.end() && "LINE NOT IN THE SET");
  Line2D& line1 = _lines[k];
  assert (_lines.find(k+1)!=_lines.end() && "LINE NOT IN THE SET");
  Line2D& line2 = _lines[k+1];

  Line2D line;
  initializeFromIndices(line, line1.p0Index, line2.p1Index);

  int imax= maxDistanceIndex(line);
  if (imax == -1){
    return false;
  }
  _lines.erase(k+1);
  line1 = line;
  return true;
}

void Line2DExtractor::initializeFromIndices(Line2D& line, int i0, int i1){
  line.p0Index = i0;
  line.p1Index = i1;
  line.fromPoints(_points[i0], _points[i1]);
}

void Line2DExtractor::compute(){
  assert(_points.size() && "you should have some point");
  _lines.clear();
  if (_points.size() < _minPointsInLine)
    return;
  Line2D firstLine;
  initializeFromIndices(firstLine,0,_points.size()-1);
  //bool hasSplitted = true;
  IntLineMap::iterator it = _lines.begin();
  while (it!=_lines.end()){
    /*bool splitResult=*/ split(it->first);
    it++;
  }
    cerr << "I split " << _lines.size() << " times";
}

// bool Line2DExtractor::compute(Vector2fVector& /*points*/ ,
// 			     VectorOfSubsetsPoints& /*subset*/, 
// 			     Line2DVector& /*lines*/)
// {
// 	//divide the entire set of points in subsets of points
//   return true;
	
	
	
// 	//for each set of points split()

// }



