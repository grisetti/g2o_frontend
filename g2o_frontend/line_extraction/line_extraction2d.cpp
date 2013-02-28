/*
 * line_extraction2d.cpp
 *
 *  Created on: Jan 15, 2013
 *      Author: Martina
 */

#include <cmath>
#include "line_extraction2d.h"
#include <fstream>
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
  _splitThreshold = 0.03*0.03;  
  _minPointsInLine = 10;
  _maxPointDistance = 3;
  _normalMergeThreshold = ::cos(M_PI/360);
  _rhoMergeThreshold = 0.05;
}


Line2DExtractor::~Line2DExtractor()
{
}


bool Line2DExtractor::split(int k) {
  assert (_lines.find(k)!=_lines.end() && "LINE NOT IN THE SET");
  Line2D& line = _lines[k];
  if (line.p1Index - line.p0Index < _minPointsInLine)
    return false;
	
  /** seek for the point with the largest distance between the indices contained in the line**/
  int imax= maxDistanceIndex(line);
//   cout<< "\t\t imax: " << imax << endl;
  if (imax <0  || imax == line.p1Index-1){
    return false;
  }
  const Vector2f& v = _points[imax];
//   cout<< "\t\t d: " << line.squaredDistance(v) << endl;
  

  std::pair<IntLineMap::iterator, bool> insertResult= _lines.insert(make_pair(imax, Line2D()));
  assert (insertResult.second && "FAILURE IN MAP INSERTION");;

  Line2D& newLine = insertResult.first->second;
  initializeFromIndices(newLine, imax, line.p1Index);
  initializeFromIndices(line, line.p0Index, imax);
//   cout << "\tfirstLine: " << line.p0Index << " " << line.p1Index << endl;
//   cout << "\tseconLine: " << newLine.p0Index << " " << newLine.p1Index << endl;
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
  IntLineMap::iterator it = _lines.find(k);
  assert (it==_lines.end() && "LINE NOT IN THE SET");
  Line2D& line1 = it->second;
  if (k == _lines.rbegin()->first)
    return false;
  
  it++;
  Line2D& line2 = it->second;;
  if (line2.p0Index-line1.p1Index > _maxPointDistance)
    return false;
  
  float normalAngleCos = line2.d().dot(line1.d());
  if (normalAngleCos > _normalMergeThreshold){
    return false;
  }
  float rho1 = line1.p().norm();
  float rho2 = line2.p().norm();
  if (fabs (rho1-rho2)>_rhoMergeThreshold){
    return false;
  }

  Line2D line;
  initializeFromIndices(line, line1.p0Index, line2.p1Index);
  
  // int imax= maxDistanceIndex(line);
  // if (imax <0  || imax == line.p1Index-1){
  //   return false;
  // }

  _lines.erase(it);
  line1 = line;
  return true;
}


void Line2DExtractor::initializeFromIndices(Line2D& line, int i0, int i1){
  line.p0Index = i0;
  line.p1Index = i1;
  line.fromPoints(_points[i0], _points[i1]);
}

void Line2DExtractor::compute(){
  assert(_points.size() && "you should have some points");
	
  _lines.clear();
  if (_points.size() < _minPointsInLine) return;
	
  Line2D firstLine;
  initializeFromIndices(firstLine,0,_points.size()-1);
  _lines.insert(make_pair(0, firstLine));
	
  IntLineMap::iterator it = _lines.begin();
	
	/** Split step **/
  while (it!=_lines.end()){
		
    const Line2D& l=it->second;
    bool splitResult= split(it->first);
//     cout << "\tsplit" << l.p0Index << " " << l.p1Index;
    if (splitResult) {
      cout << "split done" << endl;
    } else 
      cout << "split nope" << endl;
			if (! splitResult)
				it++;
		
  }
  cout << "\tI split " << _lines.size() << " times" << endl;
	
	/** Merge step **/
  it = _lines.begin();
  while (it!=_lines.end()){
    const Line2D& l=it->second;
    bool mergeResult= merge(it->first);
//     cout << "\tmerge" << l.p0Index << " " << l.p1Index;
    if (mergeResult) {
      cout << "merge done" << endl;
    } else 
      cout << "merge nope" << endl;
    if (!mergeResult)
      it++;
  }
  cout << "\tI merge " << _lines.size() << " times" << endl;
}



