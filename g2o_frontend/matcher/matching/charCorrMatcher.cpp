#include "charCorrMatcher.h"
#include <stdio.h>

using namespace std;
using namespace Eigen;



float CorrelativeCharMatcherResult::matchingScore() const
{
  return _matchingScore;
}


CorrelativeCharMatcherResult::~CorrelativeCharMatcherResult() {}



CorrelativeCharMatcher::CorrelativeCharMatcher(const float& resolution, const float& radius, const int& kernelSize,
						const float& kernelMaxValue)
: CharMatcher(resolution, radius, kernelSize, kernelMaxValue)
{}


CorrelativeCharMatcher::CorrelativeCharMatcher(const CharGrid& g, const int& kernelSize,
				       const float& kernelMaxValue)
: CharMatcher(g, kernelSize, kernelMaxValue)
{}


CorrelativeCharMatcher::~CorrelativeCharMatcher()
{
  delete cmr;
}


void CorrelativeCharMatcher::addToPrunedMap(map<DiscreteTriplet, CorrelativeCharMatcherResult*>& myMap,
					    const CorrelativeCharMatcherResult* mr, const float& dx, const float& dy, const float& dth)
{
  DiscreteTriplet currentTriplet(mr->_transformation, dx, dy, dth);
  map<DiscreteTriplet, CorrelativeCharMatcherResult*>::iterator it = myMap.find(currentTriplet);
  if(it != myMap.end())
  {
    if(it->second->_matchingScore > mr->_matchingScore)
    {
      it->second->_transformation = mr->_transformation;
      it->second->_matchingScore = mr->_matchingScore;
    }
  }
  else
  {
    CorrelativeCharMatcherResult* cmr = new CorrelativeCharMatcherResult;
    cmr->_transformation = mr->_transformation;
    cmr->_matchingScore= mr->_matchingScore;
    myMap.insert(make_pair(currentTriplet, cmr));
  }
}


void CorrelativeCharMatcher::scanMatch(const Vector2fVector& points, const Vector3f& lowerLeftF,
				   const Vector3f& upperRightF, const float& thetaRes, const float& maxScore)
{
  float dx = _scanGrid.resolution()*4;
  float dy = _scanGrid.resolution()*4;
  float dth = thetaRes*4;
  scanMatch(points, lowerLeftF, upperRightF, thetaRes, maxScore, dx, dy, dth);
}


void CorrelativeCharMatcher::scanMatch(const Vector2fVector& points, const Vector3f& lowerLeftF, const Vector3f& upperRightF,
				       const float& thetaRes, const float& maxScore, const float& dx, const float& dy, const float& dth)
{
  RegionVector regions(1);
  regions[0].lowerLeft = lowerLeftF;
  regions[0].upperRight = upperRightF;
  MatchingParameters params;
  params.searchStep = Vector3f(_scanGrid.resolution(), _scanGrid.resolution(), thetaRes);
  params.maxScore = maxScore;
  params.resultsDiscretization = Vector3f(dx,dy,dth);
  scanMatch(points, regions, params);
}


void CorrelativeCharMatcher::scanMatch(const Vector2fVector& points, const RegionVector& regions, const MatchingParameters& params)
{
//   int bestScore = numeric_limits<int>::max();
  Vector2iVector intPoints(points.size());

  float thetaRes = params.searchStep.z();
  int maxScore = params.maxScore;
  for(RegionVector::const_iterator rit = regions.begin(); rit != regions.end(); ++rit)
  {
    Vector3f lowerLeftF = rit->lowerLeft;
    Vector3f upperRightF = rit->upperRight;
    Vector2i lowerLeft = _scanGrid.world2grid(lowerLeftF.x(),  lowerLeftF.y());
    Vector2i upperRight = _scanGrid.world2grid(upperRightF.x(), upperRightF.y());
    
    float gridInverseRes = _scanGrid.inverseResolution();
    int gridSizeX = _scanGrid.size().x();
    int gridSizeY = _scanGrid.size().y();
    for(float t = lowerLeftF.z(); t < upperRightF.z(); t += thetaRes)
    {
      float c = cos(t), s = sin(t);
      Vector2i previousPoint(-10000,-10000);
      int allocatedPoints = 0;
      
      const Vector2f* _p = &(points[0]);
      Vector2i* _ip = &(intPoints[0]);
      size_t scanSize = points.size();
      for(size_t i = 0; i < scanSize; ++i)
      {
	Vector2f p(c*_p->x()-s*_p->y(), s*_p->x()+c*_p->y());
	Vector2i ip(p.x()*gridInverseRes, p.y()*gridInverseRes);
	if(ip.x() != previousPoint.x() || ip.y() != previousPoint.y())
	{
	  *_ip = ip;
	  _ip++;
	  allocatedPoints++;
	  previousPoint = ip;
	}
	_p++;
      }
      for(int i = lowerLeft.x(); i < upperRight.x(); ++i)
      {
	for(int j = lowerLeft.y(); j < upperRight.y(); ++j)
	{	  
	  int dsum = 0;
	  Vector2i offset(i,j);
	  const Vector2i* _ip = &(intPoints[0]);
	  for(int ii = 0; ii < allocatedPoints; ++ii)
	  {
	    Vector2i ip=*_ip + offset;
	    _ip++;
	    if(ip.x() >= 0 && ip.x() < gridSizeX && ip.y() >= 0 && ip.y() < gridSizeY)
	    {
	      dsum += (int) _convolvedGrid.cell(ip);
	    }
	  }
// 	  if(dsum < bestScore)
	  if(dsum < maxScore)
	  {
	    Vector2f mp(_scanGrid.grid2world(Vector2i(i,j)));
	    Vector3f current(mp.x(), mp.y(), t);
	    maxScore = dsum;
// 	    bestScore = dsum;
	    cmr = new CorrelativeCharMatcherResult;
	    cmr->_transformation = current;
	    cmr->_matchingScore = dsum;
	  }
	}
      }
    }
    _matchResults.push_back(cmr);
  }
}
