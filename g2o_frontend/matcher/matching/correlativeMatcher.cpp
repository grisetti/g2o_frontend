#include "correlativeMatcher.h"


using namespace std;
using namespace Eigen;



const float CorrelativeMatcherResult::matchingScore() const
{
  return _matchingScore;
}


CorrelativeMatcherResult::~CorrelativeMatcherResult() {}



CorrelativeMatcher::CorrelativeMatcher(const float& resolution, const float& radius, const int& kernelSize,
				       const float& kernelMaxValue, int kscale_)
: ScanMatcher(resolution, radius, kernelSize, kernelMaxValue)
{
  _kscale = kscale_;
}


CorrelativeMatcher::CorrelativeMatcher(const _GridMap<float>& g, const int& kernelSize,
				       const float& kernelMaxValue, int kscale_)
: ScanMatcher(g, kernelSize, kernelMaxValue)
{
  _kscale = kscale_;
}


CorrelativeMatcher::~CorrelativeMatcher()
{
  delete cmr;
}


void CorrelativeMatcher::addToPrunedMap(map<DiscreteTriplet, CorrelativeMatcherResult*>& myMap, const CorrelativeMatcherResult* mr,
					const float& dx, const float& dy, const float& dth)
{
  DiscreteTriplet currentTriplet(mr->_transformation, dx, dy, dth);
  map<DiscreteTriplet, CorrelativeMatcherResult*>::iterator it = myMap.find(currentTriplet);
  if(it != myMap.end())
  {
    // another triplet is found;
    if(it->second->_matchingScore > mr->_matchingScore)
    {
      it->second->_transformation = mr->_transformation;
      it->second->_matchingScore = mr->_matchingScore;
    }
  }
  else
  {
    CorrelativeMatcherResult* cmr = new CorrelativeMatcherResult;
    cmr->_transformation = mr->_transformation;
    cmr->_matchingScore= mr->_matchingScore;
    myMap.insert(make_pair(currentTriplet, cmr));
  }
}


void CorrelativeMatcher::scanMatch(const Vector2fVector& points, const Vector3f& lowerLeftF,
				   const Vector3f& upperRightF, const float& thetaRes, const float& maxScore)
{
  vector<CorrelativeMatcherResult*> mresvec;
  float dx = _scanGrid.resolution()*4;
  float dy = _scanGrid.resolution()*4;
  float dth = thetaRes *4;
  scanMatch(mresvec, points, lowerLeftF, upperRightF, thetaRes, maxScore, dx, dy, dth);
  if(mresvec.size())
  {
    CorrelativeMatcherResult* cmr = new CorrelativeMatcherResult;
    cmr->_transformation = mresvec[0]->_transformation;
    cmr->_matchingScore = mresvec[0]->_matchingScore;
    _matchResults.push_back(cmr);
  }
  
  else
  {
    CorrelativeMatcherResult* cmr = new CorrelativeMatcherResult;
    cmr->_transformation = Vector3f(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
    cmr->_matchingScore = numeric_limits<float>::max();
    _matchResults.push_back(cmr);
  }
}


void CorrelativeMatcher::scanMatch(vector<CorrelativeMatcherResult*>& mresvec,const Vector2fVector& points,
				   const Vector3f& lowerLeftF, const Vector3f& upperRightF, const float& thetaRes,
				   const float& maxScore, const float& dx, const float& dy, const float& dth)
{
  RegionVector regions(1);
  regions[0].lowerLeft = lowerLeftF;
  regions[0].upperRight = upperRightF;
  MatchingParameters params;
  params.searchStep = Vector3f(_scanGrid.resolution(), _scanGrid.resolution(), thetaRes);
  params.maxScore = maxScore;
  params.resultsDiscretization = Vector3f(dx,dy,dth);
  return scanMatch(mresvec, points, regions,params);
}


void CorrelativeMatcher::scanMatch(vector<CorrelativeMatcherResult*>& mresvec, const Vector2fVector& points,
				   const RegionVector& regions, const MatchingParameters& params)
{ 
//   float bestScore = numeric_limits<float>::max();

  map<DiscreteTriplet, CorrelativeMatcherResult*> resultMap;
  Vector2iVector intPoints(points.size());
  float thetaRes = params.searchStep.z();
  int xSteps = params.searchStep.x()/_scanGrid.resolution();
  int ySteps = params.searchStep.y()/_scanGrid.resolution();
  float maxScore = params.maxScore;
  if(xSteps <= 0)
    xSteps = 1;
  if(ySteps <= 0)
    ySteps = 1;

  int additions = 0;
  for(RegionVector::const_iterator rit = regions.begin(); rit != regions.end(); ++rit)
  {
    Vector3f lowerLeftF = rit->lowerLeft;
    Vector3f upperRightF = rit->upperRight;
    Vector2i lowerLeft = _scanGrid.world2grid(Vector2f(lowerLeftF.x(),  lowerLeftF.y()));
    Vector2i upperRight = _scanGrid.world2grid(Vector2f(upperRightF.x(), upperRightF.y()));

    for(float t = lowerLeftF.z(); t < upperRightF.z(); t += thetaRes)
    {
      float c = cos(t), s = sin(t);
      Vector2i previousPoint(-10000,-10000);
      int allocatedPoints = 0;
      const Vector2f* _p = &(points[0]);
      Vector2i* _ip = &(intPoints[0]);
      for(size_t i = 0; i < points.size(); ++i)
      {	
	Vector2f p(c*_p->x()-s*_p->y(), s*_p->x()+c*_p->y());
	Vector2i ip(p.x()*_scanGrid.inverseResolution(), p.y()*_scanGrid.inverseResolution());
	if(ip.x() != previousPoint.x() || ip.y() != previousPoint.y())
	{
	  *_ip = ip;
	  _ip++;
	  allocatedPoints++;
	  previousPoint = ip;
	}
	_p++;
      }  
      float ikscale = 1./(float)(_kscale);
      for(int i = lowerLeft.x(); i < upperRight.x(); i += xSteps)
      {
	for(int j = lowerLeft.y(); j < upperRight.y(); j += ySteps)
	{	  
	  float dsum = 0;
	  Vector2i offset(i,j);
	  const Vector2i* _ip = &(intPoints[0]);
	  for(unsigned int ii = 0; ii < allocatedPoints; ++ii)
	  {
	    Vector2i ip=*_ip+offset;
	    _ip++;
	    if(ip.x() >= 0 && ip.x() < _scanGrid.size().x() && ip.y() >= 0 && ip.y() < _scanGrid.size().y())
	    {
	      dsum += _convolvedGrid.cell(ip);
	    }
	  }
	  Vector2f mp(_scanGrid.grid2world(Vector2i(i,j)));
	  Vector3f current(mp.x(), mp.y(), t);
	  if(dsum < maxScore)
// 	  if(dsum < bestScore)
	  {
	    maxScore = dsum;
// 	    bestScore = dsum;
	    CorrelativeMatcherResult* cmr = new CorrelativeMatcherResult;
	    cmr->_transformation = current;
	    cmr->_matchingScore = dsum;
	    additions++;
	    addToPrunedMap(resultMap, cmr, params.resultsDiscretization.x(),
			   params.resultsDiscretization.y(), params.resultsDiscretization.z());
	    
	    delete cmr;
	  }
	}
      }
    }
  }
  
  CorrelativeMatcherResult* cmr = new CorrelativeMatcherResult;
  cmr->_transformation = Vector3f(.0, .0, .0);
  cmr->_matchingScore = 0;
  mresvec.resize(resultMap.size(), cmr);
  
  delete cmr;
  
  unsigned int k = 0;
  for(map<DiscreteTriplet, CorrelativeMatcherResult*>::iterator it = resultMap.begin(); it != resultMap.end(); ++it)
  {
    mresvec[k++] = it->second;
  }
//   cerr << "bareResults= " << additions << "/"  << mresvec.size() << endl;

  Comparator comp;
  sort(mresvec.begin(), mresvec.end(), comp);
}