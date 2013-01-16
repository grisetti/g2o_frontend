#include "hierarchicalMatcher.h"
#include <vector>


using namespace std;
using namespace Eigen;



const float HierarchicalMatcherResult::matchingScore() const
{
  return _matchingScore;
}


HierarchicalMatcherResult::~HierarchicalMatcherResult() {}


HierarchicalMatcher::HierarchicalMatcher(const float& resolution, const float& radius, const int& kernelSize, const float& kernelMaxValue, int kscale_)
: CorrelativeMatcher(resolution, radius, kernelSize, kernelMaxValue, kscale_)
{

}


HierarchicalMatcher::HierarchicalMatcher(const _GridMap< float >& g, const int& kernelSize, const float& kernelMaxValue, int kscale_)
: CorrelativeMatcher(kernelMaxValue, kernelMaxValue, kernelSize, kernelMaxValue, kscale_)
{

}


void HierarchicalMatcher::scanMatch(vector<CorrelativeMatcherResult*>& mresvec, const Vector2fVector& points,
				     const RegionVector& regions, const MatchingParametersVector& paramsVec)
{
  mresvec.clear();
  RegionVector currentRegions = regions;

  for(unsigned int i = 0; i < paramsVec.size(); ++i)
//   for(unsigned int i = paramsVec.size()-1; i = 0 ; ++i)
  {
    double a = getMilliSecs();
    MatchingParameters params = paramsVec[i];
    CorrelativeMatcher::scanMatch(mresvec, points, currentRegions, params);
    currentRegions.clear();
    cout << "mresvec: " << mresvec.size() << endl;
    for(unsigned int i = 0; i < mresvec.size(); ++i)
    {
      Vector3f best = mresvec[i]->_transformation;
      Region reg;
      reg.lowerLeft  = - (params.resultsDiscretization * .5) + best;
      reg.upperRight =   (params.resultsDiscretization * .5) + best;
      currentRegions.push_back(reg);
    }
    double b = getMilliSecs();
    cout << "Inner time: " << (b-a) * 1000 << " ms" << endl;
    cout << "Current regions :" << currentRegions.size() << endl;
  }
  cout << "Number of regions: " << currentRegions.size() << endl;
  MatchingParameters params = paramsVec[paramsVec.size()-1];
  CorrelativeMatcher::scanMatch(mresvec, points, currentRegions, params);
  if(mresvec.size())
  {
    HierarchicalMatcherResult* hmr = new HierarchicalMatcherResult;
    hmr->_transformation = mresvec[0]->_transformation;
    hmr->_matchingScore = mresvec[0]->matchingScore();
    _matchResults.push_back(hmr);
  }  
  else
  {
    HierarchicalMatcherResult* hmr = new HierarchicalMatcherResult;
    hmr->_transformation = Vector3f(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
    hmr->_matchingScore = numeric_limits<float>::max();
    _matchResults.push_back(hmr);
  }
}


void HierarchicalMatcher::scanMatch(vector<CorrelativeMatcherResult*>& mresvec, const Vector2fVector& points,
				    Vector3f lowerLeftF, Vector3f upperRightF, float thetaRes,
				    float maxScore, float dx, float dy, float dth, int nLevels)
{
  RegionVector regions(1);
  regions[0].lowerLeft = lowerLeftF;
  regions[0].upperRight = upperRightF;

  MatchingParametersVector pvec;
  for(int i = nLevels-1; i >= 0; i--)
  {
    int m = pow(2, i);
    int mtheta = (m*0.5 < 1) ? m : m*0.5; 
    MatchingParameters mp;
    mp.searchStep = Vector3f(m*_scanGrid.resolution(), m*_scanGrid.resolution(), mtheta*thetaRes);
    mp.maxScore = maxScore;
    mp.resultsDiscretization = Vector3f(dx*m, dy*m, dth*m);
    
    pvec.push_back(mp);
  }
  this->scanMatch(mresvec, points, regions, pvec);
}