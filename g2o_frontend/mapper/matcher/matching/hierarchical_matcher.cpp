#include "hierarchical_matcher.h"


using namespace std;
using namespace Eigen;


namespace match_this
{
float HierarchicalMatcherResult::matchingScore() const
{
  return _matchingScore;
}


HierarchicalMatcherResult::~HierarchicalMatcherResult() {}


HierarchicalMatcher::HierarchicalMatcher(const float& resolution, const float& radius, const int& kernelSize,
                                                 const float& kernelMaxValue, int kscale)
    : CorrelativeMatcher(resolution, radius, kernelSize, kernelMaxValue, kscale)
{}


HierarchicalMatcher::HierarchicalMatcher(const CharGrid& g, const int& kernelSize,
                                                 const float& kernelMaxValue, int kscale)
    : CorrelativeMatcher(g, kernelSize, kernelMaxValue, kscale)
{}


HierarchicalMatcher::~HierarchicalMatcher() {}


void HierarchicalMatcher::scanMatch(const Vector2fVector& points, Vector3f lowerLeftF,
                                        Vector3f upperRightF, float thetaRes,
                                        float maxScore, float dx, float dy, float dth)
{
    RegionVector regions(1);
    regions[0].lowerLeft = lowerLeftF;
    regions[0].upperRight = upperRightF;

    MatchingParametersVector pvec(4);
    float gridRes = _scanGrid.resolution();

    pvec[0].searchStep = Vector3f(8*gridRes, 8*gridRes, 4*thetaRes);
    pvec[0].maxScore = maxScore;
    pvec[0].resultsDiscretization = Vector3f(dx*8, dy*8, dth*8);

    pvec[1].searchStep = Vector3f(4*gridRes, 4*gridRes, 2*thetaRes);
    pvec[1].maxScore = maxScore;
    pvec[1].resultsDiscretization = Vector3f(dx*4, dy*4, dth*4);

    pvec[2].searchStep = Vector3f(2*gridRes, 2*gridRes, thetaRes);
    pvec[2].maxScore = maxScore;
    pvec[2].resultsDiscretization = Vector3f(dx*2, dy*2, dth*2);

    pvec[3].searchStep = Vector3f(gridRes, gridRes, thetaRes);
    pvec[3].maxScore = maxScore;
    pvec[3].resultsDiscretization = Vector3f(dx, dy, dth);

    scanMatch(points, regions, pvec);
}


void HierarchicalMatcher::scanMatch(const Vector2fVector& points, Vector3f lowerLeftF,
                                        Vector3f upperRightF, float thetaRes, float maxScore,
                                        float dx, float dy, float dth, int nLevels)
{
    RegionVector regions(1);
    regions[0].lowerLeft = lowerLeftF;
    regions[0].upperRight = upperRightF;

    scanMatch(points, regions, thetaRes, maxScore, dx, dy, dth, nLevels);
}


void HierarchicalMatcher::scanMatch(const Vector2fVector& points,
                                        const RegionVector& regions, float thetaRes, float maxScore,
                                        float dx, float dy, float dth, int nLevels)
{
    MatchingParametersVector pvec;
    float gridRes = _scanGrid.resolution();
    for(int i = nLevels-1; i >= 0; i--)
    {
        int m = pow(2, i);
        int mtheta = (m/2 < 1) ? m : m/2;

        MatchingParameters mp;
        mp.searchStep = Vector3f(m*gridRes, m*gridRes, mtheta*thetaRes);
        mp.maxScore = maxScore;
        mp.resultsDiscretization = Vector3f(dx*m, dy*m, dth*m);

        pvec.push_back(mp);
    }

    scanMatch(points, regions, pvec);
}


void HierarchicalMatcher::scanMatch(const Vector2fVector& points, const RegionVector& regions,
                                        const MatchingParametersVector& paramsVec)
{
    vector<CorrelativeMatcherResult*> mresvec;
    RegionVector currentRegions = regions;
    for(unsigned int i = 0; i < paramsVec.size() -1; ++i)
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



//void HierarchicalCharMatcher::scanMatch(const Vector2fVector& points, const RegionVector& regions,
//                                        const MatchingParametersVector& paramsVec)
//{
//    vector<CorrelativeCharMatcherResult*> mresvec;
//    RegionVector currentRegions = regions;
//    for(size_t i = 0; i < paramsVec.size()-1; ++i)
//    {
//        MatchingParameters params = paramsVec[i];
//        CorrelativeCharMatcher::scanMatch(mresvec, points, currentRegions, params);
//        if(mresvec.size())
//        {
//            currentRegions.clear();
//            for(size_t i = 0; i < mresvec.size(); ++i)
//            {
//                Vector3f best = mresvec[i]->_transformation;
//                Region reg;
//                Vector3f newlower = -(params.resultsDiscretization * .5) + best;
//                Vector3f newupper = (params.resultsDiscretization * .5) + best;

//                reg.lowerLeft = Vector3f(newlower.x(), newlower.y(), newlower.z());
//                reg.upperRight = Vector3f(newupper.x(), newupper.y(), newupper.z());
//                currentRegions.push_back(reg);
//            }
//        }
//        else
//        {
//            break;
//        }
//    }

//    if(mresvec.size())
//    {
//        MatchingParameters params = paramsVec[paramsVec.size()-1];
//        CorrelativeCharMatcher::scanMatch(mresvec, points, currentRegions, params);
//        HierarchicalCharMatcherResult* hmr = new HierarchicalCharMatcherResult;
//        hmr->_transformation = mresvec[0]->_transformation;
//        hmr->_matchingScore = mresvec[0]->matchingScore();
//        _matchResults.push_back(hmr);
//    }
//    else
//    {
//        HierarchicalCharMatcherResult* hmr = new HierarchicalCharMatcherResult;
//        hmr->_transformation = Vector3f(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
//        hmr->_matchingScore = numeric_limits<float>::max();
//        _matchResults.push_back(hmr);
//    }
//}
}
