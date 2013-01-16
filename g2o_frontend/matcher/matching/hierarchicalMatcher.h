#ifndef __HIERARCHICAL_MATCHER_H__
#define __HIERARCHICAL_MATCHER_H__

#include "correlativeMatcher.h"



class HierarchicalMatcherResult : public CorrelativeMatcherResult
{
  friend class HierarchicalMatcher;
  public:
    virtual const float matchingScore() const;
    virtual ~HierarchicalMatcherResult();
};


class HierarchicalMatcher : public CorrelativeMatcher
{
  public:
    HierarchicalMatcher(const float& resolution, const float& radius, const int& kernelSize,
			const float& kernelMaxValue, int kscale_= 128);


    HierarchicalMatcher(const _GridMap<float>& g, const int& kernelSize,
			const float& kernelMaxValue, int kscale_ = 128);    


    void scanMatch(std::vector<CorrelativeMatcherResult*>& mresvec, const Vector2fVector& points,
			    const RegionVector& regions, const MatchingParametersVector& paramsVec);


    void scanMatch(std::vector<CorrelativeMatcherResult*>& mresvec, const Vector2fVector& points,
			    Eigen::Vector3f lowerLeftF, Eigen::Vector3f upperRightF, float thetaRes,
			    float maxScore, float dx, float dy, float dth, int nLevels);
};
#endif