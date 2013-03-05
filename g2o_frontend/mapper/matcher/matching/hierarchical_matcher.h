#ifndef HIERARCHICALMATCHER_H
#define HIERARCHICALMATCHER_H


#include "correlative_matcher.h"



class HierarchicalMatcherResult : public CorrelativeMatcherResult
{
    friend class HierarchicalMatcher;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual float matchingScore() const;
    virtual ~HierarchicalMatcherResult();
};


class HierarchicalMatcher : public CorrelativeMatcher
{

public:
    HierarchicalMatcher(const float& resolution, const float& radius, const int& kernelSize,
                            const float& kernelMaxValue, int _gridKScale = 128);


    HierarchicalMatcher(const CharGrid& g, const int& kernelSize,
                            const float& kernelMaxValue, int _gridKScale = 128);


    virtual ~HierarchicalMatcher();


    void scanMatch(const Vector2fVector& points,
                   Eigen::Vector3f lowerLeftF, Eigen::Vector3f upperRightF,
                   float thetaRes, float maxScore, float dx, float dy, float dth);


    void scanMatch(const Vector2fVector& points,
                   const RegionVector& regions, float thetaRes,
                   float maxScore, float dx, float dy, float dth, int nLevels);


    void scanMatch(const Vector2fVector& points,
                   const RegionVector& regions, const MatchingParametersVector& paramsVec);


    void scanMatch(const Vector2fVector& points,
                   Eigen::Vector3f lowerLeftF, Eigen::Vector3f upperRightF, float thetaRes,
                   float maxScore, float dx, float dy, float dth, int nLevels);
};

#endif // HIERARCHICALMATCHER_H
