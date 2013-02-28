#ifndef CHARHIERMATCHER_H
#define CHARHIERMATCHER_H


#include "charCorrMatcher.h"



class HierarchicalCharMatcherResult : public CorrelativeCharMatcherResult
{
    friend class HierarchicalCharMatcher;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual float matchingScore() const;
    virtual ~HierarchicalCharMatcherResult();
};


class HierarchicalCharMatcher : public CorrelativeCharMatcher
{

public:
    HierarchicalCharMatcher(const float& resolution, const float& radius, const int& kernelSize,
                            const float& kernelMaxValue, int _gridKScale = 128);


    HierarchicalCharMatcher(const CharGrid& g, const int& kernelSize,
                            const float& kernelMaxValue, int _gridKScale = 128);


    virtual ~HierarchicalCharMatcher();


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
#endif // CHARHIERMATCHER_H
