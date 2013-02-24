#ifndef CHARHIERMATCHER_H
#define CHARHIERMATCHER_H


#include "charCorrMatcher.h"



class HierarchicalCharMatcherResult : public CorrelativeCharMatcherResult
{
  friend class HierarchicalCharMatcher;
  public:
    virtual float matchingScore() const;
    virtual ~HierarchicalMatcherResult();
};


class HierarchicalCharMatcher : public CorrelativeCharMatcher
{
  public:
    HierarchicalCharMatcher(const float& resolution, const float& radius, const int& kernelSize,
                            const float& kernelMaxValue, int _gridKScale = 128);


    HierarchicalCharMatcher(const CharGrid& g, const int& kernelSize,
                            const float& kernelMaxValue, int _gridKScale = 128);


    void scanMatch(const Vector2fVector& points, Eigen::Vector3f lowerLeftF, Eigen::Vector3f upperRightF,
                            double thetaRes, double maxScore, double dx, double dy, double dth);


    void scanMatch(const Vector2fVector& points, const RegionVector& regions, double thetaRes,
                            float maxScore, float dx, float dy, float dth, int nLevels);


    void scanMatch(std::vector<CorrelativeMatcherResult*>& mresvec, const Vector2fVector& points,
                   const RegionVector& regions, const MatchingParametersVector& paramsVec);


    void scanMatch(std::vector<CorrelativeMatcherResult*>& mresvec, const Vector2fVector& points,
                   Eigen::Vector3f lowerLeftF, Eigen::Vector3f upperRightF, float thetaRes,
                   float maxScore, float dx, float dy, float dth, int nLevels);
};
#endif // CHARHIERMATCHER_H
