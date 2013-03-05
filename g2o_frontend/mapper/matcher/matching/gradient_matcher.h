#ifndef GRADIENTMATCHER_H
#define GRADIENTMATCHER_H

#include "scan_matcher.h"



class GradientMatcherResult : public ScanMatcherResult
{
  friend class GradientMatcher;
  public:
    virtual float matchingScore() const;
    virtual ~GradientMatcherResult();
  
};


class GradientMatcher : public ScanMatcher
{
  public:    
    GradientMatcher(const float& resolution, const float& radius, const int& kernelSize,
			const float& kernelMaxValue, const Eigen::Vector3f& baseMove = Eigen::Vector3f(0.1, 0.1, 0.3));
    
    GradientMatcher(const CharGrid& g, const int& kernelSize,
			const float& kernelMaxValue, const Eigen::Vector3f& baseMove = Eigen::Vector3f(0.1, 0.1, 0.3));

    
    virtual ~GradientMatcher();
    
    void scanMatch(const Vector2fVector& p, const Eigen::Vector3f& ig);

  protected:
    int partialScore(const Vector2fVector& scan, const Eigen::Vector3f& t);
    
    Eigen::Vector3f _moves[6];
    GradientMatcherResult* gmr;
};

#endif // GRADIENTMATCHER_H
