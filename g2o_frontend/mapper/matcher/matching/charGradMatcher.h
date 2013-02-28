#ifndef __CHAR_GRADIENT_MATCHER_H__
#define __CHAR_GRADIENT_MATCHER_H__

#include "charMatcher.h"



class GradientCharMatcherResult : public CharMatcherResult
{
  friend class GradientCharMatcher;
  public:
    virtual float matchingScore() const;
    virtual ~GradientCharMatcherResult();
  
};


class GradientCharMatcher : public CharMatcher
{
  public:    
    GradientCharMatcher(const float& resolution, const float& radius, const int& kernelSize,
			const float& kernelMaxValue, const Eigen::Vector3f& baseMove = Eigen::Vector3f(0.1, 0.1, 0.3));
    
    GradientCharMatcher(const CharGrid& g, const int& kernelSize,
			const float& kernelMaxValue, const Eigen::Vector3f& baseMove = Eigen::Vector3f(0.1, 0.1, 0.3));

    
    virtual ~GradientCharMatcher();
    
    void scanMatch(const Vector2fVector& p, const Eigen::Vector3f& ig);

  protected:
    int partialScore(const Vector2fVector& scan, const Eigen::Vector3f& t);
    
    Eigen::Vector3f _moves[6];
    GradientCharMatcherResult* gmr;
};
#endif