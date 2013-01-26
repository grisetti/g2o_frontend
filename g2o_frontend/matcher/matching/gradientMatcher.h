#ifndef __GRADIENT_MATCHER_H__
#define __GRADIENT_MATCHER_H__

#include "scanMatcher.h" 



class GradientMatcherResult : ScanMatcherResult
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
    
    GradientMatcher(const _GridMap<float>& g, const int& kernelSize,
		    const float& kernelMaxValue, const Eigen::Vector3f& baseMove = Eigen::Vector3f(0.1, 0.1, 0.3));
    
    virtual ~GradientMatcher();
    
    void scanMatch(const Vector2fVector& p, const Eigen::Vector3f& ig);

  protected:
    float partialScore(const Vector2fVector& scan, const Eigen::Vector3f& t);
    
    Eigen::Vector3f _moves[6];
    GradientMatcherResult* gmr;
};
#endif