#ifndef __CORRELATIVE_MATCHER_H__
#define __CORRELATIVE_MATCHER_H__

#include "scanMatcher.h"


struct Region
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Eigen::Vector3f lowerLeft;
  Eigen::Vector3f upperRight;
};
typedef std::vector<Region, Eigen::aligned_allocator<Region> > RegionVector;


struct MatchingParameters
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Eigen::Vector3f searchStep;
  Eigen::Vector3f resultsDiscretization;
  float maxScore;
};
typedef std::vector<MatchingParameters, Eigen::aligned_allocator<MatchingParameters> > MatchingParametersVector;


struct DiscreteTriplet
{
  DiscreteTriplet(const Eigen::Vector3f& tr, float dx, float dy, float dth)
  {
    ix = (int)(tr.x()/dx);
    iy = (int)(tr.y()/dy);
    ith = (int)(tr.z()/dth);
  }

  bool operator < (const DiscreteTriplet& dt) const
  {
    if (ix < dt.ix )
      return true;
    if (ix == dt.ix && iy < dt.iy) 
      return true;
    if (ix == dt.ix && iy == dt.iy && ith < dt.ith)
      return true;
    return false;
  }
  float ix, iy, ith;
};


class CorrelativeMatcherResult : public ScanMatcherResult
{
  friend class CorrelativeMatcher;
  public:
    virtual float matchingScore() const;
    virtual ~CorrelativeMatcherResult();
};


class CorrelativeMatcher : public ScanMatcher
{
  public:
    CorrelativeMatcher(const float& resolution, const float& radius, const int& kernelSize,
		       const float& kernelMaxValue, int kscale_= 128);
    
    CorrelativeMatcher(const CharGrid& g, const int& kernelSize,
		       const float& kernelMaxValue, int kscale_ = 128);
    
    
    virtual ~CorrelativeMatcher();
    
    
    //! search for the best in a region
    //! @param result: the returned transformation
    //! @param points: the input scan points in cartesian coordinates
    //! @param lowerLeftF: the lower left corner of the search region (z is the theta component in radiants)
    //! @param upperRightF: the upper right element of the search region (z is the theta component in radiants)
    //! @param thetaRes: the resolution of the search
    //! @param maxScore: the maximum score of the match
    //! @returns the score of the best transformation
    void scanMatch(const Vector2fVector& points, const Eigen::Vector3f& lowerLeftF,
		   const Eigen::Vector3f& upperRightF, const float& thetaRes, const float& maxScore);

    
  protected:
    void addToPrunedMap(std::map<DiscreteTriplet, CorrelativeMatcherResult*>& myMap,
			const CorrelativeMatcherResult* mr, const float& dx, const float& dy, const float& dth);
    
    
    //! search for the N best in a region
    //! @param mresvec: the returned vector of match results
    //! @param points: the input scan points in cartesian coordinates
    //! @param lowerLeftF: the lower left corner of the search region (z is the theta component in radiants)
    //! @param upperRightF: the upper right element of the search region (z is the theta component in radiants)
    //! @param thetaRes: the resolution of the search
    //! @param maxScore: the maximum score of the match
    //! @param dx:
    //! @param dy:
    //! @param dth:
    void scanMatch(std::vector<CorrelativeMatcherResult*>& mresvec,const Vector2fVector& points,
			   const Eigen::Vector3f& lowerLeftF, const Eigen::Vector3f& upperRightF, const float& thetaRes,
			   const float& maxScore, const float& dx, const float& dy, const float& dth);


    //! search for the N best in a set of regions region
    //! @param mresvec: the returned vector of match results
    //! @param points: the input scan points in cartesian coordinates
    //! @param regions: the regions where to do the search (vector)
    //! @param params: the parameters to be used for the matching
    void scanMatch(std::vector<CorrelativeMatcherResult*>& mresvec, const Vector2fVector& points,
			   const RegionVector& regions, const MatchingParameters& params);
    
    
    int _kscale;
    CorrelativeMatcherResult* cmr;
};
#endif