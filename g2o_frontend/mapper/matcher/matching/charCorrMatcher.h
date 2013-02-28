#ifndef __CHAR_CORRELATIVE_MATCHER_H__
#define __CHAR_CORRELATIVE_MATCHER_H__

#include "charMatcher.h"


struct Region
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3f lowerLeft;
    Eigen::Vector3f upperRight;
};
typedef std::vector<Region, Eigen::aligned_allocator<Region> > RegionVector;


struct MatchingParameters
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3f searchStep;
    Eigen::Vector3f resultsDiscretization;
    float maxScore;
};
typedef std::vector<MatchingParameters, Eigen::aligned_allocator<MatchingParameters> > MatchingParametersVector;


struct DiscreteTriplet
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DiscreteTriplet(const Eigen::Vector3f& tr, float dx, float dy, float dth)
    {
        ix = (int)(tr.x()/dx);
        iy = (int)(tr.y()/dy);
        ith = (int)(tr.z()/dth);
    }

    bool operator<(const DiscreteTriplet& dt) const
    {
        if(ix < dt.ix)
        {
            return true;
        }
        if(ix == dt.ix && iy < dt.iy)
        {
            return true;
        }
        if(ix == dt.ix && iy == dt.iy && ith < dt.ith)
        {
            return true;
        }
        return false;
    }

    float ix, iy, ith;
};


class CorrelativeCharMatcherResult : public CharMatcherResult
{
    friend class CorrelativeCharMatcher;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual float matchingScore() const;
    virtual ~CorrelativeCharMatcherResult();
};


class CorrelativeCharMatcher : public CharMatcher
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    CorrelativeCharMatcher(const float& resolution, const float& radius, const int& kernelSize,
                           const float& kernelMaxValue, int _gridKScale = 128);


    CorrelativeCharMatcher(const CharGrid& g, const int& kernelSize, const float& kernelMaxValue, int _gridKScale = 128);
    

    virtual ~CorrelativeCharMatcher();

    //! search for the best in a region
    //! @param points: input scan points in cartesian coordinates
    //! @param lowerLeftF: lower left corner of the search region (z is the theta component in radians)
    //! @param upperRightF: upper right element of the search region (z is the theta component in radians)
    //! @param thetaRes: angular resolution of the search
    //! @param maxScore: maximum score of the match
    void scanMatch(const Vector2fVector& points, const Eigen::Vector3f& lowerLeftF, const Eigen::Vector3f& upperRightF,
                   const float& thetaRes, const float& maxScore);


    //! @param points: the input scan points in cartesian coordinates
    //! @param regions: the regions where to do the search (vector)
    //! @param thetaRes: the resolution of the search
    //! @param maxScore: the maximum score of the match
    void scanMatch(std::vector<CorrelativeCharMatcherResult*>& mresvec, const Vector2fVector& points,
                   const RegionVector& regions, float thetaRes, float maxScore,
                   float dx, float dy, float dth);


    void scanMatch(std::vector<CorrelativeCharMatcherResult*>& mresvec, const Vector2fVector& points,
                   const Eigen::Vector3f& lowerLeftF, const Eigen::Vector3f& upperRightF,
                   const float& thetaRes, const float& maxScore, const float& dx, const float& dy, const float& dth);
    

    //! search for the best in a set of regions
    //! @param points: input scan points in cartesian coordinates
    //! @param regions: regions in which to search for the matching
    //! @param params: matching parameters
    void scanMatch(std::vector<CorrelativeCharMatcherResult*>& mresvec, const Vector2fVector& points,
                   const RegionVector& regions, const MatchingParameters& params);
    

protected:
    void addToPrunedMap(std::map<DiscreteTriplet, CorrelativeCharMatcherResult*>& myMap,
			const CorrelativeCharMatcherResult* mr, const float& dx, const float& dy, const float& dth);

    CorrelativeCharMatcherResult* innerCMR;
};
#endif
