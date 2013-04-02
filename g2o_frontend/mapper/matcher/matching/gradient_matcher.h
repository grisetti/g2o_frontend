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
    

    //! function inherited from the super-classes Matcher, compatible with g2o
    //! represention of the graph
    //! @param ref: pointer to the vertex whose data is to use as reference
    //! @param curr: pointer to the vertex whose data is to be matched against the reference
    void match(g2o::OptimizableGraph::Vertex* ref, g2o::OptimizableGraph::Vertex* curr);


    void scanMatch(const Vector2fVector& p, const Eigen::Vector3f& ig);

protected:
    int partialScore(const Vector2fVector& scan, const Eigen::Vector3f& t);
    
    Eigen::Vector3f _moves[6];
    GradientMatcherResult* gmr;
};

#endif // GRADIENTMATCHER_H
