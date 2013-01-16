#ifndef __MATCHER__H_
#define __MATCHER__H_

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <list>
#include <vector>
#include "g2o/core/optimizable_graph.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"



class MatcherResult
{
  friend class Matcher;
  friend class Comparator;
  public:
    virtual const float matchingScore() const;
    virtual ~MatcherResult();
    
    Eigen::Vector3f _transformation;
    Eigen::Matrix3f _covariance;
    
  protected:
    float _matchingScore;
};


struct Comparator
{
  bool operator()(const MatcherResult* mr1, const MatcherResult* mr2)
  {
    return mr1->matchingScore() < mr2->matchingScore();
  } 
};


class Matcher
{ 
  public:
    typedef std::vector<g2o::OptimizableGraph::Vertex*> VertexContainer;
    typedef std::vector<MatcherResult*> ResultContainer;
    
    // D-tor
    virtual ~Matcher();
    
    virtual void clear();
    
    // Compute the matching
    void compute();
    
    // Add verteces to reference and current local maps
    bool addToReference(g2o::OptimizableGraph::Vertex* lv);
    bool addToCurrent(g2o::OptimizableGraph::Vertex* lv);
    
    // Choose reference verteces
    bool setReferenceGauge(g2o::OptimizableGraph::Vertex* v);
    bool setCurrentGauge(g2o::OptimizableGraph::Vertex* v);    
    
    inline double getMilliSecs() const {return g2o::get_time();}
    inline const ResultContainer& getMatches() const {return _matchResults;}
    inline VertexContainer currentVerteces(VertexContainer& cv) {return _currentVerteces;}
    inline VertexContainer referenceVerteces(VertexContainer& rv) {return _referenceVerteces;}
    inline const g2o::OptimizableGraph::Vertex* getCurrentGauge() const {return _currentGauge;}
    inline const g2o::OptimizableGraph::Vertex* getReferenceGauge() const {return _referenceGauge;}
    inline g2o::OptimizableGraph::Vertex* getCurrentGauge() {return _currentGauge;}
    inline g2o::OptimizableGraph::Vertex* getReferenceGauge() {return _referenceGauge;}

    
  protected:
    virtual void clearMatchResults();
    
    g2o::OptimizableGraph::Vertex* _currentGauge;	
    g2o::OptimizableGraph::Vertex* _referenceGauge;
    VertexContainer _currentVerteces;
    VertexContainer _referenceVerteces;
    ResultContainer _matchResults;
};
#endif