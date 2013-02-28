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
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual float matchingScore() const;
    virtual ~MatcherResult();

    Eigen::Vector3f _transformation;
    Eigen::Matrix3f _covariance;

protected:
    float _matchingScore;
};


struct Comparator
{
    inline bool operator()(const MatcherResult* mr1, const MatcherResult* mr2)
    {
        return mr1->matchingScore() < mr2->matchingScore();
    }
};


class Matcher
{

public:
    typedef std::vector<g2o::OptimizableGraph::Vertex*> VertexContainer;
    typedef std::vector<MatcherResult*> ResultContainer;

    virtual ~Matcher();
    
    virtual void clear();
    
    // Compute the matching
    void compute();
    

    // Add vertex to current map (the one to be matched wrt the reference map)
    inline bool addToCurrent(g2o::OptimizableGraph::Vertex* lv)
    {
        if(lv)
        {
            _currentVerteces.push_back(lv);
            return true;
        }
        else
            return false;
    }


    // Add vertex to reference map
    inline bool addToReference(g2o::OptimizableGraph::Vertex* lv)
    {
        if(lv)
        {
            _referenceVerteces.push_back(lv);
            return true;
        }
        else
            return false;
    }


    // Set currentGauge to the given vertex
    inline bool setCurrentGauge(g2o::OptimizableGraph::Vertex* v)
    {
        if(v)
        {
            _currentGauge = v;
            return true;
        }
        else
            return false;
    }


    // Set referenceGauge to the given vertex
    inline bool setReferenceGauge(g2o::OptimizableGraph::Vertex* v)
    {
        if(v)
        {
            _referenceGauge = v;
            return true;
        }
        else
            return false;
    }

    
    inline double getMilliSecs() const { return g2o::get_time(); }

    inline ResultContainer& getMatches() { return _matchResults; }
    inline const ResultContainer& getMatches() const { return _matchResults; }

    inline VertexContainer currentVerteces() { return _currentVerteces; }
    inline const VertexContainer currentVerteces() const { return _currentVerteces; }

    inline VertexContainer referenceVerteces() { return _referenceVerteces; }
    inline const VertexContainer referenceVerteces() const { return _referenceVerteces; }

    inline g2o::OptimizableGraph::Vertex* getCurrentGauge() { return _currentGauge; }
    inline const g2o::OptimizableGraph::Vertex* getCurrentGauge() const { return _currentGauge; }

    inline g2o::OptimizableGraph::Vertex* getReferenceGauge() { return _referenceGauge; }
    inline const g2o::OptimizableGraph::Vertex* getReferenceGauge() const { return _referenceGauge; }


protected:
    virtual void clearMatchResults();
    
    g2o::OptimizableGraph::Vertex* _currentGauge;	
    g2o::OptimizableGraph::Vertex* _referenceGauge;
    VertexContainer _currentVerteces;
    VertexContainer _referenceVerteces;
    ResultContainer _matchResults;
};
#endif
