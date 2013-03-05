#ifndef __MATCHER__H_
#define __MATCHER__H_

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <list>
#include <vector>
#include "g2o/core/hyper_graph.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"



class MatcherResult
{    
public:
    friend class Matcher;

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
    typedef std::set<g2o::HyperGraph::Vertex*> VertexContainer;
    typedef std::vector<MatcherResult*> ResultContainer;

    virtual ~Matcher();
    
    virtual void clear();
    
    // Compute the matching
    void compute();
    

    // Add vertex to current map (the one to be matched wrt the reference map)
    inline bool addToCurrent(g2o::HyperGraph::Vertex* lv)
    {
        if(lv)
        {
            _currentVerteces.insert(lv);
            return true;
        }
        else
            return false;
    }


    // Add vertex to current map (the one to be matched wrt the reference map)
    inline bool addToCurrent(VertexContainer lv)
    {
        for(VertexContainer::const_iterator it = lv.begin(); it != lv.end(); ++it)
        {
            if(*it)
            {
                _currentVerteces.insert(*it);
            }
            else
            {
                return false;
            }
        }
        return true;
    }


    // Add vertex to reference map
    inline bool addToReference(g2o::HyperGraph::Vertex* lv)
    {
        if(lv)
        {
            _referenceVerteces.insert(lv);
            return true;
        }
        else
            return false;
    }


    // Add vertex to reference map
    inline bool addToReference(VertexContainer lv)
    {
        for(VertexContainer::const_iterator it = lv.begin(); it != lv.end(); ++it)
        {
            if(*it)
            {
                _referenceVerteces.insert(*it);
            }
            else
            {
                return false;
            }
        }
        return true;
    }


    // Set currentGauge to the given vertex
    inline bool setCurrentGauge(g2o::HyperGraph::Vertex* v)
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
    inline bool setReferenceGauge(g2o::HyperGraph::Vertex* v)
    {
        if(v)
        {
            _referenceGauge = v;
            return true;
        }
        else
            return false;
    }

    virtual void clearMatchResults();

    virtual void match(g2o::HyperGraph::Vertex* ref, g2o::HyperGraph::Vertex* curr);
    
    inline double getMilliSecs() const { return g2o::get_time(); }

    inline ResultContainer& getMatches() { return _matchResults; }
    inline const ResultContainer& getMatches() const { return _matchResults; }

    inline VertexContainer currentVerteces() { return _currentVerteces; }
    inline const VertexContainer currentVerteces() const { return _currentVerteces; }

    inline VertexContainer referenceVerteces() { return _referenceVerteces; }
    inline const VertexContainer referenceVerteces() const { return _referenceVerteces; }

    inline g2o::HyperGraph::Vertex* currentGauge() { return _currentGauge; }
    inline const g2o::HyperGraph::Vertex* currentGauge() const { return _currentGauge; }

    inline g2o::HyperGraph::Vertex* referenceGauge() { return _referenceGauge; }
    inline const g2o::HyperGraph::Vertex* referenceGauge() const { return _referenceGauge; }

protected:
    
    g2o::HyperGraph::Vertex* _currentGauge;
    g2o::HyperGraph::Vertex* _referenceGauge;
    VertexContainer _currentVerteces;
    VertexContainer _referenceVerteces;
    ResultContainer _matchResults;
};
#endif
