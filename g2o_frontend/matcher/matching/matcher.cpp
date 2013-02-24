#include "matcher.h"


float MatcherResult::matchingScore() const
{
  return _matchingScore;
}


MatcherResult::~MatcherResult() {}


Matcher::~Matcher()
{
  clear();
  clearMatchResults();
}


void Matcher::clear() {}


void Matcher::clearMatchResults()
{
  for(ResultContainer::iterator it = _matchResults.begin(); it != _matchResults.end(); ++it)
  {
    delete *it;
  }
  _matchResults.clear();
}


//bool Matcher::addToCurrent(g2o::OptimizableGraph::Vertex* lv)
//{
//  _currentVerteces.push_back(lv);
//  return true;
//}


//bool Matcher::addToReference(g2o::OptimizableGraph::Vertex* lv)
//{
//  _referenceVerteces.push_back(lv);
//  return true;
//}


//bool Matcher::setCurrentGauge(g2o::OptimizableGraph::Vertex* v)
//{
//  _currentGauge = v;
//  return true;
//}


//bool Matcher::setReferenceGauge(g2o::OptimizableGraph::Vertex* v)
//{
//  _referenceGauge = v;
//  return true;
//}
