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
