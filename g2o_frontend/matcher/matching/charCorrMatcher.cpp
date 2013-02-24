#include "charCorrMatcher.h"


using namespace std;
using namespace Eigen;



float CorrelativeCharMatcherResult::matchingScore() const
{
    return _matchingScore;
}


CorrelativeCharMatcherResult::~CorrelativeCharMatcherResult() {}



CorrelativeCharMatcher::CorrelativeCharMatcher(const float& resolution, const float& radius,
                                               const int& kernelSize, const float& kernelMaxValue, int gridScale)
    : CharMatcher(resolution, radius, kernelSize, kernelMaxValue, gridScale)
{
    _innerResults.resize(5000);
}


CorrelativeCharMatcher::CorrelativeCharMatcher(const CharGrid& g, const int& kernelSize, const float& kernelMaxValue, int gridScale)
    : CharMatcher(g, kernelSize, kernelMaxValue, gridScale)
{
    cout << "b" << endl;
    _innerResults.resize(5000);
}


CorrelativeCharMatcher::~CorrelativeCharMatcher() {}


void CorrelativeCharMatcher::addToPrunedMap(map<DiscreteTriplet, CorrelativeCharMatcherResult*>& myMap,
                                            const CorrelativeCharMatcherResult* mr, const float& dx,
                                            const float& dy, const float& dth)
{
    DiscreteTriplet currentTriplet(mr->_transformation, dx, dy, dth);
    map<DiscreteTriplet, CorrelativeCharMatcherResult*>::iterator it = myMap.find(currentTriplet);
    if(it != myMap.end())
    {
        if(it->second->_matchingScore > mr->_matchingScore)
        {
            it->second->_transformation = mr->_transformation;
            it->second->_matchingScore = mr->_matchingScore;
        }
    }
    else
    {
        CorrelativeCharMatcherResult* cmr = new CorrelativeCharMatcherResult;
        cmr->_transformation = mr->_transformation;
        cmr->_matchingScore= mr->_matchingScore;
        myMap.insert(make_pair(currentTriplet, cmr));
    }
}


void CorrelativeCharMatcher::scanMatch(const Vector2fVector& points, const Vector3f& lowerLeftF,
                                       const Vector3f& upperRightF, const float& thetaRes, const float& maxScore)
{
    float dx = _scanGrid.resolution()*4;
    float dy = _scanGrid.resolution()*4;
    float dth = thetaRes *4;
    scanMatch(points, lowerLeftF, upperRightF, thetaRes, maxScore, dx, dy, dth);
    if(_innerResults.size())
    {
      CorrelativeCharMatcherResult* cmr = new CorrelativeCharMatcherResult;
      cmr->_transformation = _innerResults[0]->_transformation;
      cmr->_matchingScore = _innerResults[0]->_matchingScore;
      _matchResults.push_back(cmr);
    }
    else
    {
      CorrelativeCharMatcherResult* cmr = new CorrelativeCharMatcherResult;
      cmr->_transformation = Vector3f(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
      cmr->_matchingScore = numeric_limits<float>::max();
      _matchResults.push_back(cmr);
    }
}


void CorrelativeCharMatcher::scanMatch(const Vector2fVector& points, const RegionVector& regions,
                                       float thetaRes, float maxScore, float dx, float dy, float dth)
{
    MatchingParameters params;
    params.searchStep = Vector3f(_scanGrid.resolution(), _scanGrid.resolution(), thetaRes);
    params.maxScore = maxScore;
    params.resultsDiscretization = Vector3f(dx,dy,dth);
    scanMatch(points, regions, params);
    if(_innerResults.size())
    {
      CorrelativeCharMatcherResult* cmr = new CorrelativeCharMatcherResult;
      cmr->_transformation = _innerResults[0]->_transformation;
      cmr->_matchingScore = _innerResults[0]->_matchingScore;
      _matchResults.push_back(cmr);
    }
    else
    {
      CorrelativeCharMatcherResult* cmr = new CorrelativeCharMatcherResult;
      cmr->_transformation = Vector3f(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
      cmr->_matchingScore = numeric_limits<float>::max();
      _matchResults.push_back(cmr);
    }
}


void CorrelativeCharMatcher::scanMatch(const Vector2fVector& points, const Vector3f& lowerLeftF,
                                       const Vector3f& upperRightF, const float& thetaRes,
                                       const float& maxScore, const float& dx, const float& dy, const float& dth)
{
    RegionVector regions(1);
    regions[0].lowerLeft = lowerLeftF;
    regions[0].upperRight = upperRightF;

    MatchingParameters params;
    params.searchStep = Vector3f(_scanGrid.resolution(), _scanGrid.resolution(), thetaRes);
    params.maxScore = maxScore;
    params.resultsDiscretization = Vector3f(dx,dy,dth);
    scanMatch(points, regions, params);
    if(_innerResults.size())
    {
      CorrelativeCharMatcherResult* cmr = new CorrelativeCharMatcherResult;
      cmr->_transformation = _innerResults[0]->_transformation;
      cmr->_matchingScore = _innerResults[0]->_matchingScore;
      _matchResults.push_back(cmr);
    }
    else
    {
      CorrelativeCharMatcherResult* cmr = new CorrelativeCharMatcherResult;
      cmr->_transformation = Vector3f(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
      cmr->_matchingScore = numeric_limits<float>::max();
      _matchResults.push_back(cmr);
    }
}


void CorrelativeCharMatcher::scanMatch(const Vector2fVector& points, const RegionVector& regions,
                                       const MatchingParameters& params)
{
    std::map<DiscreteTriplet,CorrelativeCharMatcherResult*> resultMap;

    Vector2iVector intPoints(points.size());
    float thetaRes = params.searchStep.z();
    int xSteps = params.searchStep.x()/_scanGrid.resolution();
    int ySteps = params.searchStep.y()/_scanGrid.resolution();
    float maxScore = params.maxScore;

    if(xSteps <= 0)
    {
        xSteps = 1;
    }
    if(ySteps <= 0)
    {
        ySteps = 1;
    }

    int additions = 0;
    for(RegionVector::const_iterator rit=regions.begin(); rit!=regions.end(); ++rit)
    {
        Vector3f lowerLeftF = rit->lowerLeft;
        Vector3f upperRightF = rit->upperRight;
        Vector2i lowerLeft = _scanGrid.world2grid(Vector2f(lowerLeftF.x(),  lowerLeftF.y()));
        Vector2i upperRight = _scanGrid.world2grid(Vector2f(upperRightF.x(), upperRightF.y()));

        for(float t = lowerLeftF.z(); t < upperRightF.z(); t += thetaRes)
        {
            float c = cos(t), s = sin(t);
            Vector2i previousPoint(-10000,-10000);
            int k = 0;
            const Vector2f* _p = &(points[0]);
            Vector2i* _ip = &(intPoints[0]);
            for(size_t i = 0; i < points.size(); i++)
            {
                Vector2f p(c*_p->x()-s*_p->y(), s*_p->x()+c*_p->y());
                Vector2i ip(p.x()*_scanGrid.inverseResolution(), p.y()*_scanGrid.inverseResolution());
                if(ip.x() != previousPoint.x() || ip.y() != previousPoint.y())
                {
                    *_ip=ip;
                    _ip++;
                    k++;
                    previousPoint=ip;
                }
                _p++;
            }

            float ikscale = 1./(float)(_gridKScale);
            for(int i = lowerLeft.x(); i < upperRight.x(); i += xSteps)
            {
                for(int j = lowerLeft.y(); j < upperRight.y(); j += ySteps)
                {
                    int idsum=0;
                    Vector2i offset(i,j);
                    const Vector2i* _ip=&(intPoints[0]);
                    for (int ii=0; ii<k; ii++)
                    {
                        Vector2i ip=*_ip+offset;
                        _ip++;
                        idsum+=_convolvedGrid.cell(ip);
                    }

                    float dsum = (float)idsum * (float)ikscale;
                    dsum = k ? (dsum / (float) k) : maxScore+1;
                    Vector2f mp(_scanGrid.grid2world(Vector2i(i,j)));
                    Vector3f current(mp.x(), mp.y(), t);
                    if(dsum < maxScore)
                    {
                        CorrelativeCharMatcherResult* cmr = new CorrelativeCharMatcherResult;
                        cmr->_transformation = current;
                        cmr->_matchingScore = dsum;
                        additions++;
                        addToPrunedMap(resultMap, cmr, params.resultsDiscretization.x(),
                                       params.resultsDiscretization.y(), params.resultsDiscretization.z());
                    }
                }
            }
        }
    }

    CorrelativeCharMatcherResult* cmr = new CorrelativeCharMatcherResult;
    cmr->_transformation = Vector3f(.0, .0, .0);
    cmr->_matchingScore = 0;
    _innerResults.resize(resultMap.size(), cmr);

    unsigned int k = 0;
    for(map<DiscreteTriplet, CorrelativeCharMatcherResult*>::iterator it = resultMap.begin(); it != resultMap.end(); ++it)
    {
        _innerResults[k++] = it->second;
    }
  //   cerr << "bareResults= " << additions << "/"  << mresvec.size() << endl;

    Comparator comp;
    sort(_innerResults.begin(), _innerResults.end(), comp);
}
