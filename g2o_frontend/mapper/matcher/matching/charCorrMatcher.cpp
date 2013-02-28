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

}


CorrelativeCharMatcher::CorrelativeCharMatcher(const CharGrid& g, const int& kernelSize, const float& kernelMaxValue, int gridScale)
    : CharMatcher(g, kernelSize, kernelMaxValue, gridScale)
{

}


CorrelativeCharMatcher::~CorrelativeCharMatcher()
{
    delete innerCMR;
}


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
    vector<CorrelativeCharMatcherResult*> mresvec;
    float dx = _scanGrid.resolution()*4;
    float dy = _scanGrid.resolution()*4;
    float dth = thetaRes *4;
    scanMatch(mresvec, points, lowerLeftF, upperRightF, thetaRes, maxScore, dx, dy, dth);
    if(mresvec.size())
    {
      CorrelativeCharMatcherResult* cmr = new CorrelativeCharMatcherResult;
      cmr->_transformation = mresvec[0]->_transformation;
      cmr->_matchingScore = mresvec[0]->_matchingScore;
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


void CorrelativeCharMatcher::scanMatch(vector<CorrelativeCharMatcherResult*>& mresvec, const Vector2fVector& points, const RegionVector& regions,
                                       float thetaRes, float maxScore, float dx, float dy, float dth)
{
    MatchingParameters params;
    params.searchStep = Vector3f(_scanGrid.resolution(), _scanGrid.resolution(), thetaRes);
    params.maxScore = maxScore;
    params.resultsDiscretization = Vector3f(dx, dy, dth);
    scanMatch(mresvec, points, regions, params);

    CorrelativeCharMatcherResult* cmr = new CorrelativeCharMatcherResult;
    if(mresvec.size())
    {
        cmr->_transformation = mresvec[0]->_transformation;
        cmr->_matchingScore = mresvec[0]->_matchingScore;
        _matchResults.push_back(cmr);
    }
    else
    {
        cmr->_transformation = Vector3f(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
        cmr->_matchingScore = numeric_limits<float>::max();
        _matchResults.push_back(cmr);
    }
}


void CorrelativeCharMatcher::scanMatch(vector<CorrelativeCharMatcherResult*>& mresvec, const Vector2fVector& points,
                                       const Vector3f& lowerLeftF, const Vector3f& upperRightF, const float& thetaRes,
                                       const float& maxScore, const float& dx, const float& dy, const float& dth)
{
    RegionVector regions(1);
    regions[0].lowerLeft = lowerLeftF;
    regions[0].upperRight = upperRightF;

    MatchingParameters params;
    params.searchStep = Vector3f(_scanGrid.resolution(), _scanGrid.resolution(), thetaRes);
    params.maxScore = maxScore;
    params.resultsDiscretization = Vector3f(dx,dy,dth);
    return scanMatch(mresvec, points, regions, params);
}


void CorrelativeCharMatcher::scanMatch(vector<CorrelativeCharMatcherResult*>& mresvec, const Vector2fVector& points,
                                       const RegionVector& regions, const MatchingParameters& params)
{
    map<DiscreteTriplet,CorrelativeCharMatcherResult*> resultMap;        

    Vector2iVector intPoints(points.size());
    float thetaRes = params.searchStep.z();
    int xSteps = params.searchStep.x()/_convolvedGrid.resolution();
    int ySteps = params.searchStep.y()/_convolvedGrid.resolution();
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

    int scanSizeX =  _scanGrid.size().x();
    int scanSizeY =  _scanGrid.size().y();
    for(RegionVector::const_iterator rit=regions.begin(); rit!=regions.end(); ++rit)
    {
        Vector3f lowerLeftF = rit->lowerLeft;
        Vector3f upperRightF = rit->upperRight;
        Vector2i lowerLeft = _convolvedGrid.world2grid(Vector2f(lowerLeftF.x(),  lowerLeftF.y()));
        Vector2i upperRight = _convolvedGrid.world2grid(Vector2f(upperRightF.x(), upperRightF.y()));
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
                Vector2i ip(p.x()*_convolvedGrid.inverseResolution(), p.y()*_convolvedGrid.inverseResolution());
                if(ip.x() != previousPoint.x() || ip.y() != previousPoint.y())
                {
                    *_ip = ip;
                    _ip++;
                    k++;
                    previousPoint = ip;
                }
                _p++;
            }

            float ikscale = 1./(float)(_gridKScale);
            for(int i = lowerLeft.x(); i < upperRight.x(); i += xSteps)
            {
                for(int j = lowerLeft.y(); j < upperRight.y(); j += ySteps)
                {
                    int idsum = 0;
                    Vector2i offset(i,j);
                    const Vector2i* _ip = &(intPoints[0]);
                    for(int ii=0; ii < k; ++ii)
                    {
                        Vector2i ip = *_ip+offset;
                        _ip++;
                        int ipX = ip.x();
                        int ipY = ip.y();
//                        if(ipX >= 0 && ipX < scanSizeX && ipY >= 0 && ipY < scanSizeY)
//                        if(ipX >= 0 && ipY >= 0)
                        {
                            idsum += _convolvedGrid.cell(ip);
                        }
                    }
                    float dsum = (float)idsum * (float)ikscale;
                    dsum = k ? (dsum / (float) k) : maxScore+1;
                    if(dsum < maxScore)
                    {
                        Vector2f mp(_convolvedGrid.grid2world(Vector2i(i,j)));
                        Vector3f current(mp.x(), mp.y(), t);
                        CorrelativeCharMatcherResult* cmr = new CorrelativeCharMatcherResult;
                        cmr->_transformation = current;
                        cmr->_matchingScore = dsum;
                        additions++;
                        addToPrunedMap(resultMap, cmr, params.resultsDiscretization.x(),
                                       params.resultsDiscretization.y(), params.resultsDiscretization.z());
                        delete cmr;
                    }
                }
            }
        }
    }
    CorrelativeCharMatcherResult* cmr = new CorrelativeCharMatcherResult;
    cmr->_transformation = Vector3f(.0, .0, .0);
    cmr->_matchingScore = 0;
    mresvec.resize(resultMap.size(), cmr);

    delete cmr;

    unsigned int k = 0;
    for(map<DiscreteTriplet, CorrelativeCharMatcherResult*>::iterator it = resultMap.begin(); it != resultMap.end(); ++it)
    {
      mresvec[k++] = it->second;
    }
  //   cerr << "bareResults= " << additions << "/"  << mresvec.size() << endl;

    Comparator comp;
    sort(mresvec.begin(), mresvec.end(), comp);
}
