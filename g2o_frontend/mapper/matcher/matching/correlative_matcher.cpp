#include "correlative_matcher.h"


using namespace g2o;
using namespace std;
using namespace Eigen;

namespace match_this
{

float CorrelativeMatcherResult::matchingScore() const
{
    return _matchingScore;
}


CorrelativeMatcherResult::~CorrelativeMatcherResult() {}


CorrelativeMatcher::CorrelativeMatcher(const float& resolution, const float& radius,
                                       const int& kernelSize, const float& kernelMaxValue, int gridScale)
    : ScanMatcher(resolution, radius, kernelSize, kernelMaxValue, gridScale)
{
    _regions.reserve(100);
}


CorrelativeMatcher::CorrelativeMatcher(const CharGrid& g, const int& kernelSize, const float& kernelMaxValue, int gridScale)
    : ScanMatcher(g, kernelSize, kernelMaxValue, gridScale)
{
    _regions.reserve(100);
}


CorrelativeMatcher::~CorrelativeMatcher()
{
    delete innerCMR;
}


void CorrelativeMatcher::addToPrunedMap(map<DiscreteTriplet, CorrelativeMatcherResult*>& myMap,
                                        const CorrelativeMatcherResult* mr, const float& dx,
                                        const float& dy, const float& dth)
{
    DiscreteTriplet currentTriplet(mr->_transformation, dx, dy, dth);
    map<DiscreteTriplet, CorrelativeMatcherResult*>::iterator it = myMap.find(currentTriplet);
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
        CorrelativeMatcherResult* cmr = new CorrelativeMatcherResult;
        cmr->_transformation = mr->_transformation;
        cmr->_matchingScore= mr->_matchingScore;
        myMap.insert(make_pair(currentTriplet, cmr));
    }
}


void CorrelativeMatcher::match(OptimizableGraph::Vertex* ref, OptimizableGraph::Vertex* curr, const float& maxScore)
{
    OptimizableGraph::Data* refData = ref->userData();
    const RobotLaser* prevLaser = dynamic_cast<const RobotLaser*>(refData);
    VertexSE2* refSE2 = dynamic_cast<VertexSE2*>(ref);

    OptimizableGraph::Data* currData = curr->userData();
    const RobotLaser* currLaser = dynamic_cast<const RobotLaser*>(currData);
    VertexSE2* currSE2 = dynamic_cast<VertexSE2*>(curr);
    if(prevLaser && refSE2 && currLaser && currSE2)
    {
        Isometry2d delta = refSE2->estimate().toIsometry().inverse() * currSE2->estimate().toIsometry();
        Matrix2f mat = delta.rotation().cast<float>();
        float angle = atan2(mat(1, 0), mat(0, 0));
        Vector3f initGuess(delta.translation().x(), delta.translation().y(), angle);
        Vector3f lower(-0.3+initGuess.x(), -0.3+initGuess.y(), -0.2+initGuess.z());
        Vector3f upper(0.3+initGuess.x(), 0.3+initGuess.y(), 0.2+initGuess.z());
        float thetaRes = 0.01;

        Vector2fVector prevScan = point2vector(prevLaser->cartesian());
        this->convolveScan(prevScan);
        Vector2fVector currScan = point2vector(currLaser->cartesian());

        this->scanMatch(currScan, lower, upper, thetaRes, maxScore);
    }
    else
    {
        cout << "SM: some data is missing" << endl;
    };
}


void CorrelativeMatcher::scanMatch(const Vector2fVector& points, const Vector3f& lowerLeftF,
                                   const Vector3f& upperRightF, const float& thetaRes, const float& maxScore)
{
    vector<CorrelativeMatcherResult*> mresvec;
    float dx = _scanGrid.resolution()*4;
    float dy = _scanGrid.resolution()*4;
    float dth = thetaRes *4;
    scanMatch(mresvec, points, lowerLeftF, upperRightF, thetaRes, maxScore, dx, dy, dth);
    if(mresvec.size())
    {
        CorrelativeMatcherResult* cmr = new CorrelativeMatcherResult;
        cmr->_transformation = mresvec[0]->_transformation;
        cmr->_matchingScore = mresvec[0]->_matchingScore;
        _matchResults.push_back(cmr);
    }

    else
    {
        CorrelativeMatcherResult* cmr = new CorrelativeMatcherResult;
        cmr->_transformation = Vector3f(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
        cmr->_matchingScore = numeric_limits<float>::max();
        _matchResults.push_back(cmr);
    }
}


void CorrelativeMatcher::scanMatch(vector<CorrelativeMatcherResult*>& mresvec, const Vector2fVector& points, const RegionVector& regions,
                                   float thetaRes, float maxScore, float dx, float dy, float dth)
{
    MatchingParameters params;
    params.searchStep = Vector3f(_scanGrid.resolution(), _scanGrid.resolution(), thetaRes);
    params.maxScore = maxScore;
    params.resultsDiscretization = Vector3f(dx, dy, dth);

    _regions.clear();
    _regions = regions;

    scanMatch(mresvec, points, /*regions, */params);

    CorrelativeMatcherResult* cmr = new CorrelativeMatcherResult;
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


void CorrelativeMatcher::scanMatch(vector<CorrelativeMatcherResult*>& mresvec, const Vector2fVector& points,
                                   const Vector3f& lowerLeftF, const Vector3f& upperRightF, const float& thetaRes,
                                   const float& maxScore, const float& dx, const float& dy, const float& dth)
{
    Region r;
    r.lowerLeft = lowerLeftF;
    r.upperRight = upperRightF;

    _regions.clear();
    _regions.push_back(r);

    MatchingParameters params;
    params.searchStep = Vector3f(_scanGrid.resolution(), _scanGrid.resolution(), thetaRes);
    params.maxScore = maxScore;
    params.resultsDiscretization = Vector3f(dx,dy,dth);
    return scanMatch(mresvec, points, /*regions, */params);
}


void CorrelativeMatcher::scanMatch(vector<CorrelativeMatcherResult*>& mresvec, const Vector2fVector& points,
                                   /*const RegionVector& regions, */const MatchingParameters& params)
{
    map<DiscreteTriplet,CorrelativeMatcherResult*> resultMap;

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
    for(RegionVector::const_iterator rit = _regions.begin(); rit != _regions.end(); ++rit)
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
                        idsum += _convolvedGrid.cell(ip);
                    }
                    float dsum = (float)idsum * (float)ikscale;
                    dsum = k ? (dsum / (float) k) : maxScore+1;
                    if(dsum < maxScore)
                    {
                        Vector2f mp(_convolvedGrid.grid2world(Vector2i(i,j)));
                        Vector3f current(mp.x(), mp.y(), t);
                        CorrelativeMatcherResult* cmr = new CorrelativeMatcherResult;
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
    CorrelativeMatcherResult* cmr = new CorrelativeMatcherResult;
    cmr->_transformation = Vector3f(.0, .0, .0);
    cmr->_matchingScore = 0;
    mresvec.resize(resultMap.size(), cmr);

    delete cmr;

    unsigned int k = 0;
    for(map<DiscreteTriplet, CorrelativeMatcherResult*>::iterator it = resultMap.begin(); it != resultMap.end(); ++it)
    {
        mresvec[k++] = it->second;
    }
    //   cerr << "bareResults= " << additions << "/"  << mresvec.size() << endl;

    Comparator comp;
    sort(mresvec.begin(), mresvec.end(), comp);
}
}
