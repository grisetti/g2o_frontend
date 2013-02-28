#include "gradientMatcher.h"


using namespace std;
using namespace Eigen;


float GradientMatcherResult::matchingScore() const
{
  return _matchingScore;
}


GradientMatcherResult::~GradientMatcherResult() {}


GradientMatcher::GradientMatcher(const float& resolution, const float& radius, const int& kernelSize,
				 const float& kernelMaxValue, const Vector3f& baseMove)
: ScanMatcher(resolution, radius, kernelSize, kernelMaxValue)
{
  _moves[0] = Vector3f(+baseMove.x(), 0, 0);
  _moves[1] = Vector3f(-baseMove.x(), 0, 0);
  _moves[2] = Vector3f(0, +baseMove.y(), 0);
  _moves[3] = Vector3f(0, -baseMove.y(), 0);
  _moves[4] = Vector3f(0, 0, +baseMove.z());
  _moves[5] = Vector3f(0, 0, -baseMove.z());
}


GradientMatcher::GradientMatcher(const CharGrid& g, const int& kernelSize,
				 const float& kernelMaxValue, const Vector3f& baseMove)
: ScanMatcher(g, kernelSize, kernelMaxValue)
{
  _moves[0] = Vector3f(+baseMove.x(), 0, 0);
  _moves[1] = Vector3f(-baseMove.x(), 0, 0);
  _moves[2] = Vector3f(0, +baseMove.y(), 0);
  _moves[3] = Vector3f(0, -baseMove.y(), 0);
  _moves[4] = Vector3f(0, 0, +baseMove.z());
  _moves[5] = Vector3f(0, 0, -baseMove.z());
}


GradientMatcher::~GradientMatcher()
{
  delete gmr;
}


int GradientMatcher::partialScore(const Vector2fVector& scan, const Vector3f& t)
{
  float s = sin(t.z());
  float c = cos(t.z());
  Vector2i previousPoint(-10000, -10000);
  int innerScore = 0;
  for(unsigned int beamNo = 0; beamNo < scan.size(); ++beamNo)
  {
    const Vector2f& endPoint = scan[beamNo];
    float endPointX = endPoint.x();
    float endPointY = endPoint.y();
    Vector2f p(c * endPointX - s * endPointY + t.x(), s * endPointX + c * endPointY + t.y());
    Vector2i intPoint = _convolvedGrid.world2grid(p);
    float intPointX = intPoint.x();
    float intPointY = intPoint.y();
    if((intPointX >= 0) && (intPointY >= 0) &&
       (intPointX < _convolvedGrid.size().x()) && (intPointY < _convolvedGrid.size().y()) &&
       (intPointX != previousPoint.x() || intPointY != previousPoint.y()))
    {
      innerScore += _convolvedGrid.cell(intPoint);
    }
    previousPoint = intPoint;
  }
  return innerScore;  
}


void GradientMatcher::scanMatch(const Vector2fVector& scan, const Vector3f& initialGuess)
{
  Vector2f tempCenter = _scanGrid.center();
  _scanGrid.setCenter(-initialGuess.x(), -initialGuess.y());

  Vector3f bestMove(initialGuess.x(), initialGuess.y(),initialGuess.z());
  float bestScore = partialScore(scan, bestMove);
  int iteration = 0;
  int maxIterations = 5;
  float scale = 1.;

  Vector3f bestMoveAtResolution= bestMove;
  float bestScoreAtResolution = bestScore;
  
  for(int i = 0; i < maxIterations; ++i)
  {
    Vector3f bestLocalMove = bestMoveAtResolution;
    float bestLocalMoveScore = bestScoreAtResolution;
    bool changed = false;
    do {
      Vector3f bestLocalTempMove = bestLocalMove;
      float bestLocalTempScore = bestLocalMoveScore;
      for(unsigned int m = 0; m < 6; ++m)
      {
	Vector3f move = scale * _moves[m] + bestLocalMove;
	float innerScore = partialScore(scan, move);
	if(innerScore < bestLocalMoveScore)
	{
	  bestLocalTempScore = innerScore;
	  bestLocalTempMove = move;
	}
      }
      if(bestLocalTempScore < bestLocalMoveScore)
      {
	bestLocalMove = bestLocalTempMove;
	bestLocalMoveScore = bestLocalTempScore;
	changed = true;
      }
      else
	changed = false;
    } while(changed);
    bestScoreAtResolution = bestLocalMoveScore;
    bestMoveAtResolution = bestLocalMove;
    scale = scale * .5;
  }
  gmr = new GradientMatcherResult;
  gmr->_transformation = bestMoveAtResolution;
  gmr->_matchingScore = bestScoreAtResolution;
  _matchResults.push_back(gmr);
  
  _scanGrid.setCenter(tempCenter);
}