#include <fstream>
#include <iostream>
#include "g2o_frontend/matcher/matching/hierarchicalMatcher.h"
#include "g2o_frontend/matcher/utils/logReader.h"

using namespace std;
using namespace Eigen;



int main()
{
  float resolution = 0.03;
  float kernelRange = 0.5;
  int hV = 10;
  
  float radius = 50;
  HierarchicalMatcher hm(resolution, radius, kernelRange/resolution, kernelRange, 128);
  const string logFile = "/home/erratic/src/matcher/logFile/dis.clf";
  LogReader lr(logFile, hV);

  vector<Vector2fVector> logScan = lr.getScans();
  vector<Vector3f> logPose = lr.getPoses();
  vector<Vector3f> initialGuesses;
  initialGuesses.resize(logScan.size());

  Vector2fVector previousScan, previousReducedScan;
  Vector3f pOdom, cOdom;
  
  previousScan = logScan[0];
  pOdom = logPose[0];
  
  hm.subsample(previousReducedScan, previousScan);
  
  double totalStart = hm.getMilliSecs();
  for(size_t it = 1; it < logScan.size(); ++it)
  {
    Isometry2f t1;
    t1 = Rotation2Df(pOdom.z());
    t1.translation() = Vector2f(pOdom.x(), pOdom.y());

    Vector2fVector currentScan = logScan[it];
    cOdom = logPose[it];
    Isometry2f t2;
    t2 = Rotation2Df(cOdom.z());
    t2.translation() = Vector2f(cOdom.x(), cOdom.y());

    Isometry2f delta = t1.inverse()*t2;
    MatrixXf mat = delta.rotation();
    float angle = atan2(mat(1, 0), mat(0, 0));
    Vector3f initGuess(delta.translation().x(), delta.translation().y(), angle);
//     Vector3f lower(-10.+initGuess.x(), -10.+initGuess.y(), -M_PI);
//     Vector3f upper(+10.+initGuess.x(),  10.+initGuess.y(),  M_PI);
    Vector3f lower(-0.3+initGuess.x(), -0.3+initGuess.y(), -0.3+initGuess.z());
    Vector3f upper(0.3+initGuess.x(), 0.3+initGuess.y(), 0.3+initGuess.z());
    float thetaRes = 0.01; // was 0.025
    float max = 250;

    hm.convolveScan(previousReducedScan);
    
    Vector2fVector reducedScan;
    hm.subsample(reducedScan, currentScan);

    double start = hm.getMilliSecs();
    vector<CorrelativeMatcherResult*> mresvec;
    hm.scanMatch(mresvec, reducedScan, lower, upper, thetaRes, max, 0.5, 0.5, 0.2, 3);
    double end = hm.getMilliSecs();
    hm.clear();    
    pOdom = cOdom;
//     previousScan = currentScan;
    
    previousReducedScan = reducedScan;
    initialGuesses.push_back(initGuess);
    cout << (end - start) * 1000 << " ms" << endl;
  }
  double totalEnd = hm.getMilliSecs();
  cout << "Total time is: " << (totalEnd - totalStart)* 1000 << " ms " << endl;
  
  hm.clear();
  cout << "Got matches" << endl;
  vector<MatcherResult*> matches = hm.getMatches();

  Vector2fVector scan = logScan[0];
  Isometry2f adjust = Isometry2f::Identity();
  
//   float mapRadius = 50; //lower dimension for fakeSimulated.clf
  float mapRadius = 100; //higher dimension for dis.clf
  HierarchicalMatcher hm1(resolution, mapRadius, kernelRange/resolution, kernelRange, 128);
  hm1.integrateScan(scan, 1., adjust);
  for(size_t it = 1; it < logScan.size(); ++it)
  {
    scan = logScan[it];
    Vector3f current = matches[it-1]->_transformation;
    if(current.x() == numeric_limits<float>::max())
    {
      current = initialGuesses[it-1];
    }
    Isometry2f innerAdjust;
    innerAdjust = Rotation2Df(current.z());
    innerAdjust.translation() = Vector2f(current.x(), current.y());
    adjust = adjust * innerAdjust;
    hm1.integrateScan(scan, 1., adjust);
  }
  
  ofstream a("hierarchicalMap.ppm");
  hm1.saveScanAsPPM(a, true);
  a.close();
  
  exit(1);
}
