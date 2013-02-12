#include <fstream>
#include <iostream>
#include <time.h>

#include "g2o_frontend/matcher/matching/charCorrMatcher.h"
#include "g2o_frontend/matcher/matching/charGradMatcher.h"
#include "g2o_frontend/matcher/utils/logReader.h"


using namespace std;
using namespace Eigen;



int main()
{
  float resolution = 0.03;
  float kernelMaxValue = 3;
  int hV = 30;
  
  float radius = 50;
  CorrelativeCharMatcher highResCm(resolution, radius, kernelMaxValue, kernelMaxValue);
  CorrelativeCharMatcher middleResCm(2*resolution, radius, kernelMaxValue, kernelMaxValue);
  CorrelativeCharMatcher lowResCm(4*resolution, radius, kernelMaxValue, kernelMaxValue);
  
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

  double totalStart = highResCm.getMilliSecs();
  int i = 0;
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
    Vector3f lower((-0.3*4)+initGuess.x(), (-0.3*4)+initGuess.y(), (-0.2*4)+initGuess.z());
    Vector3f upper((0.3*4)+initGuess.x(), (0.3*4)+initGuess.y(), (0.2*4)+initGuess.z());
    float thetaRes = 0.04; //was 0.025
    float max = 350;
    
    double innerStart = highResCm.getMilliSecs();
    Vector2fVector currentReducedScan;
    lowResCm.subsample(previousReducedScan, previousScan);
    lowResCm.subsample(currentReducedScan, currentScan);
    lowResCm.convolveScan(previousReducedScan);
    lowResCm.scanMatch(currentReducedScan, lower, upper, thetaRes, max);
    
    initGuess = lowResCm.getMatches()[it-1]->_transformation;
    Vector3f lower1((-0.3*2)+initGuess.x(), (-0.3*2)+initGuess.y(), (-0.2*2)+initGuess.z());
    Vector3f upper1((0.3*2)+initGuess.x(), (0.3*2)+initGuess.y(), (0.2*2)+initGuess.z());
    thetaRes = 0.02; //was 0.025

    previousReducedScan.clear();
    currentReducedScan.clear();
    middleResCm.subsample(previousReducedScan, previousScan);
    middleResCm.subsample(currentReducedScan, currentScan);
    middleResCm.convolveScan(previousReducedScan);
    middleResCm.scanMatch(currentReducedScan, lower1, upper1, thetaRes, max);
    
    initGuess = middleResCm.getMatches()[it-1]->_transformation;
    Vector3f newLow(-0.3+initGuess.x(), -0.3+initGuess.y(), -0.2+initGuess.z());
    Vector3f newUp(0.3+initGuess.x(), 0.3+initGuess.y(), 0.2+initGuess.z());
    thetaRes = 0.01;
    
    
    previousReducedScan.clear();
    currentReducedScan.clear();
    highResCm.subsample(previousReducedScan, previousScan);
    highResCm.subsample(currentReducedScan, currentScan);
    highResCm.convolveScan(previousReducedScan);
    highResCm.scanMatch(currentReducedScan, newLow, newUp, thetaRes, max);

    double innerEnd = highResCm.getMilliSecs();
    
    cout << "inner time: " << (innerEnd-innerStart)*1000 << endl;
    highResCm.clear();
    middleResCm.clear();
    lowResCm.clear();
    
    pOdom = cOdom;
    previousScan = currentScan;
    previousReducedScan = currentReducedScan;
    initialGuesses.push_back(initGuess);

//     cout << (end - start)*1000 << " ms " << endl;
    i = it;
  }

  double totalEnd = highResCm.getMilliSecs();
  cout << "Total time is: " << (totalEnd - totalStart)* 1000 << " ms " << endl;
  cout << "Average time is: " << ((totalEnd - totalStart)* 1000)/i << " ms " << endl;
  
  highResCm.clear();
  cout << "Got matches" << endl;
  vector<MatcherResult*> matches = highResCm.getMatches();
  cout << "size of matches: " << matches.size() << endl;
  Vector2fVector scan = logScan[0];
  Isometry2f adjust = Isometry2f::Identity();
  
//   float mapRadius = 50; //lower dimension for fakeSimulated.clf
  float mapRadius = 100; //higher dimension for dis.clf
//   CorrelativeMatcher cm1(resolution, mapRadius, kernelMaxValue/resolution, kernelMaxValue, 128);
  CorrelativeCharMatcher cm1(resolution, mapRadius, 5, kernelMaxValue);
  cm1.integrateScan(scan, 1, adjust);
  for(size_t it = 1; it < logScan.size(); ++it)
  {
    scan = logScan[it];
    Vector3f current = matches[it-1]->_transformation;
    if(current.x() == numeric_limits<char>::max())
    {
      current = initialGuesses[it-1];
    }
    Isometry2f innerAdjust;
    innerAdjust = Rotation2Df(current.z());
    innerAdjust.translation() = Vector2f(current.x(), current.y());
    adjust = adjust * innerAdjust;
    cm1.integrateScan(scan, 1., adjust);
  }
  
  ofstream a("charHierarchicalMap.ppm");
  cm1.saveScanAsPPM(a, true);
  a.close();
  
  exit(1);
}