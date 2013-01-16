#include <fstream>
#include <iostream>
#include "matching/gradientMatcher.h"
#include "stuff/logReader.h"

using namespace std;
using namespace Eigen;



int main()
{
  float resolution = 0.03;
  float kernelRange = 0.5;
  int hV = 10;
  
  float radius = 50;
  GradientMatcher gm(resolution, radius, kernelRange/resolution, kernelRange);
    
  const string logFile = "../logFile/dis.clf";
  LogReader lr(logFile, hV);

  vector<Vector2fVector> logScan = lr.getScans();
  vector<Vector3f> logPose = lr.getPoses();

  Vector2fVector previousScan, currentScan;
  Vector3f pOdom, cOdom;
  
  previousScan = logScan[0];
  pOdom = logPose[0];
  
  double totalStart = gm.getMilliSecs();
  for(size_t it = 1; it < logScan.size(); ++it)
  {
    Isometry2f t1;
    t1 = Rotation2Df(pOdom.z());
    t1.translation() = Vector2f(pOdom.x(), pOdom.y());
        
    currentScan = logScan[it];
    cOdom = logPose[it];
    Isometry2f t2;
    t2 = Rotation2Df(cOdom.z());
    t2.translation() = Vector2f(cOdom.x(), cOdom.y());

    Isometry2f delta = t1.inverse()*t2;
    MatrixXf mat = delta.rotation();
    float angle = atan2(mat(1, 0), mat(0, 0));
    Vector3f initGuess(delta.translation().x(), delta.translation().y(), angle);
    
    gm.convolveScan(previousScan);
    double start = gm.getMilliSecs();
    gm.scanMatch(currentScan, initGuess);
    double end = gm.getMilliSecs();
    gm.clear();    
    pOdom = cOdom;
    previousScan = currentScan;
    
    cout << (end - start) * 1000 << " ms " << endl;
  }
  double totalEnd = gm.getMilliSecs();
  cout << "Total time is: " << (totalEnd - totalStart)* 1000 << " ms " << endl;
  gm.clear();
  vector<MatcherResult*> matches = gm.getMatches();
  Vector2fVector scan = logScan[0];
  Isometry2f adjust = Isometry2f::Identity();

//   float mapRadius = 50; //lower dimension for fakeSimulated.clf
  float mapRadius = 100; //higher dimension for dis.clf
  GradientMatcher gm1(resolution, mapRadius, kernelRange/resolution, kernelRange);
  gm1.integrateScan(scan, 1., adjust);
  for(size_t it = 1; it < logScan.size(); ++it)
  {
    scan = logScan[it];
    Vector3f current = matches[it-1]->_transformation;
    Isometry2f innerAdjust;
    innerAdjust = Rotation2Df(current.z());
    innerAdjust.translation() = Vector2f(current.x(), current.y());
    adjust = adjust * innerAdjust;
    gm1.integrateScan(scan, 1., adjust);
  }
  
  ofstream a("gradientMap.ppm");
  gm1.saveScanAsPPM(a, true);
  a.close();
  
  exit(1);
}