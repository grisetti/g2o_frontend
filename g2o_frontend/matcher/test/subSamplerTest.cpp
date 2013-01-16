#include <fstream>
#include <iostream>
#include "g2o_frontend/matcher/matching/scanMatcher.h"
#include "g2o_frontend/matcher/utils/logReader.h"


using namespace std;
using namespace Eigen;



int main()
{
  Vector2fVector scan, reducedScan;
  Vector2f a(1., 2.);
  Vector2f b(1., 2.);
  Vector2f c(2., 2.);
  Vector2f d(1., 3.);
  Vector2f e(2., 1);
  Vector2f f(-1, 1);
  
  scan.push_back(a);
  scan.push_back(b);
  scan.push_back(c);
  scan.push_back(d);
  scan.push_back(e);
  scan.push_back(f);
  
  
  float resolution = 0.03;
  float kernelRange = 0.5;
  int hV = 10;
  
  float radius = 50;
  ScanMatcher sm(resolution, radius, kernelRange/resolution, kernelRange);
  
  
  sm.subsample(reducedScan, scan);
  
  cout << "Scan size is: " << scan.size() << ", reducedScan size is: " << reducedScan.size() << endl;
  
  for(Vector2fVector::const_iterator it = reducedScan.begin(); it != reducedScan.end(); ++it)
  {
    const Vector2f p = *it;
    cout << "current point is: " << p.x() << ", " << p.y() << endl;
  }
}
