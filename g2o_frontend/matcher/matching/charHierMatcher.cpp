#include "charHierMatcher.h"


float HierarchicalCharMatcherResult::matchingScore() const
{
  return _matchingScore;
}


HierarchicalCharMatcherResult::~HierarchicalCharMatcherResult() {}



HierarchicalCharMatcher::HierarchicalCharMatcher(const float& resolution, const float& radius,
                                               const int& kernelSize, const float& kernelMaxValue, int gridScale)
: CorrelativeCharMatcher(resolution, radius, kernelSize, kernelMaxValue, gridScale)
{}


HierarchicalCharMatcher::HierarchicalCharMatcher(const CharGrid& g, const int& kernelSize,
                                               const float& kernelMaxValue, int gridScale)
: CorrelativeCharMatcher(g, kernelSize, kernelMaxValue, gridScale)
{}



void Grid::hierarchicalSearch(std::vector<MatcherResult>& mresvec,
            const Vector2dVector& points,
            const RegionVector& regions, const MatchingParametersVector& paramsVec) {
  mresvec.clear();
  RegionVector currentRegions = regions;
  clock_t t_ini, t_fin;
  double secs;
  for (size_t i = 0; i<paramsVec.size()-1; i++){
    cout << "Iteration " << i << endl;
    MatchingParameters params = paramsVec[i];

    t_ini = clock();
    greedySearch(mresvec, points, currentRegions, params);
    t_fin = clock();
    secs = (double)(t_fin - t_ini) / CLOCKS_PER_SEC;
    printf("%.16g milisegundos\n", secs * 1000.0);
    if (mresvec.size()){
      cerr <<  "bestScore iteration " << i << " = " << mresvec[0].score << endl;
      cerr << "Scores between: " << mresvec[0].score << " and " << mresvec[mresvec.size()-1].score << endl;
    }
    currentRegions.clear();
    for (size_t i = 0; i < mresvec.size(); i++){
      Vector3d best=mresvec[i].transformation;
      Region reg;
      reg.lowerLeft  = - (params.resultsDiscretization * .5) + best;
      reg.upperRight =   (params.resultsDiscretization * .5) + best;
      currentRegions.push_back(reg);
    }
  }
  cout << "Iteration " << paramsVec.size()-1 << endl;
  MatchingParameters params = paramsVec[paramsVec.size()-1];

  t_ini = clock();
  greedySearch(mresvec, points, currentRegions, params);
  t_fin = clock();

  secs = (double)(t_fin - t_ini) / CLOCKS_PER_SEC;
  printf("%.16g milisegundos\n", secs * 1000.0);
  if (mresvec.size()){
    cerr <<  "bestScore iteration " <<  paramsVec.size()-1 << " = " << mresvec[0].score << endl;
    cerr << "Scores between: " << mresvec[0].score << " and " << mresvec[mresvec.size()-1].score << endl;
  }
}

void Grid::hierarchicalSearch(std::vector<MatcherResult>& mresvec,
                  const Vector2dVector& points,
                  Eigen::Vector3d lowerLeftF, Eigen::Vector3d upperRightF, double thetaRes,
                  double maxScore, double dx, double dy, double dth)

{
  RegionVector regions(1);
  regions[0].lowerLeft=lowerLeftF;
  regions[0].upperRight=upperRightF;

  MatchingParametersVector pvec(4);

  pvec[0].searchStep=Vector3d(8*res(), 8*res(), 4*thetaRes);
  pvec[0].maxScore=maxScore;
  pvec[0].resultsDiscretization=Vector3d(dx*8,dy*8,dth*8);

  pvec[1].searchStep=Vector3d(4*res(), 4*res(), 2*thetaRes);
  pvec[1].maxScore=maxScore;
  pvec[1].resultsDiscretization=Vector3d(dx*4,dy*4,dth*4);

  pvec[2].searchStep=Vector3d(2*res(), 2*res(), thetaRes);
  pvec[2].maxScore=maxScore;
  pvec[2].resultsDiscretization=Vector3d(dx*2,dy*2,dth*2);

  pvec[3].searchStep=Vector3d(res(), res(), thetaRes);
  pvec[3].maxScore=maxScore;
  pvec[3].resultsDiscretization=Vector3d(dx,dy,dth);

  hierarchicalSearch(mresvec,  points, regions, pvec);
}
void Grid::hierarchicalSearch(std::vector<MatcherResult>& mresvec,
            const Vector2dVector& points,
                const RegionVector& regions,
                  double thetaRes, double maxScore, double dx, double dy, double dth, int nLevels){

  MatchingParametersVector pvec;

  for (int i = nLevels-1; i >= 0; i--){
    int m = pow(2, i);
    int mtheta = (m/2 < 1) ? m : m/2;

    MatchingParameters mp;
    mp.searchStep=Vector3d(m*res(), m*res(), mtheta*thetaRes);
    mp.maxScore=maxScore;
    mp.resultsDiscretization=Vector3d(dx*m,dy*m,dth*m);

    pvec.push_back(mp);
  }



  return hierarchicalSearch(mresvec, points, regions, pvec);


}

void Grid::hierarchicalSearch(std::vector<MatcherResult>& mresvec,
                  const Vector2dVector& points,
                  Eigen::Vector3d lowerLeftF, Eigen::Vector3d upperRightF, double thetaRes,
                  double maxScore, double dx, double dy, double dth, int nLevels)

{
  RegionVector regions(1);
  regions[0].lowerLeft=lowerLeftF;
  regions[0].upperRight=upperRightF;

  return hierarchicalSearch(mresvec,  points, regions, thetaRes, maxScore, dx, dy, dth, nLevels);
}
