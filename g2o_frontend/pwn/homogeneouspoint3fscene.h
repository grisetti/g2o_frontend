#ifndef _HOMOGENEOUSPOINT3FSCENE_H_
#define _HOMOGENEOUSPOINT3FSCENE_H_

#include "homogeneousvector4f.h"
#include "homogeneouspoint3fstats.h"
#include "homogeneouspoint3fomega.h"
#include "correspondencegenerator.h"

class HomogeneousPoint3fScene {
 public:
  HomogeneousPoint3fScene() {}
  
  DepthImage& depthImage() { return _depthImage; }
  Eigen::MatrixXi& indexImage() { return _indexImage; }
  HomogeneousPoint3fVector& points() { return _points; }
  HomogeneousNormal3fVector& normals() { return _normals; }
  HomogeneousPoint3fStatsVector& stats() { return _stats; }
  HomogeneousPoint3fOmegaVector& pointOmegas() { return _pointOmegas; }
  HomogeneousPoint3fOmegaVector& normalOmegas() { return _normalOmegas; }
  Eigen::Isometry3f cameraPose() { return _cameraPose; }

  void setDepthImage(DepthImage &depthImage_) { _depthImage = depthImage_; }
  void setIndexImage(Eigen::MatrixXi &indexImage_) { _indexImage = indexImage_; }
  void setPoints(HomogeneousPoint3fVector &points_) { _points = points_; }
  void setNormals(HomogeneousNormal3fVector &normals_) { _normals = normals_; }
  void setStats(HomogeneousPoint3fStatsVector &stats_) { _stats = stats_; }
  void setPointOmegas(HomogeneousPoint3fOmegaVector &pointOmegas_) { _pointOmegas = pointOmegas_; }
  void setNormalOmegas(HomogeneousPoint3fOmegaVector &normalOmegas_) { _normalOmegas = normalOmegas_; }
  void setCameraPose(Eigen::Isometry3f cameraPose_) { _cameraPose = cameraPose_; }

 protected:
  DepthImage _depthImage;
  Eigen::MatrixXi _indexImage;

  HomogeneousPoint3fVector _points;
  HomogeneousNormal3fVector _normals;
  
  HomogeneousPoint3fStatsVector _stats;
  
  HomogeneousPoint3fOmegaVector _pointOmegas;
  HomogeneousPoint3fOmegaVector _normalOmegas;

  Eigen::Isometry3f _cameraPose;
};

std::vector<HomogeneousPoint3fScene> HomogeneousPoint3fSceneVector;

#endif
