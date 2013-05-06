#ifndef _HOMOGENEOUSPOINT3FSCENE_H_
#define _HOMOGENEOUSPOINT3FSCENE_H_

#include "homogeneousvector4f.h"
#include "homogeneouspoint3fstats.h"
#include "homogeneouspoint3fomega.h"

class HomogeneousPoint3fScene {
 public:
  HomogeneousPoint3fScene() {}
  
  HomogeneousPoint3fVector& points() { return _points; }
  HomogeneousNormal3fVector& normals() { return _normals; }
  HomogeneousPoint3fStatsVector& stats() { return _stats; }
  HomogeneousPoint3fOmegaVector& pointOmegas() { return _pointOmegas; }
  HomogeneousPoint3fOmegaVector& normalOmegas() { return _normalOmegas; }

  void setPoints(HomogeneousPoint3fVector points_) { _points = points_; }
  void setNormals(HomogeneousNormal3fVector normals_) { _normals = normals_; }
  void setStats(HomogeneousPoint3fStatsVector stats_) { _stats = stats_; }
  void setPointOmegas(HomogeneousPoint3fOmegaVector pointOmegas_) { _pointOmegas = pointOmegas_; }
  void setNormalOmegas(HomogeneousPoint3fOmegaVector normalOmegas_) { _normalOmegas = normalOmegas_; }

  bool load(const char* filename);
  bool load(std::istream & is);
  bool save(const char* filename, int step, bool binary);
  bool save(std::ostream &os, int step=1, bool binary=false) ;

 protected:
  HomogeneousPoint3fVector _points;
  HomogeneousNormal3fVector _normals;
  
  HomogeneousPoint3fStatsVector _stats;
  
  HomogeneousPoint3fOmegaVector _pointOmegas;
  HomogeneousPoint3fOmegaVector _normalOmegas;
};

#endif
