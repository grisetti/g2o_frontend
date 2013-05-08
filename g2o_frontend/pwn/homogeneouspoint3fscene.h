#ifndef _HOMOGENEOUSPOINT3FSCENE_H_
#define _HOMOGENEOUSPOINT3FSCENE_H_

#include "homogeneouspoint3fstats.h"
#include "homogeneouspoint3fomega.h"

class HomogeneousPoint3fScene {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  HomogeneousPoint3fScene() {}
  
  inline const HomogeneousPoint3fVector& points() const { return _points; }
  inline const HomogeneousNormal3fVector& normals() const { return _normals; }
  inline const HomogeneousPoint3fStatsVector& stats() const { return _stats; }
  inline const HomogeneousPoint3fOmegaVector& pointOmegas() const { return _pointOmegas; }
  inline const HomogeneousPoint3fOmegaVector& normalOmegas() const { return _normalOmegas; }

  inline HomogeneousPoint3fVector& points() { return _points; }
  inline HomogeneousNormal3fVector& normals() { return _normals; }
  inline HomogeneousPoint3fStatsVector& stats() { return _stats; }
  inline HomogeneousPoint3fOmegaVector& pointOmegas() { return _pointOmegas; }
  inline HomogeneousPoint3fOmegaVector& normalOmegas() { return _normalOmegas; }

  bool load(const char *filename);
  bool load(std::istream &is);
  bool save(const char *filename, int step = 1, bool binary = false);
  bool save(std::ostream &os, int step = 1, bool binary = false);

 protected:
  HomogeneousPoint3fVector _points;
  HomogeneousNormal3fVector _normals; 
  
  HomogeneousPoint3fStatsVector _stats;
  
  HomogeneousPoint3fOmegaVector _pointOmegas;
  HomogeneousPoint3fOmegaVector _normalOmegas;
};

#endif
