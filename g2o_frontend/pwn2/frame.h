#ifndef _FRAME_H_
#define _FRAME_H_

#include "pointstats.h"
#include "informationmatrix.h"

class Frame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frame() {}
  
  inline const PointVector& points() const { return _points; }
  inline const NormalVector& normals() const { return _normals; }
  inline const PointStatsVector& stats() const { return _stats; }
  inline const InformationMatrixVector& pointInformationMatrix() const { return _pointInformationMatrix; }
  inline const InformationMatrixVector& normalInformationMatrix() const { return _normalInformationMatrix; }

  inline PointVector& points() { return _points; }
  inline NormalVector& normals() { return _normals; }
  inline PointStatsVector& stats() { return _stats; }
  inline InformationMatrixVector& pointInformationMatrix() { return _pointInformationMatrix; }
  inline InformationMatrixVector& normalInformationMatrix() { return _normalInformationMatrix; }

  bool load(const char *filename);
  bool load(std::istream &is);
  bool save(const char *filename, int step = 1, bool binary = false);
  bool save(std::ostream &os, int step = 1, bool binary = false);
  void clear();
  void transformInPlace(const Eigen::Isometry3f& T);

 protected:
  PointVector _points;
  NormalVector _normals;
  PointStatsVector _stats;
  InformationMatrixVector _pointInformationMatrix;
  InformationMatrixVector _normalInformationMatrix;
};

#endif
