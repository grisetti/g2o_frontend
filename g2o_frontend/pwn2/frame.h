#ifndef _FRAME_H_
#define _FRAME_H_

#include "pointstats.h"
#include "informationmatrix.h"

namespace pwn {

class Frame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frame() {}
  virtual ~Frame() {}

  inline const PointVector& points() const { return _points; }
  inline const NormalVector& normals() const { return _normals; }
  inline const PointStatsVector& stats() const { return _stats; }
  inline const InformationMatrixVector& pointInformationMatrix() const { return _pointInformationMatrix; }
  inline const InformationMatrixVector& normalInformationMatrix() const { return _normalInformationMatrix; }
  inline const std::vector<int>& traversabilityVector() const { return _traversabilityVector; }

  inline PointVector& points() { return _points; }
  inline NormalVector& normals() { return _normals; }
  inline PointStatsVector& stats() { return _stats; }
  inline InformationMatrixVector& pointInformationMatrix() { return _pointInformationMatrix; }
  inline InformationMatrixVector& normalInformationMatrix() { return _normalInformationMatrix; }
  inline std::vector<int>& traversabilityVector() { return _traversabilityVector; }


  bool load(const char *filename);
  bool load(std::istream &is);
  bool save(const char *filename, Eigen::Isometry3f T, int step = 1, bool binary = false);
  bool save(const char *filename, int step = 1, bool binary = false);
  bool save(ostream &os, Eigen::Isometry3f T, int step, bool binary);
  bool save(std::ostream &os, int step = 1, bool binary = false);
  void clear();
  void transformInPlace(const Eigen::Isometry3f& T);

 protected:
  PointVector _points;
  NormalVector _normals;
  PointStatsVector _stats;
  std::vector<int> _traversabilityVector;
  InformationMatrixVector _pointInformationMatrix;
  InformationMatrixVector _normalInformationMatrix;
};

}

#endif
