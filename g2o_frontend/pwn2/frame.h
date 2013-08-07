#ifndef _PWN_FRAME_H_
#define _PWN_FRAME_H_

#include "stats.h"
#include "informationmatrix.h"
#include "gaussian3.h"

namespace pwn {

class Frame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frame() {}
  virtual ~Frame() {}

  inline const PointVector& points() const { return _points; }
  inline const NormalVector& normals() const { return _normals; }
  inline const StatsVector& stats() const { return _stats; }
  inline const InformationMatrixVector& pointInformationMatrix() const { return _pointInformationMatrix; }
  inline const InformationMatrixVector& normalInformationMatrix() const { return _normalInformationMatrix; }
  inline const std::vector<int>& traversabilityVector() const { return _traversabilityVector; }
  inline const Gaussian3fVector& gaussians() const { return _gaussians; }

  inline PointVector& points() { return _points; }
  inline NormalVector& normals() { return _normals; }
  inline StatsVector& stats() { return _stats; }
  inline InformationMatrixVector& pointInformationMatrix() { return _pointInformationMatrix; }
  inline InformationMatrixVector& normalInformationMatrix() { return _normalInformationMatrix; }
  inline std::vector<int>& traversabilityVector() { return _traversabilityVector; }
  inline Gaussian3fVector& gaussians() { return _gaussians; }

  bool load(Eigen::Isometry3f &T, const char *filename);
  bool load(Eigen::Isometry3f &T, std::istream &is);
  bool save(const char *filename, int step = 1, bool binary = false, Eigen::Isometry3f T = Eigen::Isometry3f::Identity());
  bool save(std::ostream &os, int step = 1, bool binary = false, Eigen::Isometry3f T = Eigen::Isometry3f::Identity());
  void clear();
  void add(Frame frame, const Eigen::Isometry3f &T = Eigen::Isometry3f::Identity());
  void transformInPlace(const Eigen::Isometry3f& T);

 protected:
  PointVector _points;
  NormalVector _normals;
  StatsVector _stats;
  InformationMatrixVector _pointInformationMatrix;
  InformationMatrixVector _normalInformationMatrix;
  std::vector<int> _traversabilityVector;
  Gaussian3fVector _gaussians;
};

}

#endif
