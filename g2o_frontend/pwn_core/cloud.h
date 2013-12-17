#pragma once

#include "stats.h"
#include "informationmatrix.h"
#include "gaussian3.h"

namespace pwn {

  class Cloud {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Cloud() {}
    virtual ~Cloud() {}

    inline const PointVector& points() const { return _points; }
    inline PointVector& points() { return _points; }
    
    inline const NormalVector& normals() const { return _normals; }
    inline NormalVector& normals() { return _normals; }
    
    inline const StatsVector& stats() const { return _stats; }
    inline StatsVector& stats() { return _stats; }
    
    inline const InformationMatrixVector& pointInformationMatrix() const { return _pointInformationMatrix; }
    inline InformationMatrixVector& pointInformationMatrix() { return _pointInformationMatrix; }

    inline const InformationMatrixVector& normalInformationMatrix() const { return _normalInformationMatrix; }
    inline InformationMatrixVector& normalInformationMatrix() { return _normalInformationMatrix; }
    
    inline const std::vector<int>& traversabilityVector() const { return _traversabilityVector; }
    inline std::vector<int>& traversabilityVector() { return _traversabilityVector; }
    
    inline const Gaussian3fVector& gaussians() const { return _gaussians; }
    inline Gaussian3fVector& gaussians() { return _gaussians; }

    bool load(Eigen::Isometry3f &T, const char *filename);
    bool load(Eigen::Isometry3f &T, std::istream &is);

    bool save(const char *filename, Eigen::Isometry3f T = Eigen::Isometry3f::Identity(), int step = 1, bool binary = true);
    bool save(std::ostream &os, Eigen::Isometry3f T = Eigen::Isometry3f::Identity(), int step = 1, bool binary = true);
    
    void clear();
    void add(Cloud cloud, const Eigen::Isometry3f &T = Eigen::Isometry3f::Identity());
    
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
