#ifndef _PWN_INFORMATIONMATRIXCALCULATOR_H_
#define _PWN_INFORMATIONMATRIXCALCULATOR_H_

#include "g2o_frontend/boss_logger/eigen_boss_plugin.h" 
#include "g2o_frontend/boss/object_data.h"
#include "g2o_frontend/boss/identifiable.h"

#include "stats.h"
#include "informationmatrix.h"

namespace pwn {

  typedef Eigen::DiagonalMatrix<float, 4> Diagonal4f;

  class InformationMatrixCalculator : public boss::Identifiable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
    InformationMatrixCalculator(int id=-1, boss::IdContext* context=0): Identifiable(id,context) {
      _flatInformationMatrix.setZero();
      _nonFlatInformationMatrix.setZero();
      _flatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
      _nonFlatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
      _curvatureThreshold = 1.0f;
    }

    inline InformationMatrix flatInformationMatrix() const { return _flatInformationMatrix; }
    inline InformationMatrix nonFlatInformationMatrix() const { return _nonFlatInformationMatrix; }
    inline float curvatureThreshold() const { return _curvatureThreshold; }

    inline void setFlatInformationMatrix(const InformationMatrix flatInformationMatrix_) { _flatInformationMatrix = flatInformationMatrix_; }
    inline void setNonFlatInformationMatrix(const InformationMatrix nonFlatInformationMatrix_) { _nonFlatInformationMatrix = nonFlatInformationMatrix_; }
    inline void setCurvatureThreshold(const float curvatureThreshold_) { _curvatureThreshold = curvatureThreshold_; }

    virtual void compute(InformationMatrixVector &informationMatrix,
			 const StatsVector &stats,
			 const NormalVector &imageNormals) = 0;


    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
  
  protected:
    InformationMatrix _flatInformationMatrix;
    InformationMatrix _nonFlatInformationMatrix;
    float _curvatureThreshold;
  };

  class PointInformationMatrixCalculator : public InformationMatrixCalculator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PointInformationMatrixCalculator(int id=-1, boss::IdContext* context=0): InformationMatrixCalculator(id,context) {
      _flatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1000.0f, 1.0f, 1.0f));
      _nonFlatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
      _curvatureThreshold = 0.02f;
    }

    virtual void compute(InformationMatrixVector &informationMatrix,
			 const StatsVector &statsVector,
			 const NormalVector &imageNormals);


    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
  };

  class NormalInformationMatrixCalculator : public InformationMatrixCalculator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    NormalInformationMatrixCalculator(int id=-1, boss::IdContext* context=0): InformationMatrixCalculator(id,context) {
      _flatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(100.0f, 100.0f, 100.0f));
      _nonFlatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
      _curvatureThreshold = 0.02f;
    }

    virtual void compute(InformationMatrixVector &informationMatrix,
			 const StatsVector &statsVector,
			 const NormalVector &imageNormals);

    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);

  };

}

#endif
