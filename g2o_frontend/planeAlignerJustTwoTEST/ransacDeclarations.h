#ifndef _MAL_RANSACDEC
#define _MAL_RANSACDEC

#include <Eigen/Geometry>
#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
#include "g2o_frontend/data/point_cloud_data.h"
#include "g2o_frontend/sensor_data/rgbd_data.h"
#include "g2o_frontend/ransac/alignment_plane_linear.h"
#include "g2o_frontend/ransac/ransac.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"

using namespace Eigen;
using namespace g2o;
using namespace Slam3dAddons;

template <typename TypeDomain_, int dimension_>
struct EuclideanMapping{
    typedef TypeDomain_ TypeDomain;
    typedef typename Eigen::Matrix<double, dimension_, 1> VectorType;
    virtual int dimension() const {return dimension_;}
    virtual TypeDomain fromVector(const VectorType& v) const =  0;
    virtual VectorType toVector(const TypeDomain& t) const = 0;
};

template <int dimension_>
struct VectorMapping : public EuclideanMapping<Eigen::Matrix<double, dimension_, 1>, dimension_>{
    typedef typename EuclideanMapping<Eigen::Matrix<double, dimension_, 1>, dimension_>::TypeDomain TypeDomain;
    typedef typename EuclideanMapping<Eigen::Matrix<double, dimension_, 1>, dimension_>::VectorType VectorType;
    virtual TypeDomain fromVector(const VectorType& v) const {return v;}
    virtual VectorType toVector(const TypeDomain& t) const {return t;}
};


struct PlaneMapping: public EuclideanMapping<Plane3D,4>{
    typedef typename EuclideanMapping<Plane3D, 4>::TypeDomain TypeDomain;
    typedef typename EuclideanMapping<Plane3D, 4>::VectorType VectorType;
    virtual TypeDomain fromVector(const VectorType& v) const {
        Plane3D l(v);
        return l;
    }
    virtual VectorType toVector(const TypeDomain& t) const {
        return t.toVector();
    }
};

typedef std::vector<g2o_frontend::Correspondence> CorrespondenceVector;

template <typename MappingType, typename RansacType, typename EdgeCorrespondenceType>
bool testRansac(typename RansacType::TransformType& result,CorrespondenceVector& correspondences,g2o_frontend::IndexVector & iv){

    RansacType ransac;

    //ransac.correspondenceValidators()=validators;
    ransac.setCorrespondences(correspondences);
    ransac.setMaxIterations(10000);
    ransac.setInlierErrorThreshold(0.02);
    ransac.setInlierStopFraction(0.2);

    return ransac(result,iv);
}
//**************************************************************************************************************************************
//**************************************************************************************************************************************


#endif
