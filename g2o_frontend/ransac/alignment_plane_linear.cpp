#include "g2o_frontend/basemath/bm_se3.h"
#include "alignment_plane_linear.h"
#include <Eigen/SVD>
#include <Eigen/Cholesky>

#include <iostream>
using namespace std;

namespace g2o_frontend{
using namespace g2o;
using namespace Slam3dAddons;
using namespace Eigen;

AlignmentAlgorithmPlane3DLinear::AlignmentAlgorithmPlane3DLinear(): AlignmentAlgorithmSE3Plane3D(2)
{

}

bool AlignmentAlgorithmPlane3DLinear::operator()(AlignmentAlgorithmPlane3DLinear::TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices)
{

    //If my correspondaces indices are less then a minimum threshold, stop it please
    if ((int)indices.size()<minimalSetSize()) return false;
    
    //My initial guess for the transformation it's the identity matrix
    //maybe in the future i could have a less rough guess
    transform = Isometry3d::Identity();
    //Unroll the matrix to a vector
    Vector12d x=homogeneous2vector(transform.matrix());

    //Initialization of variables, melting fat, i've so much space
    Matrix12d H;
    H.setZero();
    Vector12d b;
    b.setZero();
    Matrix6x12d A;

    //iteration for each correspondace
    for (size_t i=0; i<indices.size(); i++)
    {
        A.setZero();
        const Correspondence& c = correspondences[indices[i]];
        const EdgeLine3D* edge = static_cast<const EdgeLine3D*>(c.edge());
    }


    return true;
}


RansacPlane3DLinear::RansacPlane3DLinear():  GeneralizedRansac<AlignmentAlgorithmPlane3DLinear>(2)
{

}

}
