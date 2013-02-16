#include "g2o_frontend/basemath/bm_se3.h"
#include "alignment_plane_linear.h"
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

using namespace std;

namespace g2o_frontend{
using namespace g2o;
using namespace Slam3dAddons;
using namespace Eigen;

AlignmentAlgorithmPlaneLinear::AlignmentAlgorithmPlaneLinear(): AlignmentAlgorithmSE3Plane3D(2)
{

}

typedef Eigen::Matrix<double, 3, 3>  Matrix3x3d;
typedef Eigen::Matrix<double, 3, 9>  Matrix3x9d;
typedef Eigen::Matrix<double, 12, 1> Matrix12x1d;
typedef Eigen::Matrix<double, 4, 1>  Matrix4x1d;
typedef Eigen::Matrix<double, 3, 1>  Matrix3x1d;
typedef Eigen::Matrix<double, 3, 3>  Matrix3x3d;
typedef Eigen::Matrix<double, 9, 9>  Matrix9x9d;
typedef Eigen::Matrix<double, 9, 1>  Matrix9x1d;

bool AlignmentAlgorithmPlaneLinear::operator()(AlignmentAlgorithmPlaneLinear::TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices)
{

    //If my correspondaces indices are less then a minimum threshold, stop it please
    if ((int)indices.size()<minimalSetSize()) return false;

    //My initial guess for the transformation it's the identity matrix
    //maybe in the future i could have a less rough guess
    transform = Isometry3d::Identity();
    //Unroll the matrix to a vector
    Matrix12x1d x=homogeneous2vector(transform.matrix());
    Matrix9x1d rx=x.block<9,1>(0,0);

    //Initialization of variables, melting fat, i've so much space
    Matrix9x9d H;
    H.setZero();
    Matrix9x1d b;
    b.setZero();
    Matrix3x9d A;

    Matrix3x3d Omega;
    Omega.setOnes();

    //iteration for each correspondace
    for (size_t i=0; i<indices.size(); i++)
    {
        A.setZero();
        const Correspondence& c = correspondences[indices[i]];
        const EdgePlane* edge = static_cast<const EdgePlane*>(c.edge());

        //estraggo i vertici dagli edge
        const VertexPlane* v1 = static_cast<const VertexPlane*>(edge->vertex(0));
        const VertexPlane* v2 = static_cast<const VertexPlane*>(edge->vertex(1));

        //recupero i dati dei piani
        const AlignmentAlgorithmPlaneLinear::PointEstimateType& pi= v1->estimate();
        const AlignmentAlgorithmPlaneLinear::PointEstimateType& pj= v2->estimate();

        //recupeo le normali, mi servono per condizionare la parte rotazionale
        Matrix3x1d ni;
        Matrix3x1d nj;
        Matrix4x1d coeffs_i=pi.toVector();
        Matrix4x1d coeffs_j=pj.toVector();

        ni=coeffs_i.block<3,1>(0,0);
        nj=coeffs_j.block<3,1>(0,0);

        //inizializzo lo jacobiano, solo la parte rotazionale (altrimenti cado in una situazione non lineare)
        A.block<1,3>(0,0)=nj.transpose();
        A.block<1,3>(1,3)=nj.transpose();
        A.block<1,3>(2,6)=nj.transpose();

        //errore
        //inizializzo errore
        Vector3d ek;
        ek.setZero();
        ek=A*x.block<9,1>(0,0)-ni;
        H+=A.transpose()*Omega*A;
        b+=A.transpose()*Omega*ek;
    }
    LDLT<Matrix9x9d>ldlt(H);
    if (ldlt.isNegative()) return false;
    rx=ldlt.solve(-b); // using a LDLT factorizationldlt;
    x.block<9,1>(0,0)=rx;

    Matrix12x1d Xtemp=x;
    Xtemp.block<0,3>(0,0)+=x.block<3,0>(0,0);
    Xtemp.block<0,3>(1,0)+=x.block<3,0>(3,0);
    Xtemp.block<0,3>(2,0)+=x.block<3,0>(6,0);

    //now the problem si solved but i could have (and probably i have!)
    //a not orthogonal rotation matrix, so i've to recondition it

    // recondition the rotation
    JacobiSVD<Matrix3x3d> svd(Xtemp.block<3,3>(0,0), Eigen::ComputeThinU | Eigen::ComputeThinV);
    if (svd.singularValues()(0)<.5) return false;
    Matrix3x3d R=svd.matrixU()*svd.matrixV().transpose();
    Isometry3d X = Isometry3d::Identity();
    X.linear()=R;
    X.translation() = Xtemp.block<3,1>(0,3);

    Matrix3x3d H2=Matrix3x3d::Zero();
    Vector3d b2=Vector3d::Zero();
    Matrix3x1d A2=Matrix3x1d::Zero();
    //at this point rotation is cured, now it's time to work on the translation
    double err=0;
    for (size_t i=0; i<indices.size(); i++)
    {
        const Correspondence& c = correspondences[indices[i]];
        const EdgePlane* edge = static_cast<const EdgePlane*>(c.edge());

        //estraggo i vertici dagli edge
        const VertexPlane* v1 = static_cast<const VertexPlane*>(edge->vertex(0));
        const VertexPlane* v2 = static_cast<const VertexPlane*>(edge->vertex(1));

        //recupero i dati dei piani
        const AlignmentAlgorithmPlaneLinear::PointEstimateType& pi= v1->estimate();
        const AlignmentAlgorithmPlaneLinear::PointEstimateType& pj= v2->estimate();

        double di=0;
        double dj=0;
        Matrix3x1d ni;
        Matrix3x1d nj;
        Matrix4x1d coeffs_i=pi.toVector();
        Matrix4x1d coeffs_j=pj.toVector();

        ni=coeffs_i.block<3,1>(0,0);
        nj=coeffs_j.block<3,1>(0,0);

        di=coeffs_i.w();
        dj=coeffs_i.w();
        A2=(nj.transpose()*R.transpose());

        double ek;
        ek=dj-A2.transpose()*X.translation()-di;
        H2+=A2*Omega(4,4)*A2.transpose();
        b2+= (A2.transpose()*Omega(4,4)*ek);
        err += ek*Omega(4,4)*ek;
    }

    //solving the system
    Vector3d dt;
    dt=H2.ldlt().solve(-b2); // using a LDLT factorizationldlt;
    X.translation()+=dt;
    transform = X;
    cerr << "transform: " << endl;
    cerr << g2o::internal::toVectorMQT(transform) << endl;;
    return true;
}


RansacPlaneLinear::RansacPlaneLinear():  GeneralizedRansac<AlignmentAlgorithmPlaneLinear>(2)
{

}

}
