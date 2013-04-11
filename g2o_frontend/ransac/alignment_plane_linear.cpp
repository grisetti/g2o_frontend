#include "g2o_frontend/basemath/bm_se3.h"
#include "alignment_plane_linear.h"
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <math.h>
using namespace std;

namespace g2o_frontend{
using namespace g2o;
using namespace Slam3dAddons;
using namespace Eigen;

#define DEBUG 0

AlignmentAlgorithmPlaneLinear::AlignmentAlgorithmPlaneLinear(): AlignmentAlgorithmSE3Plane3D(3)
{

}

typedef Eigen::Matrix<double, 3, 3>  Matrix3x3d;
typedef Eigen::Matrix<double, 3, 9>  Matrix3x9d;
typedef Eigen::Matrix<double, 12, 1> Matrix12x1d;
typedef Eigen::Matrix<double, 4, 1>  Matrix4x1d;
typedef Eigen::Matrix<double, 3, 1>  Matrix3x1d;
typedef Eigen::Matrix<double, 3, 3>  Matrix3x3d;
typedef Eigen::Matrix<double, 9, 9>  Matrix9d;
typedef Eigen::Matrix<double, 9, 1>  Matrix9x1d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;

bool AlignmentAlgorithmPlaneLinear::operator()(AlignmentAlgorithmPlaneLinear::TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices)
{
    double err=0;
    //If my correspondaces indices are less then a minimum threshold, stop it please
    if ((int)indices.size()<minimalSetSize()) return false;

    //My initial guess for the transformation it's the identity matrix
    //maybe in the future i could have a less rough guess
    transform = Isometry3d::Identity();
    //Unroll the matrix to a vector
    Vector12d x=homogeneous2vector(transform.matrix());
    Matrix9x1d rx=x.block<9,1>(0,0);

    //Initialization of variables, melting fat, i've so much space
    Matrix9d H;
    H.setZero();
    Vector9d b;
    b.setZero();
    Matrix3x9d A;
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
        Vector3d ni;
        Vector3d nj;
        Vector4d coeffs_i=pi.toVector();
        Vector4d coeffs_j=pj.toVector();

        ni=coeffs_i.head(3);
        nj=coeffs_j.head(3);

        //inizializzo lo jacobiano, solo la parte rotazionale (per ora)
        A.block<1,3>(0,0)=nj.transpose();
        A.block<1,3>(1,3)=nj.transpose();
        A.block<1,3>(2,6)=nj.transpose();

        if(DEBUG){
        cerr << "[DEBUG] PI"<<endl;
        cerr << coeffs_i<<endl;
        cerr << "[DEBUG] PJ "<<endl;
        cerr << coeffs_j<<endl;
        cerr << "[ROTATION JACOBIAN][PLANE "<<i<<"]"<<endl;
        cerr << A<<endl;
        }
        //errore
        //inizializzo errore
        Vector3d ek;
        ek.setZero();
        ek=A*x.block<9,1>(0,0)-coeffs_i.head(3);
        H+=A.transpose()*A;

        b+=A.transpose()*ek;

        err+=abs(ek.dot(ek));

        if(DEBUG)
        cerr << "[ROTATIONAL Hessian for plane "<<i<<"]"<<endl<<H<<endl;
        if(DEBUG)
        cerr << "[ROTATIONAL B for plane "<<i<<"]"<<endl<<b<<endl;
    }
    LDLT<Matrix9d> ldlt(H);
    if (ldlt.isNegative()) return false;
    rx=ldlt.solve(-b); // using a LDLT factorizationldlt;

    x.block<3,1>(0,0)+=rx.block<3,1>(0,0);
    x.block<3,1>(3,0)+=rx.block<3,1>(3,0);
    x.block<3,1>(6,0)+=rx.block<3,1>(6,0);
    if(DEBUG){
    cerr << "Solving the linear system"<<endl;
    cerr << "------------>H"<<endl;
    cerr << H<<endl;
    cerr << "------------>b"<<endl;
    cerr << b<<endl;
    cerr << "------------>rx"<<endl;
    cerr << rx<<endl;
    cerr << "------------>xTEMP"<<endl;
    cerr << vector2homogeneous(x)<<endl<<endl;

    cerr << "łłłłłłłłłłł łłłłłłłłłłł łłłłłłłłłłł łłłłłłłłłłł łłłłłłłłłłł łłłłłłłłłłł"<<endl;
    cerr << "łłłłłłłłłłł Ringraziamo Cthulhu la parte rotazionale è finitałłłłłłłłłłł"<<endl;
    cerr << "łłłłłłłłłłł łłłłłłłłłłł łłłłłłłłłłł łłłłłłłłłłł łłłłłłłłłłł łłłłłłłłłłł"<<endl;
    }
    Matrix4d Xtemp=vector2homogeneous(x);

    //now the problem si solved but i could have (and probably i have!)
    //a not orthogonal rotation matrix, so i've to recondition it

    Matrix3x3d R=Xtemp.block<3,3>(0,0);
    // recondition the rotation
    JacobiSVD<Matrix3x3d> svd(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
    if (svd.singularValues()(0)<.5) return false;
    R=svd.matrixU()*svd.matrixV().transpose();
    Isometry3d X = Isometry3d::Identity();
    X.linear()=R;
    X.translation()= x.block<3,1>(0,3);
if(DEBUG)
    cerr << "X dopo il ricondizionamento appare come:"<<endl;
if(DEBUG)
    cerr << X.matrix()<<endl;


    Matrix3d H2=Matrix3d::Zero();
    Vector3d b2=Vector3d::Zero();
    Vector3d A2=Vector3d::Zero();

    //at this point rotation is cured, now it's time to work on the translation

    for (size_t i=0; i<indices.size(); i++)
    {
        if(DEBUG)
        cerr << "[TRANSLATION JACOBIAN START][PLANE "<<i<<"]"<<endl;

        const Correspondence& c = correspondences[indices[i]];
        const EdgePlane* edge = static_cast<const EdgePlane*>(c.edge());

        //estraggo i vertici dagli edge
        const VertexPlane* v1 = static_cast<const VertexPlane*>(edge->vertex(0));
        const VertexPlane* v2 = static_cast<const VertexPlane*>(edge->vertex(1));

        //recupero i dati dei piani
        const AlignmentAlgorithmPlaneLinear::PointEstimateType& pi= v1->estimate();
        const AlignmentAlgorithmPlaneLinear::PointEstimateType& pj= v2->estimate();

        //recupeo le normali, mi servono per condizionare la parte rotazionale
        Vector3d ni;
        Vector3d nj;
        Vector4d coeffs_i=pi.toVector();
        Vector4d coeffs_j=pj.toVector();
        double di;
        double dj;

        ni=coeffs_i.head(3);
        nj=coeffs_j.head(3);

        di=coeffs_i(3);
        dj=coeffs_j(3);
        if(DEBUG)
        cerr << "[TRANSLATION JACOBIAN][ JACOBIAN IS: ]"<<endl;
        A2=(-nj.transpose()*R.transpose());
        cerr << A2<<endl;

        double ek;
        ek=dj+A2.transpose()*X.translation()-di;
        H2+=A2*A2.transpose();
        b2+= (A2.transpose()*ek);
        err += abs(ek*ek);

        if(DEBUG)
        cerr << "[TRANSLATIONAL Hessian for plane "<<i<<"]"<<endl<<H2<<endl;
        if(DEBUG)
        cerr << "[TRANSLATIONAL B for plane "<<i<<"]"<<endl<<b2<<endl;
    }
    if(DEBUG)
    cerr << "[FINAL][TRANSLATIONAL Hessian]"<<endl<<H2<<endl;
    if(DEBUG)
    cerr << "[FINAL][TRANSLATIONAL B]"<<endl<<b2<<endl;

    //solving the system
    Vector3d dt;
    LDLT<Matrix3d> ldlt2(H2);
    dt=ldlt2.solve(-b2); // using a LDLT factorizationldlt;
    if(DEBUG)
    cerr << "[FINAL][TRANSLATIONAL DT]"<<endl<<dt<<endl;


    X.translation()+=dt;
    transform = X;
    if(DEBUG)
    {
    cerr << "TRANSFORM found: " << endl<< endl<< endl;
    cerr << g2o::internal::toVectorMQT(X) << endl;;
    cerr << transform.matrix()<< endl;;
    }
    return true;
}


//3 because the minimal set is 3

RansacPlaneLinear::RansacPlaneLinear():  GeneralizedRansac<AlignmentAlgorithmPlaneLinear>(3)
{

}

}
