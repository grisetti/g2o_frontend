#ifndef _MAL_GRAPHUTILS
#define _MAL_GRAPHUTILS


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

#include "ransacDeclarations.h"
#include "printHelper.h"

using namespace Eigen;
using namespace g2o;
using namespace Slam3dAddons;

void getCalibPlanes(VertexSE3* v,vector<Plane3D>* container,Vector3d color,Matrix3d &info)
{
    OptimizableGraph::EdgeSet edges  = v->edges();

    for (HyperGraph::EdgeSet::iterator it = edges.begin(); it!=edges.end(); it++)
    {
        HyperGraph::Edge* _e = *it;
        EdgeSE3PlaneSensorCalib * eSE3Calib = new EdgeSE3PlaneSensorCalib;
        eSE3Calib   =dynamic_cast< EdgeSE3PlaneSensorCalib*>(_e);

        if(eSE3Calib)
        {
            eSE3Calib->setInformation(info);
            VertexPlane* vplane=new VertexPlane;
            vplane->setEstimate(eSE3Calib->measurement());
            eSE3Calib->color=color;
            //outgraph.addEdge(eSE3Calib);
            if(vplane)
            {
                vplane->color=color;
                Plane3D plane = vplane->estimate();
                container->push_back(plane);
            }
        }
    }
}


double computeError(Plane3D &p1,Plane3D &p2)
{
    Vector4d diff = p1.toVector()-p2.toVector();
    diff.head<3>() *= 10;
    double Error=diff.squaredNorm();
    return Error;
}

void compute_Correspondance_Vector(vector<Plane3D> &c1,
                                   vector<Plane3D> &c2,
                                   vector<Plane3D> &c2R,
                                   CorrespondenceVector &correspondanceVector)
{
    // C1 plane container 1
    // C2 plane container 2
    // C2R plane container 2

    for(int i=0;i<c1.size();i++)
    {
        Plane3D p1=c1.at(i);

        for(int j=0;j<c2.size();j++)
        {

            Plane3D p2=c2.at(j);
            double error = computeError(p1,p2);

            //DEBUG INFO ----
            printPlaneCoeffsAsRow(p1);
            cout << " <> ";
            printPlaneCoeffsAsRow(p2);
            cout <<" ["<<error<<"]"<<endl;
            //DEBUG INFO ----

            //FILLING CORRESPONDANCE VECTOR
            EdgePlane* eplane = new EdgePlane;

            VertexPlane* vPlane1=new VertexPlane;
            VertexPlane* vPlane2=new VertexPlane;

            vPlane1->setEstimate(p1);
            vPlane2->setEstimate(c2R.at(j));

            eplane->setVertex(0,vPlane1);
            eplane->setVertex(1,vPlane2);
            g2o_frontend::Correspondence corr(eplane,error);
            if(error<1)
                correspondanceVector.push_back(corr);

        }
        cout << endl;
    }
}

void executeRansac(CorrespondenceVector &correspondanceVector,
                   std::vector<int> &Indeces,
                   Isometry3d &transform,
                   int iterations,
                   float inliersThreshold,
                   float inliersStop)
{

    transform.setIdentity();

    g2o_frontend::RansacPlaneLinear ransac;
    ransac.setCorrespondences(correspondanceVector);
    ransac.setMaxIterations(iterations);
    ransac.setInlierErrorThreshold(inliersThreshold);
    ransac.setInlierStopFraction(inliersStop);
    ransac(transform,Indeces);
}

#endif
