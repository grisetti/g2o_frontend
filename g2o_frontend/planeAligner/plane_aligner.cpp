#include <signal.h>

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
#include <Eigen/Geometry>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include "g2o/types/slam3d_addons/vertex_plane.h"
#include "g2o/types/slam3d_addons/edge_plane.h"
#include "g2o/types/slam3d_addons/edge_se3_plane_calib.h"
#include "g2o/core/hyper_graph.h"
#include "g2o_frontend/ransac/ransac.h"
#include "g2o_frontend/ransac/alignment_plane_linear.h"
#include "g2o_frontend/basemath/bm_se2.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include "g2o_frontend/ransac/alignment_plane_linear.h"
#include "g2o_frontend/ransac/ransac.h"

using namespace std;
using namespace g2o;
using namespace Slam3dAddons;
using namespace cv;
using namespace Eigen;
using namespace g2o_frontend;

VertexPlane myfancyVertex;
RGBDData imageOriginal;

struct planeAndVertex
{
    Plane3D plane;
    VertexSE3 vertex;
    VertexPlane vplane;
};

struct planeCorrespondence
{
    int p1;
    int p2;
    double error;
};

std::vector<planeAndVertex> container1;
std::vector<planeAndVertex> container2;
std::vector<planeCorrespondence> corrs;


volatile bool hasToStop;
void sigquit_handler(int sig)
{
    if (sig == SIGINT) {
        hasToStop = 1;
        static int cnt = 0;
        if (cnt++ == 2) {
            cerr << __PRETTY_FUNCTION__ << " forcing exit" << endl;
            exit(1);
        }
    }
}
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

typedef std::vector<Correspondence> CorrespondenceVector;

template <typename MappingType, typename RansacType, typename EdgeCorrespondenceType>
bool testRansac(typename RansacType::TransformType& result,CorrespondenceVector correspondences){

    typedef typename RansacType::AlignerType AlignerType;
    typedef typename RansacType::PointVertexType PointVertexType;
    typedef typename RansacType::PointEstimateType PointEstimateType;
    typedef typename RansacType::TransformType TransformType;
    typedef typename MappingType::VectorType VectorType;

    RansacType ransac;
    //ransac.correspondenceValidators()=validators;
    ransac.setCorrespondences(correspondences);
    ransac.setMaxIterations(1000);
    ransac.setInlierErrorThreshold(1.);
    ransac.setInlierStopFraction(0.5);
    return ransac(result);
}


int main(int argc, char**argv)
{
    hasToStop = false;
    string filename;
    string outfilename;
    CommandArgs arg;
    arg.param("o", outfilename, "otest.g2o", "output file name");
    arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
    arg.parseArgs(argc, argv);

    //Reading graph
    //**************************************************************************************************************************************
    OptimizableGraph graph;
    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmGaussNewton* solverGauss   = new OptimizationAlgorithmGaussNewton(blockSolver);

    SparseOptimizer outgraph;
    outgraph.setAlgorithm(solverGauss);

    cout << "<Parsing .g2o file>"<<endl;
    graph.load(filename.c_str());
    cout << "<done>"<<endl;

    cerr << "loaded graph, now processing planes" << endl;
    signal(SIGINT, sigquit_handler);

    //**************************************************************************************************************************************


    cerr <<"entering cycle..."<<endl;
    Isometry3d transformation2to1;
    transformation2to1.setIdentity();
    Isometry3d cameraOffset;
    cameraOffset.setIdentity();




    int i=1;

    OptimizableGraph::Vertex* _v=graph.vertex(i);
    VertexSE3* v=dynamic_cast<VertexSE3*>(_v);

    if(v)
    {
        cerr << "Processing [1] vertex"<<v->id()<<endl;
        OptimizableGraph::EdgeSet e = v->edges();

        int vertexHasPlanes=0;

        cerr << "*************** PLANES ***************"<<endl;
        for (HyperGraph::EdgeSet::iterator it = e.begin(); it!=e.end(); it++)
        {

            HyperGraph::Edge* _e = *it;
            const EdgeSE3PlaneSensorCalib* ePlane =dynamic_cast<const EdgeSE3PlaneSensorCalib*>(_e);
            if(ePlane)
            {
                const VertexSE3* offset = dynamic_cast<const  VertexSE3*>(ePlane->vertex(2));
                cerr <<"Sensor offset transformation " <<endl<<offset->estimate().matrix()<<endl;
                cameraOffset=offset->estimate();

                vertexHasPlanes++;
                const Plane3D thePlane= ePlane->measurement();
                const VertexPlane* aaa = dynamic_cast<const  VertexPlane*>(ePlane->vertex(1));

                VertexPlane tmp=*aaa;

                planeAndVertex pav;
                pav.plane=thePlane;

                pav.vplane.setEstimate(tmp.estimate());
                pav.vplane.setId(tmp.id());

                container1.push_back(pav);

                cerr <<"adding EDGEPLANECALIB to graph " <<endl;
                outgraph.addEdge(const_cast<EdgeSE3PlaneSensorCalib*>(ePlane));
                cerr <<"vertex 1 has a plane " <<endl<<thePlane.toVector()<<endl;


            }
        }

        cerr << "*************** Odometry Transformation Transformation ***************"<<endl;
        for (HyperGraph::EdgeSet::iterator it = e.begin(); it!=e.end(); it++)
        {

            HyperGraph::Edge* _e = *it;
            const EdgeSE3 * ese3 =dynamic_cast<const EdgeSE3*>(_e);
            if(ese3)
            {
                const VertexSE3* vFrom = dynamic_cast<const  VertexSE3*>(ese3->vertex(0));
                const VertexSE3* vTo = dynamic_cast<const  VertexSE3*>(ese3->vertex(1));


                cerr <<"Transformation FROM ID "<< vFrom->id()<< " TO ID "<<vTo->id()<<endl;
                cerr <<"Transformation Matrix is "<<endl<<ese3->measurement().matrix()<<endl;
                transformation2to1=ese3->measurement();

                if(vTo->id()==2)
                {
                    VertexSE3* vFromToSave=const_cast<VertexSE3*>(vFrom);
                    VertexSE3* vToToSave=const_cast<VertexSE3*>(vTo);
                    EdgeSE3* odomFrame=const_cast<EdgeSE3*>(ese3);

                    outgraph.addVertex(vFromToSave);
                    outgraph.addVertex(vToToSave);
                    outgraph.addEdge(odomFrame);
                }
            }
        }


    }

    i=2;
    _v=graph.vertex(i);
    v=dynamic_cast<VertexSE3*>(_v);
    if(v)
    {
        cerr << "Processing [2] vertex"<<v->id()<<endl;
        OptimizableGraph::EdgeSet e = v->edges();

        int vertexHasPlanes=0;

        cerr << "*************** PLANES ***************"<<endl;
        for (HyperGraph::EdgeSet::iterator it = e.begin(); it!=e.end(); it++)
        {

            HyperGraph::Edge* _e = *it;
            const EdgeSE3PlaneSensorCalib* ePlane =dynamic_cast<const EdgeSE3PlaneSensorCalib*>(_e);
            if(ePlane)
            {
                vertexHasPlanes++;
                const Plane3D thePlane= ePlane->measurement();
                const VertexPlane* aaa = dynamic_cast<const  VertexPlane*>(ePlane->vertex(1));

                cerr <<"adding EDGEPLANECALIB to graph " <<endl;
                outgraph.addEdge(const_cast<EdgeSE3PlaneSensorCalib*>(ePlane));

                cerr <<"vertex 2 has a plane " <<endl<<thePlane.toVector()<<endl;


                Isometry3d tmp = cameraOffset.inverse()*transformation2to1.inverse()*cameraOffset;
                Plane3D tst = tmp*thePlane;
                planeAndVertex pav;
                pav.plane=tst;
                pav.vplane.setEstimate(aaa->estimate());
                pav.vplane.setId(aaa->id());

                container2.push_back(pav);


            }
        }


    }

    //FINE LETTURA DELLE COSE DAL FILE G2O PASSATO IN INGRESSO, TEST DI SCRITTURA DI UN GENERICO FILE G2O
    cerr << "saving graph..."<<endl;
    ofstream oscani ("cani.g2o");
    outgraph.save(oscani);
    cerr << "graph saved!"<<endl;
    cerr << "Now a test!"<<endl;

    cerr << "Container 1 has "<<container1.size()<< " things"<<endl;


    for(int i=0;i<container1.size();i++)
    {
        Vector4d c=container1[i].plane.toVector();
        cerr << c[0] <<" "<<c[1]<<" "<<c[2]<<" "<<c[3]<<endl;
    }

    cerr << "Container 2 has "<<container2.size()<< " things"<<endl;

    for(int i=0;i<container2.size();i++)
    {
        Vector4d c=container2.at(i).plane.toVector();
        cerr << c[0] <<" "<<c[1]<<" "<<c[2]<<" "<<c[3]<<endl;
    }

    cerr << "Finding correspondences..."<<endl;

    for(int i=0;i<container1.size();i++)
    {
        Vector4d p1_coeff=container1[i].plane.toVector();
        planeCorrespondence mycorr;
        mycorr.error=999;
        mycorr.p1=-1;
        mycorr.p2=-1;
        for(int k=0;k<container2.size();k++)
        {
            Vector4d p2_coeff=container2[k].plane.toVector();
            double diff1=   abs(
                        p1_coeff[0]-p2_coeff[0]+
                        p1_coeff[1]-p2_coeff[1]+
                        p1_coeff[2]-p2_coeff[2]);

            double diff2=   abs(p1_coeff[3]-p2_coeff[3]);

            Vector4d diffff = p1_coeff-p2_coeff;
            diffff.head<3>() *= 10;
            double erore = diffff.squaredNorm();

            cerr << "Differences between frame 1 plane "<<i<<" and frame 2 plane \t"<<k<<" is "<< erore <<endl;
            //if per prendere solo le migliori, le voglio prendere tutte però
//            if(mycorr.error>(diff1+diff2))
//            {
//                mycorr.p1=i;
//                mycorr.p2=k;
//                mycorr.error=erore;

//            }
            mycorr.p1=i;
            mycorr.p2=k;
            mycorr.error=erore;
            corrs.push_back(mycorr);
        }
        //corrs.push_back(mycorr);
        cerr <<endl;
    }

    //vettore delle corrispondenze da passare a ransac
    g2o_frontend::CorrespondenceVector correspondences;
    cerr << "Corrispondenze trovate e i loro errori:"<<endl;


//    for (int i=0;i<container1.size();i++)
//    {

//        cerr <<"[1] (" <<i<<") Vertex id "<<container1.at(i).vplane.id()<<endl;
//    }

//    cerr << endl;
//    for (int i=0;i<container1.size();i++)
//    {

//        cerr <<"[2] ("<< i<<") Vertex id "<<container2.at(i).vplane.id()<<endl;
//    }


    for (int i=0;i<corrs.size();i++)
    {
        cerr << corrs.at(i).p1 << " " << corrs.at(i).p2 << " "<< corrs.at(i).error<<endl;
        EdgePlane* eplane = new EdgePlane;

        cerr <<     graph.vertex(container1.at(corrs.at(i).p1).vplane.id())->id()
             <<     " <=> "
             <<     graph.vertex(container2.at(corrs.at(i).p2).vplane.id())->id()
             <<endl;
//        int v1id=graph.vertex(container1.at(corrs.at(i).p1).vplane.id())->id();
//        int v2id=graph.vertex(container2.at(corrs.at(i).p2).vplane.id())->id();

        VertexPlane* vp1 = dynamic_cast<VertexPlane*>(graph.vertex(container1.at(corrs.at(i).p1).vplane.id()));
        VertexPlane* vp2 = dynamic_cast<VertexPlane*>(graph.vertex(container2.at(corrs.at(i).p2).vplane.id()));


        eplane->setVertex(0,vp1);
        eplane->setVertex(1,vp2);

        correspondences.push_back(eplane);
    }


    cerr << "Debug time:"<<endl;
    cerr << "correspondence vector has "<< correspondences.size()<< " edges"<<endl;

    cerr << "Calling RANSAC..."<<endl;

    Isometry3d tresult;
    tresult.setIdentity();

    bool result = testRansac<PlaneMapping, RansacPlaneLinear, EdgePlane>(tresult, correspondences);
    cerr << "Resutl is "<<result<<endl;
    cerr << "Result transform is "<<endl<<tresult.matrix()<<endl;
    cerr << "Result transform vector is "<< endl<< g2o::internal::toVectorMQT(tresult)<<endl;
    exit(0);

}
