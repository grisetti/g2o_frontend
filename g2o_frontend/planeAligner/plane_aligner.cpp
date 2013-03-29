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
#include <cstdio>
#include <iomanip>

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

std::set<VertexPlane*> morte;

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
bool testRansac(typename RansacType::TransformType& result,CorrespondenceVector& correspondences,IndexVector& iv){

    RansacType ransac;

    //ransac.correspondenceValidators()=validators;
    ransac.setCorrespondences(correspondences);
    ransac.setMaxIterations(1000);
    ransac.setInlierErrorThreshold(1.);
    ransac.setInlierStopFraction(0.5);

    return ransac(result,iv);
}


int main(int argc, char**argv)
{
    hasToStop = false;
    string filename;
    string outfilename;
    CommandArgs arg;
    int theError;
    arg.param("o", outfilename, "otest.g2o", "output file name");
    arg.param("e",theError,5,"errore");
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

    int acc=0;

    OptimizableGraph::Vertex* _v=graph.vertex(0);
    _v->setFixed(true);
    VertexSE3* v=dynamic_cast<  VertexSE3*>(_v);

    //needed to check if the VertexSE3 iteration reach the end
    int lastid;
    lastid=0;
    //--------------------------------------------------------

    //main loop, for 100 vertifces
    while(v && acc<100)
    {
        acc++;
        //accedo ad ogni vertice
        OptimizableGraph::EdgeSet e = v->edges();
        //correspondence vector da riempire

        CorrespondenceVector mycorrVector;

        VertexSE3* vnext=new VertexSE3;
        VertexSE3* a=new VertexSE3;
        VertexSE3* sensorOffset=new VertexSE3;

        //di quei vertici accedo ad ogni edge
        for (HyperGraph::EdgeSet::iterator it = e.begin(); it!=e.end(); it++)
        {

            HyperGraph::Edge* _e = *it;
            //di quegli edge accedo solo a quelli SE3 per sapere su quale vertice spostarmi
            EdgeSE3 * eSE3 =dynamic_cast< EdgeSE3*>(_e);
            Isometry3d odometry;
            if(eSE3)
            {
                //accedo al vertice successivo sul quale andare
                a = dynamic_cast<  VertexSE3*>(eSE3->vertex(1));

                //VertexSE3* b = dynamic_cast<  VertexSE3*>(eSE3->vertex(0));
                odometry=eSE3->measurement();

                if(a->id()!=v->id() && a)
                {
                    cerr <<"V[" <<v->id() << "] -> V[" << a->id() <<"] "<< endl;


                    _v=graph.vertex(a->id());

                    //v è il nuovo vertice su cui mi devo spostare
                    vnext=dynamic_cast<VertexSE3*>(_v);

                    //edgeset per estrarre i piani del vertice successivo
                    OptimizableGraph::EdgeSet enext = vnext->edges();

                    //adesso mi serve cercare tutti i piani che sono legati all'edge su cui sto lavorando
                    for (HyperGraph::EdgeSet::iterator it = e.begin(); it!=e.end(); it++)
                    {
                        HyperGraph::Edge* _e = *it;
                        EdgeSE3PlaneSensorCalib * eSE3Calib = new EdgeSE3PlaneSensorCalib;
                        eSE3Calib   =dynamic_cast< EdgeSE3PlaneSensorCalib*>(_e);

                        Matrix3d info= Matrix3d::Identity();
                        info*=1000;
                        info(2,2)=10;



                        if(eSE3Calib)
                        {
                            eSE3Calib->setInformation(info);
                            VertexPlane* vplane = dynamic_cast< VertexPlane*>(eSE3Calib->vertex(1));
                            vplane->color=Vector3d(0,0,0);
                            eSE3Calib->color=Vector3d(0.1,0.1,0.1);

                            //aggiungo un vertice tra i due

                            if(vplane)
                            {
                                Plane3D plane = vplane->estimate();

                                //a questo punto mi serve sapere quali sono tutti i plane3D del vertice successivo iterando su enext

                                for (HyperGraph::EdgeSet::iterator itnext = enext.begin(); itnext!=enext.end(); itnext++)
                                {
                                    HyperGraph::Edge* _enext = *itnext;
                                    EdgeSE3PlaneSensorCalib * eSE3Calibnext =dynamic_cast< EdgeSE3PlaneSensorCalib*>(_enext);
                                    if(eSE3Calibnext)
                                    {
                                        //configuro l'information matrix anche all'edge successivo
                                        eSE3Calibnext->setInformation(info);
                                        VertexPlane* vplanenext = dynamic_cast< VertexPlane*>(eSE3Calibnext->vertex(1));
                                        sensorOffset = dynamic_cast< VertexSE3*>(eSE3Calibnext->vertex(2));

                                        VertexSE3* vparamToSet = dynamic_cast< VertexSE3*>(eSE3Calibnext->vertex(2));

                                        Isometry3d paramISO;
                                        paramISO.setIdentity();
                                        //Vector7d paramV;
                                        //paramV<<0,0,0,0.5000,-0.5000,0.5000,-0.5000;
                                        //vparamToSet->setFixed(true);
                                        //vparamToSet->setEstimate(g2o::internal::fromVectorQT(paramV));

                                        if(vplanenext)
                                        {
                                            Plane3D planenext = vplanenext->estimate();

                                            //porto il planenext nel frame di riferimento del primo

                                            Plane3D modifiedPlaneNext;
                                            modifiedPlaneNext=
                                                    sensorOffset->estimate().inverse()*
                                                    odometry.inverse()*
                                                    sensorOffset->estimate()*planenext.toVector();


                                            Vector4d diff = plane.toVector()-modifiedPlaneNext.toVector();
                                            //Vector4d diff = plane.toVector()-planenext.toVector();
                                            diff.head<3>() *= 10;
                                            double error = diff.squaredNorm();
                                            if(error<10) cerr <<"\t (P1) ";
                                            else cerr <<"\t [P1] ";
                                            cerr
                                                    <<setw(12)<<plane.toVector()[0] << " "
                                                   <<setw(12)<<plane.toVector()[1] << " "
                                                  <<setw(12)<<plane.toVector()[2] << " "
                                                 <<setw(12)<<plane.toVector()[3] << " "
                                                <<" [P2] "
                                               <<setw(12) << modifiedPlaneNext.toVector()[0] << " "
                                              <<setw(12)<<modifiedPlaneNext.toVector()[1] << " "
                                             <<setw(12)<<modifiedPlaneNext.toVector()[2] << " "
                                            <<setw(12)<<modifiedPlaneNext.toVector()[3] << "  "
                                            <<setw(20)<<" [ERR] "<< error<<endl;

                                            //time to create the correspondence vector
                                            if(error<10)
                                            {

                                                EdgePlane* eplane = new EdgePlane;
                                                eplane->setVertex(0,const_cast< VertexPlane*>(vplane));
                                                eplane->setVertex(1,const_cast< VertexPlane*>(vplanenext));

                                                Correspondence corr(eplane,error);

                                                mycorrVector.push_back(corr);

                                                morte.insert(vplane);
                                                morte.insert(vplanenext);
                                            }


                                        }
                                    }
                                }

                            }
                        }
                    }
                    Isometry3d tresult;
                    tresult.setIdentity();
                    IndexVector iv;

                    RansacPlaneLinear ransac;

                    //ransac.correspondenceValidators()=validators;
                    ransac.setCorrespondences(mycorrVector);
                    ransac.setMaxIterations(2000);
                    ransac.setInlierErrorThreshold(1.);
                    ransac.setInlierStopFraction(0.5);

                    ransac(tresult,iv);
                    std::vector<double> errorsVector=ransac.errors();

                    bool result = testRansac<PlaneMapping, RansacPlaneLinear, EdgePlane>(tresult, mycorrVector,iv);
                    cerr << "\t\t planes matching: "<<endl;

                    for (int i=0;i<iv.size();i++)
                    {
                        Correspondence tmpCorr=mycorrVector.at(iv.at(i));
                        VertexPlane *vp1 = new VertexPlane;
                        vp1=  dynamic_cast<VertexPlane*>(tmpCorr.edge()->vertex(0));
                        VertexPlane *vp2 = new VertexPlane;
                        vp2= dynamic_cast<VertexPlane*>(tmpCorr.edge()->vertex(1));

//                        Plane3D p1=vp1->estimate();
//                        Plane3D p2=vp2->estimate();

//                                                cerr << "id Plane1: "<<vp1->id()<<" "<< "id Plane2: "<<vp2->id()<<endl;
//                                                cerr << "Plane coeff: "<<p1.toVector()[0]<< " "<<p1.toVector()[1]<< " "<<p1.toVector()[2]<< " "<<p1.toVector()[3]<< " ||||"
//                                                     <<p2.toVector()[0]<< " "<<p2.toVector()[1]<< " "<<p2.toVector()[2]<< " "<<p2.toVector()[3]<< " @ "<<errorsVector.at(iv.at(i)) <<endl;
                    }

                    cerr << "adding planes to the output graph..."<<endl;

                    std::vector<int> ids;
                    std::set<VertexPlane *> murdered;

                    for (int i=0;i<iv.size();i++)
                    {
                        Correspondence tmpCorr=mycorrVector.at(iv.at(i));
                        VertexPlane *vp1 = dynamic_cast<VertexPlane*>(tmpCorr.edge()->vertex(0));
                        VertexPlane *vp2 = dynamic_cast<VertexPlane*>(tmpCorr.edge()->vertex(1));

                        if(vp1 && vp2)
                        {
                            cerr << "trying..."<<endl;
                            bool gottaGo=1;
                            for(int j=0;j<ids.size();j++)
                            {

                                if(ids.at(j)==vp1->id()) gottaGo=0;
                                if(ids.at(j)==vp2->id()) gottaGo=0;
                            }

                            if(gottaGo)
                            {
                                cerr << "trying to merge vertices... "<<vp2->id() << " is going to be merged on "<<vp1->id()<<" ...";

                                vp1->color=Vector3d(1,0,0);
                                vp2->color=Vector3d(1,0,0);

                                HyperGraph::EdgeSet theSet=vp1->edges();
                                for (HyperGraph::EdgeSet::iterator itnext = theSet.begin(); itnext!=theSet.end(); itnext++)
                                {
                                    HyperGraph::Edge* _enext = *itnext;
                                    EdgeSE3PlaneSensorCalib * eSE3Calib =dynamic_cast< EdgeSE3PlaneSensorCalib*>(_enext);
                                    if(eSE3Calib)
                                    {
                                       eSE3Calib->color=Vector3d(1,1,0);
                                    }
                                 }

                                theSet=vp2->edges();
                                for (HyperGraph::EdgeSet::iterator itnext = theSet.begin(); itnext!=theSet.end(); itnext++)
                                {
                                    HyperGraph::Edge* _enext = *itnext;
                                    EdgeSE3PlaneSensorCalib * eSE3Calib =dynamic_cast< EdgeSE3PlaneSensorCalib*>(_enext);
                                    if(eSE3Calib)
                                    {
                                       eSE3Calib->color=Vector3d(0,1,0);
                                    }
                                 }


                                bool res=graph.mergeVertices(vp1,vp2,0);
                                cerr << "done [ "<<res<<" ]!"<<endl;
                                murdered.insert(vp2);

                                ids.push_back(vp1->id());
                                ids.push_back(vp2->id());
                            }
                        }



                    }

//                    for(std::set<VertexPlane*>::iterator it=murdered.begin();it!=murdered.end();it++)
//                    {
//                        cerr << "Slaughtering the vertex "<<(*it)->id()<< " has "<< (*it)->edges().size() << endl;
//                        graph.removeVertex(*it);


//                    }



                }



            }

        }





        v=vnext;
        //check per determinare che non ci siano altri vertici
        if(lastid==v->id()) break;
        lastid=v->id();
    }

    int iteratore=100;
    cerr << "removing things..."<<endl;
    while(v!=0 && iteratore<2545)
    {

        OptimizableGraph::Vertex* _v=graph.vertex(iteratore);
        v=dynamic_cast<VertexSE3*>(_v);
        if(v!=0)
        {
            graph.removeVertex(v);
            //cerr << "vertex "<< iteratore << " is a SE3"<<endl;
        }
        else
            cerr << "vertex "<< iteratore << " is not an S3"<<endl;
        iteratore++;

    }

    //    for(int i=2546;i<=2839;i++)
    //    {
    //        VertexPlane* victim=dynamic_cast<VertexPlane*>(graph.vertex(i));
    //        if(victim)
    //        {
    //            cerr << "found a possible victim"<<endl;
    //            std::set<VertexPlane*>::iterator it;
    //            it=morte.find(victim);
    //            if(it==morte.end())
    //            {
    //                cerr << "\t victim has to be sacrified"<<endl;
    //                graph.removeVertex(victim);
    //            }
    //            else
    //                cerr << "\t victim will be saved"<<endl;

    //        }
    //    }

    cerr << "saving graph..."<<endl;
    ofstream oscani ("colorato.g2o");
    graph.save(oscani);



    exit(0);

}

