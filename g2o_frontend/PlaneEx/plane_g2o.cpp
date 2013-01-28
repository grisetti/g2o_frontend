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
#include "g2o_frontend/data/rgbd_image_data.h"
#include "g2o_frontend/thesis/RGBDData.h"
#include <Eigen/Geometry>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>

#define fx_d 5.9421434211923247e+02
#define fy_d 5.9104053696870778e+02
#define cx_d 3.3930780975300314e+02
#define cy_d 2.4273913761751615e+02
#define k1_d -2.6386489753128833e-01
#define k2_d 9.9966832163729757e-01
#define p1_d -7.6275862143610667e-04
#define p2_d 5.0350940090814270e-03
#define k3_d -1.3053628089976321e+00


using namespace std;
using namespace g2o;
using namespace Slam3dAddons;
using namespace cv;
RGBDData imageOriginal;

//PLANE
//***********************************************
struct plane
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    Vec3f color;
    plane()
    {

        this->color[0]=((float)rand())/RAND_MAX;
        this->color[1]=((float)rand())/RAND_MAX;
        this->color[2]=((float)rand())/RAND_MAX;
    }
};



//VOXELIZATION
//***********************************************

struct voxelAcc
{
    Vec3f acc;
    int i;

    voxelAcc()
    {
        acc[0]=acc[1]=acc[2]=0;
        i=0;
    };

    void add(Vec3f & v)
    {
        i++;
        acc+=v;
    };

    Vec3f average() const
    {
        float f=1./i;
        return acc*f;
    }
};

struct something
{
    int i[3];
    bool operator < (const something & s) const
    {
        if(i[0]<s.i[0]) return true;
        if(i[0]==s.i[0] && i[1]<s.i[1]) return true;
        if(i[1]==s.i[1] && i[2]<s.i[2]) return true;
        return false;
    }
};
//***********************************************

typedef std::map<something, voxelAcc> accumulatorMap;

//***********************************************


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


int main(int argc, char**argv){
    hasToStop = false;
    string filename;
    string outfilename;
    CommandArgs arg;
    float voxelSize;
    int minPlanePoints;
    int maxDist;
    int minAcc;
    arg.param("o", outfilename, "otest.g2o", "output file name");
    arg.param("voxelSize", voxelSize, 10, "grid voxel size");
    arg.param("minAcc", minAcc, 10, "minimum accurancy");
    arg.param("minPlanePoins", minPlanePoints, 500, "minimum point for a plane model to be considered as a valid plane");
    arg.param("maxDist", maxDist, 2000, "maximum valid distance of points");
    arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
    arg.parseArgs(argc, argv);

    // graph construction
    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmGaussNewton* solverGauss   = new OptimizationAlgorithmGaussNewton(blockSolver);
    SparseOptimizer * graph = new SparseOptimizer();
    graph->setAlgorithm(solverGauss);
    cout << "<Parsing .g2o file>"<<endl;
    graph->load(filename.c_str());
    cout << "<done>"<<endl;

    // sort the vertices based on the id
    std::vector<int> vertexIds(graph->vertices().size());
    int k=0;
    for (OptimizableGraph::VertexIDMap::iterator it=graph->vertices().begin(); it!= graph->vertices().end(); it ++){
        vertexIds[k++] = (it->first);
    }
    std::sort(vertexIds.begin(), vertexIds.end());



    int vertexNum = 0;
    cerr<< "found " << vertexIds.size() << " vertices " << endl;
    if (vertexIds.size())
        vertexNum = *vertexIds.rbegin() + 1;

    cerr << "loaded graph, now processing planes" << endl;
    signal(SIGINT, sigquit_handler);

    VertexSE3* vparam = new VertexSE3;
    vparam->setId(vertexNum++);
    graph->addVertex(vparam);

    //VERTEX_SE3:QUAT 2545  0 0 0  0.5000   -0.5000    0.5000   -0.5000
    //angle2quat(-pi/2, 0,-pi/2) //'ZYX'

    int test=0;
    for (size_t i=0; i<vertexIds.size()  && ! hasToStop; i++){ //process each id in this for

        OptimizableGraph::Vertex* _v=graph->vertex(vertexIds[i]);
        VertexSE3* v=dynamic_cast<VertexSE3*>(_v);
        if (!v) continue;
        OptimizableGraph::Data* d = v->userData();
        k = 0;

        test++;
        while(d ){
            Vector<Plane3D> stashOfPlanes;
            //RGBDImageData* imageData = dynamic_cast<RGBDImageData*>(d);
            RGBDData* imageData = dynamic_cast<RGBDData*>(d);
            d=d->next();
            if (imageData) {
                // extract plane
                const Parameter* pa = graph->parameters().getParameter(imageData->paramIndex());
                const ParameterCamera* camParam = dynamic_cast<const ParameterCamera*>(pa);

                if(camParam!=0) {
                    vparam->setEstimate(camParam->offset());
                }

                //cerr << "image found" << endl;
                //string base_name=imageData->baseFilename();
                //string intensity="_intensity.pgm";
                //string realPath= strcat(base_name,intensity);

                cout << "\t Processing "<<imageData->baseFilename()<<"\n";
                Mat *imageToProcess;
                imageToProcess = imageData->_depthImage; //no need to reproject image 'cause we've the depth registered
                std::vector<Vec3f> Cloud;

                int rows=imageToProcess->rows;
                int cols=imageToProcess->cols;

                unsigned short * dptra=imageToProcess->ptr<unsigned short>(0);;
                for(int i = 0; i < rows; i++)
                {

                    for(int j = 0; j < cols; j++)
                    {
                        unsigned short d = *dptra;
                        if(d != 0)
                        {

                            Vec3f cloudElem;
                            if(d<maxDist) //2000 is max distance
                            {
                                cloudElem[0]=(double)((double)j-(double)cx_d)*(double)d/(double)fx_d;
                                cloudElem[1]=(double)((double)i-(double)cy_d)*(double)d/(double)fy_d;
                                cloudElem[2]=d;

                                Cloud.push_back(cloudElem);

                            }
                        }
                        dptra++;
                    }
                }
                //cout << "\t\t Cloud has " << Cloud.size() << " points"<<endl;

                //**************************************************************
                //voxel
                //**************************************************************

                //cout << "\t\t starting voxelization"<<endl;

                accumulatorMap accMap;
                float res = voxelSize;
                float ires=1./res;
                std::vector<Vec3f> * theCloud=&Cloud;

                for(int i=0;i<(int)theCloud->size();i++)
                {
                    //cout << (*Cloud)[i][0];
                    something s;
                    res=(float)voxelSize;
                    ires=1./res;
                    s.i[0]=(int)(*theCloud)[i][0]*ires;
                    s.i[1]=(int)(*theCloud)[i][1]*ires;
                    s.i[2]=(int)(*theCloud)[i][2]*ires;

                    accumulatorMap::iterator it=accMap.find(s);
                    if(it==accMap.end())
                    {
                        voxelAcc vac;
                        vac.acc[0]=(*theCloud)[i][0];
                        vac.acc[1]=(*theCloud)[i][1];
                        vac.acc[2]=(*theCloud)[i][2];
                        vac.i=1;
                        accMap.insert(make_pair(s,vac));
                    }
                    else
                    {
                        voxelAcc & vac = it->second;
                        vac.add((*theCloud)[i]);
                    }

                }



                theCloud->clear();
                for(accumulatorMap::iterator it=accMap.begin();it!=accMap.end();it++ )
                {
                    voxelAcc &aMap=it->second;
                    Vec3f tmp = aMap.average();
                    theCloud->push_back(tmp);
                }
                //cout << "\t\t voxelized in "<<theCloud->size()<<" points"<<endl;
                //cout << "\t\t Voxelization finished..."<<endl;

                //**************************************************************
                //time to extract some planes!
                //**************************************************************

                //ESTRAZIONE DEI PIANI
                pcl::PointCloud<pcl::PointXYZ> pclCLOUD;

                //POPOLO LA CLOUD PCL
                for(int i=0;i<theCloud->size();i++)
                {
                    pcl::PointXYZ tmpXYZ((*theCloud)[i][0],(*theCloud)[i][1],(*theCloud)[i][2]);
                    pclCLOUD.push_back(tmpXYZ);
                }


                //--------------------------------------------------------------------------------
                //DICHIARAZIONI PER L'ESTRAZIONE DEI TANTI PIANI
                const int numpoint=minPlanePoints; //numero minimo di punti affinchè l'oggetto venga considerato un piano
                const int tresh=minAcc;     //distanza minima tra i punti
                int planesExtracted=0;
                pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                //------------// Create the segmentation object
                pcl::SACSegmentation<pcl::PointXYZ> seg;
                //------------// Optional
                seg.setOptimizeCoefficients (true);
                //------------// Mandatory
                seg.setModelType (pcl::SACMODEL_PLANE);
                seg.setMethodType (pcl::SAC_RANSAC);
                seg.setDistanceThreshold(tresh);
                //--------------------------------------------------------------------------------
                //Procedure
                seg.setInputCloud (pclCLOUD.makeShared ());
                seg.segment (*inliers, *coefficients);
                theCloud->clear();

                while(
                      (inliers->indices.size()!=0) &
                      (inliers->indices.size()>numpoint) )
                {


                    planesExtracted++;
                    plane tmpCLOUD;

                    //Istanzio un plane, lo popolo con i punti degli inliers rilevati in precedenza
                    const int repj=1000; //metri
                    Vec3f com_tmp;
                    com_tmp[0]=0;com_tmp[1]=0;com_tmp[2]=0;

                    for (size_t i = 0; i < inliers->indices.size (); ++i)
                    {
                        tmpCLOUD.cloud.push_back(pcl::PointXYZ(
                                                     pclCLOUD[inliers->indices[i]].x/repj,
                                                     pclCLOUD[inliers->indices[i]].y/repj,
                                                     pclCLOUD[inliers->indices[i]].z/repj));

                        com_tmp[0]+= pclCLOUD[inliers->indices[i]].x/repj;
                        com_tmp[1]+= pclCLOUD[inliers->indices[i]].y/repj;
                        com_tmp[2]+= pclCLOUD[inliers->indices[i]].z/repj;
                    }

                    com_tmp[0]=com_tmp[0]/inliers->indices.size ();
                    com_tmp[1]=com_tmp[1]/inliers->indices.size ();
                    com_tmp[2]=com_tmp[2]/inliers->indices.size ();

                    /*
                    cout << "\t\t coeffiecienti estratti a:("<<
                            coefficients->values[0]<<") b:("<<
                            coefficients->values[1]<<") c:("<<
                            coefficients->values[2]<<") d:("<<
                            coefficients->values[3]<<")"<<endl;
                    */
                    //creo un oggetto g2o::Plane3d
                    Plane3D p;
                    Vector4d planeCoeff;
                    planeCoeff[0]=coefficients->values[0];
                    planeCoeff[1]=coefficients->values[1];
                    planeCoeff[2]=coefficients->values[2];
                    planeCoeff[3]=(coefficients->values[3]/1000); //in meters please
                    cout << "COEFF: "<< planeCoeff[0] << " "<< planeCoeff[1]<<" "<< planeCoeff[2]<<" "<< planeCoeff[3]<<endl;
                    p.fromVector(planeCoeff);
                    stashOfPlanes.push_back(p);
                    tmpCLOUD.cloud.clear();

                    //Elimino i punti appena rilevati dalla nuvola inizale
                    //****************************************************

                    pcl::PointCloud<pcl::PointXYZ> temporanyCLOUD;
                    for(int k=0;k<pclCLOUD.size();k++)
                    {
                        bool valid=1;
                        for (size_t j = 0; j < inliers->indices.size (); ++j)
                        {
                            if(inliers->indices[j]!=k) valid=1;
                            else
                            {
                                valid=0;
                                break;
                            }
                        }
                        if(valid)
                        {
                            temporanyCLOUD.push_back(pclCLOUD[k]);
                        }
                    }
                    //****************************************************
                    pclCLOUD.clear();

                    for(int k=0;k<temporanyCLOUD.size();k++)
                    {
                        pclCLOUD.push_back(temporanyCLOUD[k]);
                    }

                    seg.setInputCloud (pclCLOUD.makeShared ());
                    seg.segment (*inliers, *coefficients);
                }



                //Plane3D p; // extracted plane

                /*
                if (!camParam ){
                    camParam = param->offset();
                    vparam->setEstimate(camParam->offset());

                }
                */

                Isometry3d sensorAndRobot = v->estimate() * vparam->estimate();
                for(int i=0;i<stashOfPlanes.size();i++)
                {
                    VertexPlane* vplane = new VertexPlane();
                    vplane->setId(vertexNum);
                    vplane->setEstimate(sensorAndRobot*stashOfPlanes[i]);
                    graph->addVertex(vplane);


                    EdgeSE3PlaneSensorCalib* edge= new EdgeSE3PlaneSensorCalib();
                    edge->setVertex(0,v);
                    edge->setVertex(1,vplane);
                    edge->setVertex(2,vparam);
                    edge->setMeasurement(stashOfPlanes[i]);
                    edge->setInformation(Matrix3d::Identity());
                    graph->addEdge(edge);
                    vertexNum++;
                }
            }

        }

    }

    cerr << endl;

    cerr << "saving.... " << endl;
    ofstream os (outfilename.c_str());
    graph->save(os);
    cerr << endl;
}
