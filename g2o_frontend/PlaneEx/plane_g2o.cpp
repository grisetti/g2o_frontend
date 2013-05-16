#include <signal.h>
#include <ncurses.h>
#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"
#include "g2o/types/slam3d/types_slam3d.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"


#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
#include "g2o_frontend/sensor_data/rgbd_data.h"
#include <Eigen/Geometry>

#include "extractionUtils.h"


using namespace std;
using namespace g2o;
using namespace Slam3dAddons;
using namespace cv;
using namespace Eigen;




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

    //COMMAND ARGS THINGS
    float voxelSize;
    int minPlanePoints;
    int maxDist;
    int minAcc;
    arg.param("o", outfilename, "otest.g2o", "output file name");
    arg.param("voxelSize", voxelSize, 10, "grid voxel size");
    arg.param("minAcc", minAcc, 20, "minimum accurancy");
    arg.param("minPlanePoins", minPlanePoints, 1000, "minimum point for a plane model to be considered as a valid plane");
    arg.param("maxDist", maxDist, 2000, "maximum valid distance of points");
    arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
    arg.parseArgs(argc, argv);
    //----------------------------------------------------------------------------------------------------------------------



    //GLOBALS
    //==========================================================================
    OptimizableGraph * graph = new OptimizableGraph();
    const Mat *imageToProcess;
    VertexSE3* vparam;
    //==========================================================================


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

    if (vertexIds.size()){
        vertexNum = *vertexIds.rbegin() + 1;
    }


    cerr << "loaded graph, now processing planes" << endl;
    signal(SIGINT, sigquit_handler);


    //paramV<<0,0,0,0.5000,-0.5000,0.5000,-0.5000;
    //at begin we initialize the parameter of the camera to the identity transformation, later we'll get from the graph a initial transformation
    Vector7d paramV;
    paramV<<0,0,0,0,0,0,0;

    vparam= new VertexSE3; //vparam is the vertexSE3 of the parameter of the camera
    vparam->setFixed(true);
    vparam->setEstimate(g2o::internal::fromVectorQT(paramV));
    vparam->setId(vertexNum++);
    graph->addVertex(vparam);

    //VERTEX_SE3:QUAT 2545  0 0 0  0.5000   -0.5000    0.5000   -0.5000
    //angle2quat(-pi/2, 0,-pi/2) //'ZYX'


    //process each vertex
    for (size_t i=0; i<vertexIds.size()  && ! hasToStop; i++)
    {
        //-------------------------------------------------------------
        OptimizableGraph::Vertex* _v=graph->vertex(vertexIds[i]);
        VertexSE3* v=dynamic_cast<VertexSE3*>(_v);
        if (!v) continue;
        // v is a vertex SE3
        //-------------------------------------------------------------

        //-------------------------------------------------------------
        //accessing the userdata
        OptimizableGraph::Data* d = v->userData();
        //has userdata
        while(d){
            Vector<Plane3D> stashOfPlanes;
            RGBDData* imageData = dynamic_cast<RGBDData*>(d);
            d=d->next();
            //userdata is RGBDData !!IMPORTANT!!
            if (imageData) {

                //get camera paramter
                const Parameter* parameter = graph->parameters().getParameter(imageData->paramIndex());
                //casting it
                const ParameterCamera* camParameter = dynamic_cast<const ParameterCamera*>(parameter);
                //if exists using it as the offset parameter for the kinect camera
                if(camParameter!=0) {
                    vparam->setEstimate(camParameter->offset());
                }

                cout << "\t Processing "<<imageData->baseFilename()<<"\n";
                imageData->update();
                imageToProcess = imageData->depthImage(); //no need to reproject image, we've the depth registered
                std::vector<Vec3f> Cloud;

                create_cloud(Cloud,*imageToProcess,maxDist);
                cout << "\t\t Cloud has " << Cloud.size() << " points"<<endl;

                //**************************************************************
                //voxel
                //**************************************************************

                cout << "\t\t starting voxelization"<<endl;
                voxelize(Cloud,voxelSize);

//                accumulatorMap accMap;
//                float res = voxelSize;
//                float ires=1./res;
//                std::vector<Vec3f> * theCloud=&Cloud;

//                for(int i=0;i<(int)theCloud->size();i++)
//                {
//                    //cout << (*Cloud)[i][0];
//                    something s;
//                    res=(float)voxelSize;
//                    ires=1./res;
//                    s.i[0]=(int)(*theCloud)[i][0]*ires;
//                    s.i[1]=(int)(*theCloud)[i][1]*ires;
//                    s.i[2]=(int)(*theCloud)[i][2]*ires;

//                    accumulatorMap::iterator it=accMap.find(s);
//                    if(it==accMap.end())
//                    {
//                        voxelAcc vac;
//                        vac.acc[0]=(*theCloud)[i][0];
//                        vac.acc[1]=(*theCloud)[i][1];
//                        vac.acc[2]=(*theCloud)[i][2];
//                        vac.i=1;
//                        accMap.insert(make_pair(s,vac));
//                    }
//                    else
//                    {
//                        voxelAcc & vac = it->second;
//                        vac.add((*theCloud)[i]);
//                    }

//                }



//                theCloud->clear();
//                for(accumulatorMap::iterator it=accMap.begin();it!=accMap.end();it++ )
//                {
//                    voxelAcc &aMap=it->second;
//                    Vec3f tmp = aMap.average();
//                    theCloud->push_back(tmp);
//                }
//                cout << "\t\t voxelized in "<<theCloud->size()<<" points"<<endl;
//                cout << "\t\t Voxelization finished..."<<endl;

                //**************************************************************
                //time to extract some planes!
                //**************************************************************

                //ESTRAZIONE DEI PIANI


                pcl::PointCloud<pcl::PointXYZ> pclCLOUD;

                std::vector<Vec3f> * theCloud = &Cloud;

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

                    Vector3d piano(coefficients->values[0],coefficients->values[1],coefficients->values[2]);
                    Vector3d mass(com_tmp[0],com_tmp[1],com_tmp[2]);
                    float facingDirection=piano.dot(mass);
                    //cout << "FACING HAS VALUE OF "<<facingDirection<<endl;
                    //cout << mass<<endl<<endl;
                    //cout << piano<<endl;
                    double a=coefficients->values[0];
                    double b=coefficients->values[1];
                    double c=coefficients->values[2];
                    double d=coefficients->values[3];

                    if(facingDirection>0 && 0)
                    {
                        cout << "la normale punta in direzione opposta al kinect, va rimappata!"<<endl;

                        a=-a;
                        b=-b;
                        c=-c;
                        d=-d;
                        cout <<" D "<<d<<endl;
                    }

                    planeCoeff[0]=a;
                    planeCoeff[1]=b;
                    planeCoeff[2]=c;
                    planeCoeff[3]=(double)(d/1000.0f); //in meters please
                    cout << "VALUE OF D "<<planeCoeff[3]<<endl;

                    //i piani vanno rimappati nel frame giusto del robot
                    //                    Eigen::Quaternion<double> quat(0.5000,-0.5000,0.5000,-0.5000);
                    //                    Eigen::Isometry3d cose(quat);
                    //                    Plane3D myPlane;

                    //                    myPlane.fromVector(Vector4d(planeCoeff[0],planeCoeff[1],planeCoeff[2],planeCoeff[3]));
                    //                    myPlane=cose*myPlane;

                    //                    cout << "@COEFF: "<< planeCoeff[0] << " "<< planeCoeff[1]<<" "<< planeCoeff[2]<<" "<< planeCoeff[3]<<endl;
                    //                    planeCoeff=myPlane.toVector();
                    //                    cout << "#COEFF: "<< planeCoeff[0] << " "<< planeCoeff[1]<<" "<< planeCoeff[2]<<" "<< planeCoeff[3]<<endl;

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

                imageData->release();
                Isometry3d sensorAndRobot = v->estimate()*vparam->estimate();
                Isometry3d tmp=v->estimate();
                cout << "VERTEX ESTIMATE: "<<g2o::internal::toVectorMQT(tmp).transpose()<<endl;
                for(int i=0;i<stashOfPlanes.size();i++)
                {
                    VertexPlane* vplane = new VertexPlane();
                    vplane->setId(vertexNum);
                    vplane->setEstimate(sensorAndRobot*stashOfPlanes[i]);
                    cout << stashOfPlanes[i].coeffs().transpose()<< " BECAME "<<vplane->estimate().coeffs().transpose()<<endl;
                    vplane->color=Vector3d(0,0,0);
                    graph->addVertex(vplane);


                    EdgeSE3PlaneSensorCalib* edge= new EdgeSE3PlaneSensorCalib();
                    edge->setVertex(0,v);
                    edge->setVertex(1,vplane);
                    edge->setVertex(2,vparam);
                    edge->setMeasurement(stashOfPlanes[i]);
                    edge->setInformation(Matrix3d::Identity());
                    edge->color=Vector3d(1,0.8,0.8);
                    graph->addEdge(edge);
                    edge->computeError();
                    //cerr << "edge.chi2()=" << edge->chi2() << endl;
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
