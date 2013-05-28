// kinect
#include "SensorHandlerRGBDCamera.h"
// laser
#include "SensorHandlerLaserRobot.h"
// imu
#include "SensorHandlerImu.h"
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.hpp>
#include <g2o/core/hyper_graph.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o_frontend/sensor_data/priority_synchronous_data_queue.h>
#include <g2o/types/slam3d/isometry3d_mappings.h>
#include <g2o_frontend/sensor_data/sensor_data.h>

#include <g2o_frontend/pwn2/depthimage.h>
#include <g2o_frontend/pwn2/depthimageconverter.h>
#include <g2o_frontend/pwn2/pinholepointprojector.h>
#include <g2o_frontend/pwn2/correspondencefinder.h>
#include <g2o_frontend/pwn2/statsfinder.h>
#include <g2o_frontend/pwn2/aligner.h>
#include <g2o_frontend/pwn2/linearizer.h>
#include "g2o_frontend/pwn_mapper/pwn_mapper_controller.h"
#include "g2o_frontend/pwn_viewer/drawable_frame.h"

#include <g2o_frontend/traversability/traversability_analyzer.h>

//HAKKE
#include <g2o_frontend/sensor_data/rgbd_data.h>
#include <g2o_frontend/sensor_data/imu_data.h>


#include <g2o/stuff/command_args.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <fstream>
#include <pthread.h>


using namespace g2o;
using namespace pwn;

std::deque<OptimizableGraph::Vertex*> vertecesQueue;

#define conditionalPrint(x)			\
  if(verbose>=x) cerr

bool extractRelativePrior(Eigen::Isometry3f& priorMean, Matrix6f& priorInfo, VertexSE3* referenceVertex, VertexSE3* currentVertex);
bool extractAbsolutePrior(Eigen::Isometry3f& priorMean, Matrix6f& priorInfo, VertexSE3* currentVertex);

#define annoyingLevel 2
#define defaultLevel  1 

/** ROS stuff **/
PrioritySynchronousDataQueue _queue;
ros::NodeHandle* nptr;
tf::TransformListener* tfListener;


/** Sensors stuff **/
// kinect stuff
SensorRGBDCamera* kinect = 0 ;
SensorHandlerRGBDCamera* shKinect = 0;

// laser stuff
SensorLaserRobot* laser = 0 ;
SensorHandlerLaserRobot* shLaser = 0;

// imu stuff
SensorImu* imu = 0;
SensorHandlerImu* shImu = 0;

/** parameters stuff **/
// filename of the g2o to save
string filename;
// thresholds
Eigen::Vector2d minDistances(0.1, 0.1); // minimum translational and angular distances between frames
// time after the creation of a vertex, when we put data in it
double vertexTimeWindow = 1./30.;
// minimum time when we save something even if the robot does not move
double minTime = 1;
// delay before doing the writing
double initialDelay = 3;
// verbose level
int verbose;

g2o::OptimizableGraph* graph = new g2o::OptimizableGraph();

Eigen::Isometry3d fromStampedTransform(const tf::StampedTransform& transform) {
  geometry_msgs::TransformStamped msg;
  tf::transformStampedTFToMsg(transform, msg);
  Eigen::Isometry3d t;
  Eigen::Quaterniond q(msg.transform.rotation.w, 
		       msg.transform.rotation.x, 
		       msg.transform.rotation.y, 
		       msg.transform.rotation.z);
  t.linear()=q.matrix();
  t.translation() = Eigen::Vector3d(msg.transform.translation.x,
				    msg.transform.translation.y,
				    msg.transform.translation.z);
  return t;
}

Eigen::Vector2d isometry2distance(const Eigen::Isometry3d& t) {
  return Eigen::Vector2d(t.translation().norm(),
			 fabs(Eigen::AngleAxisd(t.rotation()).angle()));
}

bool readAndProcess(Frame *frame, int &imageRows, int &imageCols, std::string fname, int reduction, Eigen::Isometry3f sensorOffset, 
		    DepthImageConverter &converter, TraversabilityAnalyzer &traversabilityAnalyzer) {
  DepthImage depthImage;
  if(!depthImage.load(fname.c_str(), true)){
    cerr << " Skipping " << fname << endl;
    return false;
  }

  DepthImage scaledDepthImage;
  DepthImage::scale(scaledDepthImage, depthImage, reduction);
  imageRows = scaledDepthImage.rows();
  imageCols = scaledDepthImage.cols();
	
  converter.compute(*frame, scaledDepthImage, sensorOffset);
  traversabilityAnalyzer.createTraversabilityVector(frame->points(),
						    frame->normals(),
						    frame->traversabilityVector());
  return true;
}

void computeSensorOffsetAndK(Eigen::Isometry3f &sensorOffset, Eigen::Matrix3f &cameraMatrix, ParameterCamera *cameraParam, int reduction) {
  sensorOffset = Isometry3f::Identity();
  cameraMatrix.setZero();
      
  int cmax = 4;
  int rmax = 3;
  for (int c=0; c<cmax; c++){
    for (int r=0; r<rmax; r++){
      sensorOffset.matrix()(r, c) = cameraParam->offset()(r, c);
      if (c<3)
	cameraMatrix(r,c) = cameraParam->Kcam()(r, c);
    }
  }
  sensorOffset.translation() = Vector3f(0.15f, 0.0f, 0.05f);
  Quaternionf quat = Quaternionf(0.5, -0.5, 0.5, -0.5);
  sensorOffset.linear() = quat.toRotationMatrix();
  sensorOffset.matrix().block<1, 4>(3, 0) << 0.0f, 0.0f, 0.0f, 1.0f;
	
  float scale = 1./reduction;
  cameraMatrix *= scale;
  cameraMatrix(2,2) = 1;
}

void computeTraverse() {
  DepthImage depthImage, scaledDepthImage;
  pwn::PWNMapperController *controller = new pwn::PWNMapperController();
  g2o::HyperGraph::Vertex *_v = 0;
  controller->init(graph);
  while(true) {
    if(vertecesQueue.size() > 3) {
      _v = vertecesQueue.front();
      g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(_v);
      vertecesQueue.pop_front();
      if(!v)
	continue;
      
      if(!controller->addVertex(v))
	continue;

      controller->alignIncrementally();

      controller->computeTraversability();
    }
    else {
      usleep(20e3);
    }
  }
}

bool extractRelativePrior(Eigen::Isometry3f& priorMean, Matrix6f& priorInfo, 
			  VertexSE3* referenceVertex, VertexSE3* currentVertex) {
  bool priorFound = false;
  priorInfo.setZero();
  for (HyperGraph::EdgeSet::const_iterator it=referenceVertex->edges().begin();
       it!= referenceVertex->edges().end(); it++){
    const EdgeSE3* e = dynamic_cast<const EdgeSE3*>(*it);
    if (e->vertex(0)==referenceVertex && e->vertex(1) == currentVertex){
      priorFound=true;
      for (int c=0; c<6; c++)
	for (int r=0; r<6; r++)
	  priorInfo(r,c) = e->information()(r,c);

      for(int c=0; c<4; c++)
	for(int r=0; r<3; r++)
	  priorMean.matrix()(r,c) = e->measurement().matrix()(r,c);
      priorMean.matrix().row(3) << 0,0,0,1;
    }
  }
  return priorFound;
}


bool extractAbsolutePrior(Eigen::Isometry3f& priorMean, Matrix6f& priorInfo, 
			  VertexSE3* currentVertex){
  ImuData* imuData = 0;
  OptimizableGraph::Data* d = currentVertex->userData();
  while(d) {
    ImuData* imuData_ = dynamic_cast<ImuData*>(d);
    if (imuData_){
      imuData = imuData_;
    }
    d=d->next();
  }
	
  if (imuData){
    Eigen::Matrix3d R=imuData->getOrientation().matrix();
    Eigen::Matrix3d Omega = imuData->getOrientationCovariance().inverse();
    priorMean.setIdentity();
    priorInfo.setZero();
    for (int c = 0; c<3; c++)
      for (int r = 0; r<3; r++)
	priorMean.linear()(r,c)=R(r,c);
      
    for (int c = 0; c<3; c++)
      for (int r = 0; r<3; r++)
	priorInfo(r+3,c+3)=Omega(r,c);
    return true;
  }
  return false;
}

void writeQueue() {
  SensorData* data;
  std::ofstream ofG2O(&filename[0]);
  geometry_msgs::TransformStamped msg;
  int num = 0;

  
  // this is the vertex where we are packing the data
  g2o::VertexSE3* activeVertex = 0;
  // this is the timestamp of the first measurement added to the vertex
  double activeVertexTime=0;

  // this is the previous vertex
  g2o::VertexSE3* previousVertex = 0;
  // this is the timestamp of the first measurement added to the previous vertex
  double previousVertexTime=0;

  // the last position of the robot (not of the vertex, needed to measure the distances)
  Eigen::Isometry3d lastRobotPose;

  // set of sensors we packed in the current data. 
  // We do not want to put 10 camera images of the same camera in the same vertex.
  std::set<Sensor*> addedSensors;


  Eigen::Vector2d distances(0.,0);
  while (true)
  {
    if(! _queue.empty())
    {
      data = (SensorData*)_queue.front();
      double timeNow = _queue.lastElementTime();
      conditionalPrint(annoyingLevel) <<  "size=" << _queue.size() << " lastTime=" << FIXED(timeNow) << endl;
      if (timeNow - data->timeStamp()> initialDelay)
      { // we have enough stuff in the queue
	_queue.pop_front();
	if (! nptr->ok())
	  continue;

	tf::StampedTransform transform;
	bool we_got_transf = false;
	try
	{
	  ros::Time timeStamp;
	  // Get transformation
	  (*tfListener).lookupTransform("/odom", "/base_link", timeStamp.fromSec(data->timeStamp()), transform);
	  we_got_transf = true;
	}
	catch (tf::TransformException & ex)
	{
	  ROS_ERROR("%s", ex.what());
	}

	if (! we_got_transf)
	  continue;
				

	Eigen::Isometry3d currentRobotPose = fromStampedTransform(transform);
	double currentDataTime = data->timeStamp();
	distances += isometry2distance(lastRobotPose.inverse()*currentRobotPose);
	double passedTime = currentDataTime-previousVertexTime;
	lastRobotPose = currentRobotPose;

	conditionalPrint(annoyingLevel) << "distances: " << distances[0] << " " << distances[1] <<  " " << passedTime << endl;
	if (distances[0] < minDistances[0] && 
	    distances[1] < minDistances[1] && 
	    passedTime   < minTime){
	  conditionalPrint(annoyingLevel) << "reject: (time/distance)" << endl;
	  // SKIP THE FRAME
	  delete data;
	  data = 0;
	  continue;
	} 

	if (!activeVertex) {
	  activeVertex = new g2o::VertexSE3();
	  activeVertex->setId(num);
	  activeVertex->setEstimate(fromStampedTransform(transform));
	  activeVertexTime = currentDataTime;
	}
						
	Sensor* sensor = data->sensor();
	assert (sensor && "!");
						
	// check if we already packed the data for this kind of sensor
	if (addedSensors.count(sensor)){
	  conditionalPrint(annoyingLevel) << "reject: (sensor) "<< endl;
	  delete data;
	} else {
	  addedSensors.insert(sensor);
	  Parameter* parameter = sensor->parameter();
	  assert (parameter && "!@#");
	  //data->writeOut(filename);
	  if (! graph->parameters().getParameter(parameter->id())){
	    graph->parameters().addParameter(parameter);
	    graph->saveParameter(ofG2O, parameter);
	  }
					
	  activeVertex->addUserData(data);
	  data->setDataContainer(activeVertex);
	}
	// detach the data from the thing
	data = 0;

	if (currentDataTime - activeVertexTime > vertexTimeWindow) {
	  conditionalPrint(annoyingLevel) << "flush" << endl;
	  graph->addVertex(activeVertex);
	  graph->saveVertex(ofG2O, activeVertex);
	  				
	  if (previousVertex) {
	    EdgeSE3* e = new EdgeSE3();
	    e->setVertex(0, previousVertex);
	    e->setVertex(1, activeVertex);
	    e->setMeasurementFromState();
	    Eigen::Matrix<double, 6,6> m;
	    m.setIdentity();
	    e->setInformation(m);
	    graph->addEdge(e);
	    graph->saveEdge(ofG2O, e);

	    // JACP: do not do the remove, scan the data list and do a release() of the images which are big. The rest can stay in memory
	    g2o::HyperGraph::Data* d = previousVertex->userData();
	    while (d) {
	      RGBDData* rgbd = dynamic_cast<RGBDData*> (d);
	      if (rgbd)
		rgbd->release();
	      d = d->next();
	    }
	    vertecesQueue.push_back(previousVertex);
	  }
					
	  previousVertex = activeVertex;
	  previousVertexTime = activeVertexTime;
	  addedSensors.clear();
	  activeVertex = 0;
	  distances.setZero();
	  num++;
	  conditionalPrint(defaultLevel) << ".";
	}
      }
    }
    usleep(20e3); // queue is emp-ty
  }
}

int main(int argc, char** argv) {
  // Initialize ros
  g2o::CommandArgs arg;
  arg.param("dl", minDistances(0), 0.1, "translational sampling distance [m]");
  arg.param("dr", minDistances(1), 0.1, "rotational sampling distance [rad]");
  arg.param("dt", minTime, 1,   "temporal sampling distance [sec]");
  arg.param("it", minTime, 5,   "initial delay [sec]");
  arg.param("w",  vertexTimeWindow, 1./30, "interval to pack readings in the same vertex");
  arg.param("v",  verbose, 1, "verbose level");
  arg.paramLeftOver("graph-output", filename , "", "graph file which will be written", true);
  arg.parseArgs(argc, argv);
  if (filename==""){
    conditionalPrint(annoyingLevel) << "an output filename should be specified" << endl;
    return 0;
  }

  ros::init(argc, argv, "data_logger_node");
  ros::NodeHandle nh;
  nptr = &nh;

  // create the tf listener
  tfListener = new tf::TransformListener;
	
  // creare i sensors
  kinect = new SensorRGBDCamera();
  kinect->setTopics("/kinect/rgb/image_color","/kinect/depth_registered/image_raw");
  kinect->parameter()->setId(0);
  // creare i sensor handler e passargli il puntatore alla coda &queue
  shKinect = new SensorHandlerRGBDCamera(tfListener);
  // init dei sensor handler e calibration
  shKinect->setQueue(&_queue);
  shKinect->setSensor(kinect);
  shKinect->setNodeHandler(nptr);
  // registrazione su ROS delle callback dei sensor handler
  shKinect->registerCallback();

  laser = new SensorLaserRobot();
  laser->setTopic("/front_scan");
  laser->parameter()->setId(1);
  // creare i sensor handler e passargli il puntatore alla coda &queue
  shLaser = new SensorHandlerLaserRobot(tfListener);
  // init dei sensor handler e calibration
  shLaser->setQueue(&_queue);
  shLaser->setSensor(laser);
  shLaser->setNodeHandler(nptr);
  // registrazione su ROS delle callback dei sensor handler
  shLaser->registerCallback();
  
  // Create new Imu sensor
  imu = new SensorImu();
  imu->setDataTopic("/imu/data");
  imu->setMagneticTopic("/magnetic");
  imu->parameter()->setId(2);
  // Create new sensor handler
  shImu = new SensorHandlerImu(tfListener);
  shImu->setQueue(&_queue);
  shImu->setSensor(imu);
  shImu->setNodeHandler(nptr);
  shImu->registerCallback();

  // thread to process queue
  boost::thread thrd;
  thrd = boost::thread(writeQueue);
  // thread to process the vertex queue
  boost::thread thrdTraverse;
  thrdTraverse = boost::thread(computeTraverse);
  ros::spin();
  return (0);
}
