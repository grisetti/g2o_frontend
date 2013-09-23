#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <visualization_msgs/Marker.h>


#include "g2o_frontend/boss_logger/bframe.h"
#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"

#include "highgui.h"
#include <boost/bind.hpp>
#include <fstream>

#include <iostream>
using namespace std;
using namespace pwn;
using namespace boss;


  template <typename T1, typename T2>
  void convertScalar(T1& dest, const T2& src){
    for (int i=0; i<src.matrix().cols(); i++)
      for (int j=0; j<src.matrix().rows(); j++)
	dest.matrix()(j,i) = src.matrix()(j,i);
  }

struct PwnTracker{
  PwnTracker(ros::NodeHandle& nh_,   
	     tf::TransformListener* tfListener_, 
	     tf::TransformBroadcaster* tfBroadcaster_, 
	     std::string& topicName_,
	     ostream& os_): _nh(nh_), os(os_){
    _topic = topicName_;
    _imageTransport = new image_transport::ImageTransport(_nh);
    _cameraSubscriber = 0;
    _previousFrame = 0;
    _globalT.setIdentity();
    _previousFrameTransform.setIdentity();
    _scale = 2;
    _tfListener = tfListener_;
    _tfBroadcaster = tfBroadcaster_;
    _counter = 0;
    _numKeyframes = 0;
    m_odometry.type = visualization_msgs::Marker::LINE_STRIP;
    m_odometry.color.a = 1.0f;
    m_odometry.color.r = 1.0f;
    m_odometry.color.g = 0.0f;
    m_odometry.color.b = 1.0f;
    m_odometry.scale.x = 0.005; 
 }

  cv::Mat makeThumbnail(Frame* f, int r, int c, 
			const Eigen::Isometry3f& offset, 
			const Eigen::Matrix3f& cameraMatrix,
			float scale){
    PinholePointProjector proj;
    proj.setImageSize(r,c);
    proj.setCameraMatrix(cameraMatrix);
    proj.scale(scale);
    pwn::IntImage indices(proj.imageRows(), proj.imageCols());
    Eigen::MatrixXf depthBuffer(proj.imageRows(), proj.imageCols());
    proj.setTransform(offset);
    proj.project(indices, depthBuffer, f->points());
    cv::Mat thumb(proj.imageCols(), proj.imageRows(), CV_8UC3);
    for (int i = 0; i<indices.cols(); i++)
      for (int j = 0; j<indices.rows(); j++){
	cv::Vec3b& pixel = thumb.at<cv::Vec3b>(i,j);
	int idx = indices(j,i);
	pixel[0] = 0;
	pixel[1] = 0;
	pixel[2] = 0;
	if (idx==0)
	  continue;
	const pwn::Normal& n = f->normals()[idx];
	if (n.squaredNorm()<0.1)
	  continue;
	pixel[0] = 127*(n.x()-0.5);
	pixel[1] = 127*(n.y()-0.5);
	pixel[2] = 127*(n.z()-0.5);
      }
    return thumb;
  }

  void subscribe(){
    _cameraSubscriber = new image_transport::CameraSubscriber(_imageTransport->subscribeCamera(_topic,1,boost::bind(&PwnTracker::callback,this, _1, _2)));
    cerr << "subscribed image: [" <<  _topic << "]" << endl;
    _markerPub = _nh.advertise<visualization_msgs::Marker>("keyframes", 10);
    m_odometry.header.frame_id="/world";
  }

  virtual void callback(const sensor_msgs::Image::ConstPtr& img, const sensor_msgs::CameraInfo::ConstPtr& info) {
    Eigen::Isometry3f sensorOffset = Eigen::Isometry3f::Identity();
    try{
      tf::StampedTransform t;
      _tfListener->lookupTransform(_base_frame_id, img->header.frame_id, img->header.stamp, t);
      sensorOffset.translation().x()=t.getOrigin().x();
      sensorOffset.translation().y()=t.getOrigin().y();
      sensorOffset.translation().z()=t.getOrigin().z();
      Eigen::Quaternionf rot;
      rot.x()=t.getRotation().x();
      rot.y()=t.getRotation().y();
      rot.z()=t.getRotation().z();
      rot.w()=t.getRotation().w();
      sensorOffset.linear()=rot.toRotationMatrix();
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    }



    Eigen::Matrix3f cameraMatrix;
    int i=0;
    for (int r=0; r<3; r++)
      for (int c=0; c<3; c++, i++)
    	cameraMatrix(r,c) = info->K[i];
    cameraMatrix(2,2) = 1;

    // cameraMatrix << 
    // 570.342, 0,       320,
    // 0,       570.342, 240,
    // 0.0f, 0.0f, 1.0f;  
    
    cv_bridge::CvImagePtr ptr=cv_bridge::toCvCopy(img, img->encoding);
    DepthImage depthImage;
    depthImage.fromCvMat(ptr->image);
    
    
    PinholePointProjector* projector=dynamic_cast<PinholePointProjector*>(_converter->_projector);

    cameraMatrix(2,2)=1;
    projector->setCameraMatrix(cameraMatrix);
    projector->setImageSize(depthImage.rows(), depthImage.cols());

    DepthImage scaledImage;
    DepthImage::scale(scaledImage,depthImage,_scale);
    int r=scaledImage.rows();
    int c=scaledImage.cols();
    projector->scale(1.0/_scale);
    pwn::Frame* frame = new pwn::Frame;
    _converter->compute(*frame,scaledImage, sensorOffset);
    // scale image and camera matrix


    bool newFrame = false;
    if (_previousFrame){
      _aligner->setCurrentSensorOffset(sensorOffset);
      _aligner->setCurrentFrame(frame);
      
      _aligner->correspondenceFinder()->setImageSize(r,c);
      PinholePointProjector* alprojector=(PinholePointProjector*)(_aligner->projector());
      alprojector->setCameraMatrix(projector->cameraMatrix());
      alprojector->setImageSize(projector->imageRows(), projector->imageCols());

      Eigen::Isometry3f guess=_previousFrameTransform.inverse()*_globalT;
      _aligner->setInitialGuess(guess);
      _aligner->align();
 
      // cerr << "inliers: " << _aligner->inliers() << endl;
      // cerr << "chi2: " << _aligner->error() << endl;
      // cerr << "chi2/inliers: " << _aligner->error()/_aligner->inliers() << endl;
      // cerr << "initialGuess: " << t2v(guess).transpose() << endl;
      // cerr << "transform   : " << t2v(_aligner->T()).transpose() << endl;
      if (_aligner->inliers()>-1){
    	_globalT = _previousFrameTransform*_aligner->T();
    	//cerr << "TRANSFORM FOUND" <<  endl;
      } else {
    	//cerr << "FAILURE" <<  endl;
    	_globalT = _globalT*guess;
      }
      if (! (_counter%50) ) {
    	Eigen::Matrix3f R = _globalT.linear();
    	Eigen::Matrix3f E = R.transpose() * R;
    	E.diagonal().array() -= 1;
    	_globalT.linear() -= 0.5 * R * E;
      }
      _globalT.matrix().row(3) << 0,0,0,1;

      geometry_msgs::TransformStamped st;
      {
	st.header.frame_id = "/world";
	st.child_frame_id =  _base_frame_id;
	st.header.stamp = img->header.stamp;
	st.transform.translation.x = _globalT.translation().x();
	st.transform.translation.y = _globalT.translation().y();
	st.transform.translation.z = _globalT.translation().z();
	Eigen::Quaternionf rot(_globalT.linear());
	st.transform.rotation.x = rot.x();
	st.transform.rotation.y = rot.y();
	st.transform.rotation.z = rot.z();
	st.transform.rotation.w = rot.w();
	_tfBroadcaster->sendTransform(st);
      }
 
      int maxInliers = r*c;
      float inliersFraction = (float) _aligner->inliers()/(float) maxInliers;
      if (inliersFraction<0.6){
	newFrame = true;
	geometry_msgs::Point p;
	Eigen::Isometry3f t = _globalT;
	p.x = t.translation().x();
	p.y = t.translation().y();
	p.z = t.translation().z();
	m_odometry.points.push_back(p);

	char filename[1024];
	sprintf (filename, "frame-%05d.pwn", _numKeyframes);
	frame->save(filename,1,true,_globalT);

	_numKeyframes ++;
	delete _previousFrame;
	_aligner->setReferenceSensorOffset(sensorOffset);
	_aligner->setReferenceFrame(frame);
	_previousFrame = frame;
	_previousFrameTransform = _globalT;
	cerr << "new frame added (" << _numKeyframes <<  ")" << endl;
	cerr << "inliers: " << _aligner->inliers() << endl;
	cerr << "maxInliers: " << maxInliers << endl;
	cerr << "chi2: " << _aligner->error() << endl;
	cerr << "chi2/inliers: " << _aligner->error()/_aligner->inliers() << endl;
	cerr << "initialGuess: " << t2v(guess).transpose() << endl;
	cerr << "transform   : " << t2v(_aligner->T()).transpose() << endl;
	cerr << "globalTransform   : " << t2v(_globalT).transpose() << endl;
      } else {
	delete frame;
      }
    } else {
      newFrame = true;
      _aligner->setReferenceSensorOffset(sensorOffset);
      _aligner->setReferenceFrame(frame);
      _previousFrame = frame;
      _previousFrameTransform = _globalT;
      _numKeyframes ++;
      /*Eigen::Isometry3f t = _globalT;
      geometry_msgs::Point p;
      p.x = t.translation().x();
      p.y = t.translation().y();
      p.z = t.translation().z();
      m_odometry.points.push_back(p);
      */

    }
    _counter++;

    if (newFrame) {
      char filename [1024];
      os << "\"KeyFrame\"  { \"id\": " << _numKeyframes << ", ";
      os << "\"offset\":  [ " << t2v(sensorOffset).transpose() << " ], ";
      os << "\"globalT\": [ " << t2v(_globalT).transpose() << " ], ";
      sprintf(filename, "img-%05d.pgm", _numKeyframes);
      depthImage.save(filename, true);
      os << "\"image\": \"" << filename << "\", ";
      sprintf (filename, "frame-%05d.pwn", _numKeyframes);
      frame->save(filename,1,true,_globalT);
      os << "\"cloud\": \"" << filename << "\" ";
      cv::Mat thumbnail = makeThumbnail(frame, 
					depthImage.rows(), 
					depthImage.cols(),
					sensorOffset,
					cameraMatrix, 
					0.1);
      sprintf(filename, "thumbnail-%05d.pbm", _numKeyframes);
      os << "\"thumbnail\": \"" << filename << "\" }" << endl;
      cerr << "thumbnail size: " << thumbnail.rows << " " << thumbnail.cols << endl;
      cv::imwrite(filename, thumbnail);
    }
    m_odometry.header.stamp = ros::Time::now();
    _markerPub.publish(m_odometry);
  }

  virtual ~PwnTracker() {
    delete _imageTransport;
    if (_cameraSubscriber)
      delete _cameraSubscriber;
  }  
  
  
  ros::NodeHandle& _nh;
  image_transport::ImageTransport * _imageTransport;
  image_transport::CameraSubscriber *_cameraSubscriber;

  Aligner* _aligner;
  DepthImageConverter* _converter;
  int _scale;
  pwn::Frame* _previousFrame;
  Eigen::Isometry3f _previousFrameTransform;
  std::string _topic;
  std::string _base_frame_id;
  Eigen::Isometry3f _globalT;
  tf::TransformListener* _tfListener;
  tf::TransformBroadcaster* _tfBroadcaster;
  int _counter;
  int _numKeyframes;
  visualization_msgs::Marker m_odometry; 
  ros::Publisher _markerPub; 
  ostream& os;
};


std::vector<Serializable*> readConfig(Aligner*& aligner, DepthImageConverter*& converter, const std::string& configFile){
  aligner = 0;
  converter = 0;
  Deserializer des;
  des.setFilePath(configFile);
  Serializable* s;
  std::vector<Serializable*> instances;
  cerr << "Reading" << endl;
  while ((s=des.readObject())){
    instances.push_back(s);
    Aligner* al=dynamic_cast<Aligner*>(s);
    if (al) {
      cerr << "got aligner" << endl;
      aligner = al;
    }
    DepthImageConverter* conv=dynamic_cast<DepthImageConverter*>(s);
    if  (conv) {      
      cerr << "got converter" << endl;
      converter = conv;
    }
  }
  if (aligner) {
    cerr << "alpp: " << aligner->projector() << endl;
    cerr << "allz: " << aligner->linearizer() << endl;
    if (aligner->linearizer())
      cerr << "lzal: " << aligner->linearizer()->aligner() << endl;
    
  }

  return instances;
}


int main(int argc, char** argv){
  if (argc<2) {
    cerr << " u should provide a config file" << endl;
    return 0;
  }

  Aligner* aligner;
  DepthImageConverter* converter;
  std::vector<Serializable*> instances = readConfig(aligner, converter, argv[1]);

  ros::init(argc, argv, "pwn_tracker");
  ros::NodeHandle nh;
  tf::TransformListener* tfListener = new tf::TransformListener(nh, ros::Duration(30.0));
  tf::TransformBroadcaster* tfBroadcaster = new tf::TransformBroadcaster();

  std::string _topic = "/camera/depth_registered/image_rect_raw";
  ofstream os("tracker.log");
  PwnTracker* tracker = new PwnTracker(nh, tfListener, tfBroadcaster, _topic, os);
  tracker->_aligner = aligner;
  tracker->_converter = converter;
  tracker->_base_frame_id = "/camera_link";
  tracker->subscribe();
  while (ros::ok()){
    ros::spinOnce();
  }
}
