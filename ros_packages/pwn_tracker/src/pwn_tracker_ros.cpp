#include "pwn_tracker_ros.h"


namespace pwn_tracker{

  using namespace std;
  using namespace pwn;
  using namespace boss;

  PwnTrackerRos::PwnTrackerRos(ros::NodeHandle& nh_,   
			       tf::TransformListener* tfListener_, 
			       tf::TransformBroadcaster* tfBroadcaster_, 
			       std::string& topicName_,
			       const std::string& filename,
			       pwn::Aligner* aligner, 
			       pwn::DepthImageConverter* converter, 
			       boss_map::MapManager* manager): PwnTracker(aligner, converter, manager), _nh(nh_){
    _topic = topicName_;
    _imageTransport = new image_transport::ImageTransport(_nh);
    _cameraSubscriber = 0;
    _tfListener = tfListener_;
    _tfBroadcaster = tfBroadcaster_;
    _filename = filename;
  }
  
  PwnTrackerRos::~PwnTrackerRos() {
    delete _imageTransport;
    if (_cameraSubscriber)
      delete _cameraSubscriber;
  }

  void PwnTrackerRos::subscribe() {
    _cameraSubscriber = new image_transport::CameraSubscriber(_imageTransport->subscribeCamera(_topic,1,boost::bind(&PwnTrackerRos::callback,this, _1, _2)));
    cerr << "subscribed image: [" <<  _topic << "]" << endl;
  }

  void PwnTrackerRos::broadcastTransform(const sensor_msgs::Image::ConstPtr& img) {
    geometry_msgs::TransformStamped st;
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

  bool PwnTrackerRos::retrieveImageParameters(Eigen::Isometry3f& sensorOffset, 
					      Eigen::Matrix3f& cameraMatrix, 
					      const sensor_msgs::Image::ConstPtr& img,
					      const sensor_msgs::CameraInfo::ConstPtr& info) {
    sensorOffset = Eigen::Isometry3f::Identity();
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
      return false;
    }
    int i=0;
    for (int r=0; r<3; r++)
      for (int c=0; c<3; c++, i++)
    	cameraMatrix(r,c) = info->K[i];
    cameraMatrix(2,2) = 1;
    
    std::cout << "Sensor offset: " << t2v(sensorOffset).transpose() << endl;

    sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

    // cameraMatrix << 
    //   525.0f,   0.0f, 319.5f,
    //   0.0f, 525.0f, 239.5f,
    //   0.0f,   0.0f,   1.0f;  

    return true;
  }

  void PwnTrackerRos::callback(const sensor_msgs::Image::ConstPtr& img, const sensor_msgs::CameraInfo::ConstPtr& info) {

    
    Eigen::Isometry3f sensorOffset;
    Eigen::Matrix3f cameraMatrix;
    if (!retrieveImageParameters(sensorOffset, cameraMatrix, img, info))
      return;
    
    cv_bridge::CvImagePtr ptr=cv_bridge::toCvCopy(img, img->encoding);
    DepthImage depthImage;
    if(img->encoding == "32FC1")
      depthImage.fromCvMat32FC1(ptr->image, 1.0f);
    else
      depthImage.fromCvMat(ptr->image);

    processFrame(depthImage, sensorOffset, cameraMatrix);

  }


  void PwnTrackerRos::newFrameCallback(PwnTrackerFrame* frame) {
    cerr << "newFrame: " << frame << endl;
    ser.writeObject(*frame);
  }
  void PwnTrackerRos::newRelationCallback(PwnTrackerRelation* relation){
    ser.writeObject(*relation);
  }

  void PwnTrackerRos::newAlignmentCallback(const Eigen::Isometry3f& globalT, 
					   const Eigen::Isometry3f& localT, 
					   int inliers, float error ){
    cerr << "align: " << t2v(_globalT).transpose() << endl;
  }
  
  void PwnTrackerRos::initCallback(){
    ser.setFilePath(_filename);
    ser.writeObject(*_manager);
  }
}
