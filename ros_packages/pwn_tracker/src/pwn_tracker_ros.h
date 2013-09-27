#ifndef _PWN_TRACKER_ROS_
#define _PWN_TRACKER_ROS_

#include "pwn_tracker.h"
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
#include <boost/bind.hpp>

namespace pwn_tracker{

using namespace std;
using namespace boss;
using namespace boss_logger;
using namespace boss_map;
using namespace pwn;



  struct PwnTrackerRos: public PwnTracker{
    PwnTrackerRos(ros::NodeHandle& nh_,   
	       tf::TransformListener* tfListener_, 
	       tf::TransformBroadcaster* tfBroadcaster_, 
	       std::string& topicName_,
	       const std::string& filename,
	       pwn::Aligner* aligner, 
	       pwn::DepthImageConverter* converter, 
	       boss_map::MapManager* manager);
    virtual ~PwnTrackerRos();  
    void subscribe();
    void broadcastTransform(const sensor_msgs::Image::ConstPtr& img);
    bool retrieveImageParameters(Eigen::Isometry3f& sensorOffset, 
				 Eigen::Matrix3f& cameraMatrix, 
				 const sensor_msgs::Image::ConstPtr& img,
				 const sensor_msgs::CameraInfo::ConstPtr& info);

    virtual void callback(const sensor_msgs::Image::ConstPtr& img, const sensor_msgs::CameraInfo::ConstPtr& info);


    virtual void newFrameCallback(PwnTrackerFrame* frame);
    virtual void newRelationCallback(PwnTrackerRelation* relation);
    virtual void initCallback();
    void newAlignmentCallback(const Eigen::Isometry3f& globalT, 
			      const Eigen::Isometry3f& localT, 
			      int inliers, float error );
    

    ros::NodeHandle& _nh;
    std::string _filename, _topic, _base_frame_id;
    image_transport::ImageTransport * _imageTransport;
    image_transport::CameraSubscriber *_cameraSubscriber;
    tf::TransformListener* _tfListener;
    tf::TransformBroadcaster* _tfBroadcaster;
    visualization_msgs::Marker m_odometry; 
    ros::Publisher _markerPub; 
    MapManager* manager;
    Serializer ser;
  };


}// end namespace

#endif
