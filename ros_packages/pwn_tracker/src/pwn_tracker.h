#include "g2o_frontend/boss_map/boss_map.h"

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
#include "g2o_frontend/boss_logger/bimagesensor.h"
//#include "g2o_frontend/boss_map/boss_map.h"

#include "highgui.h"
#include <boost/bind.hpp>
#include <fstream>
#include <iostream>

namespace pwn_tracker{

using namespace std;
using namespace boss;
using namespace boss_logger;
using namespace boss_map;
using namespace pwn;


  struct PwnTrackerFrame: public boss_map::MapNode {
    PwnTrackerFrame (MapManager* manager=0, int id=-1, IdContext* context = 0);
    //! boss serialization
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialization
    virtual void deserialize(ObjectData& data, IdContext& context);


    pwn::FrameBLOBReference cloud;
    boss_logger::ImageBLOBReference depthImage;
    int imageRows, imageCols;
    boss_logger::ImageBLOBReference normalThumbnail;
    boss_logger::ImageBLOBReference depthThumbnail;
    Eigen::Isometry3f sensorOffset;
    Eigen::Matrix3f cameraMatrix;
    float scale;
  };


  struct PwnTrackerRelation: public MapNodeBinaryRelation{
    PwnTrackerRelation(MapManager* manager=0, int id=-1, IdContext* context = 0);
    //! boss serialization
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialization
    virtual void deserialize(ObjectData& data, IdContext& context);
    //! called when all links are resolved, adjusts the bookkeeping of the parents
    inline PwnTrackerFrame* from() { return static_cast<PwnTrackerFrame*>(_nodes[0]); }
    inline PwnTrackerFrame* to()   { return static_cast<PwnTrackerFrame*>(_nodes[1]); }
    void setFrom(PwnTrackerFrame* from_) {_nodes[0] = from_; }
    void setTo(PwnTrackerFrame* to_) {_nodes[1] = to_; }
    int inliers;
    float error;
  };


  struct PwnTracker{
    PwnTracker(ros::NodeHandle& nh_,   
	       tf::TransformListener* tfListener_, 
	       tf::TransformBroadcaster* tfBroadcaster_, 
	       std::string& topicName_,
	       const std::string& filename);

    void makeThumbnails(cv::Mat& depthThumbnail, cv::Mat& normalThumbnail, 
			Frame* f, int r, int c, 
			const Eigen::Isometry3f& offset, 
			const Eigen::Matrix3f& cameraMatrix,
			float scale);

    pwn::Frame* makeCloud(int& r, int& c,
			  Eigen::Matrix3f& cameraMatrix, 
			  const Eigen::Isometry3f& sensorOffset, 
			  const DepthImage& depthImage);
    void subscribe();
    void broadcastTransform(const sensor_msgs::Image::ConstPtr& img);
    bool retrieveImageParameters(Eigen::Isometry3f& sensorOffset, 
				 Eigen::Matrix3f& cameraMatrix, 
				 const sensor_msgs::Image::ConstPtr& img,
				 const sensor_msgs::CameraInfo::ConstPtr& info);

    virtual void callback(const sensor_msgs::Image::ConstPtr& img, const sensor_msgs::CameraInfo::ConstPtr& info);

    virtual ~PwnTracker();  
  
    ros::NodeHandle& _nh;
    image_transport::ImageTransport * _imageTransport;
    image_transport::CameraSubscriber *_cameraSubscriber;

    Aligner* _aligner;
    DepthImageConverter* _converter;
    int _scale;
    pwn::Frame* _previousCloud;
    PwnTrackerFrame* _previousTrackerFrame;
    Eigen::Isometry3f _previousCloudTransform;
    std::string _topic;
    std::string _base_frame_id;
    Eigen::Isometry3f _globalT;
    tf::TransformListener* _tfListener;
    tf::TransformBroadcaster* _tfBroadcaster;
    int _counter;
    int _numKeyframes;
    visualization_msgs::Marker m_odometry; 
    ros::Publisher _markerPub; 
    MapManager* manager;
    Serializer ser;
  };

  std::vector<Serializable*> readConfig(Aligner*& aligner, DepthImageConverter*& converter, const std::string& configFile);
}// end namespace
